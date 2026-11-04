const model = {
  state: null,
  candidates: [],
  clouds: { product_cloud: null, scene_cloud: null },
  versions: {},
  selected: null,
  view: { yaw: -0.72, pitch: 0.35, zoom: 1.0 },
};

const $ = (id) => document.getElementById(id);
const fmt = (value, digits = 3) =>
  Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "-";
const mm = (value) =>
  Number.isFinite(Number(value)) ? `${(Number(value) * 1000).toFixed(1)} mm` : "-";
const poseText = (pose) => {
  if (!Array.isArray(pose?.position) || !Array.isArray(pose?.orientation)) return "-";
  const xyz = pose.position.map((value) => fmt(value, 4)).join(", ");
  const quat = pose.orientation.map((value) => fmt(value, 4)).join(", ");
  return `${pose.frame_id || "?"}  xyz [${xyz}] m  q [${quat}]`;
};

async function getJson(path) {
  const response = await fetch(`${path}?t=${Date.now()}`, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${path}`);
  return response.json();
}

function setImage(name, path, version) {
  if (!version || model.versions[name] === version) return;
  model.versions[name] = version;
  const image = $(name);
  const empty = $(`${name}-empty`);
  image.onload = () => empty.classList.add("hidden");
  image.onerror = () => empty.classList.remove("hidden");
  image.src = `${path}?v=${version}`;
}

function updateTile(name, data) {
  const tile = $(`tile-${name}`);
  const state = data?.state || "waiting";
  tile.className = `status-tile ${state}`;
  tile.querySelector(".tile-state").textContent =
    ({ ok: "正常", running: "运行中", error: "失败", waiting: "等待" })[state] || state;
  tile.querySelector(".tile-message").textContent = data?.message || "尚未收到结果";
}

function updateStatus(state) {
  $("target").textContent = state.status?.target || "等待商品";
  updateTile("segmentation", state.status?.segmentation);
  updateTile("grasp", state.status?.grasp);
  updateTile("execution", state.status?.execution);
  const connection = $("connection");
  connection.textContent = "已连接";
  connection.className = "connection ok";

  const segmentation = state.status?.segmentation?.payload || {};
  $("image-frame").textContent = segmentation.frame_id || "等待图像";
  const frame =
    state.clouds?.product_cloud?.frame_id || state.clouds?.scene_cloud?.frame_id || "";
  $("cloud-frame").textContent = frame || "等待点云";
  $("frame-id").textContent = frame || "等待数据";

  const grasp = state.status?.grasp?.payload || {};
  $("result-reason").textContent =
    grasp.reason || state.status?.grasp?.message || "等待结果";
  $("result-center").textContent = Array.isArray(grasp.center_m)
    ? grasp.center_m.map(mm).join(" / ")
    : "-";
  $("result-axis").textContent = Array.isArray(grasp.approach_axis)
    ? grasp.approach_axis.map((value) => fmt(value, 3)).join(" / ")
    : "-";
  $("result-width").textContent = grasp.required_width_m
    ? mm(grasp.required_width_m)
    : "-";
  if (Number.isFinite(Number(grasp.candidate_count))) {
    $("candidate-total").textContent = `${Number(grasp.candidate_count)} candidates`;
    $("constrained").textContent = Number(grasp.constrained_candidate_count || 0);
  }
  $("object-pose").textContent = poseText(
    grasp.object_pose || segmentation.object_pose,
  );
  $("pregrasp-pose").textContent = poseText(grasp.pregrasp_pose);
  $("grasp-pose").textContent = poseText(grasp.grasp_pose);
  $("capture-stamp").textContent = segmentation.stamp_ns
    ? `${(Number(segmentation.stamp_ns) / 1e9).toFixed(6)} s`
    : "-";
}

function updateDiagnostics(summary) {
  $("candidate-total").textContent = `${summary.candidate_count || 0} candidates`;
  $("near-center").textContent = summary.near_center_count || 0;
  $("aligned").textContent = summary.approach_aligned_count || 0;
  $("width-ok").textContent = summary.width_feasible_count || 0;
  $("constrained").textContent = summary.fit_contributor_count || 0;
  $("constraint-note").textContent =
    `软权重尺度：中心 ${mm(summary.max_center_distance_m)} · ` +
    `方向 ${fmt(summary.max_approach_angle_deg, 1)} deg · ` +
    `夹爪宽度上限 ${mm(summary.max_width_m)}`;
}

function candidateClass(candidate) {
  if (candidate.fit_contributor) return ["accepted", "拟合"];
  if (candidate.reasons?.includes("width")) return ["width", "宽度"];
  return ["other", "低权重"];
}

function updateCandidates(payload) {
  model.candidates = payload.candidates || [];
  updateDiagnostics(payload);
  const body = $("candidate-rows");
  body.replaceChildren();
  if (!model.candidates.length) {
    body.innerHTML = '<tr><td colspan="6" class="table-empty">等待候选数据</td></tr>';
    return;
  }
  $("canvas-empty").classList.add("hidden");
  model.candidates.forEach((candidate, index) => {
    const row = document.createElement("tr");
    if (model.selected === index) row.classList.add("selected");
    const [klass, label] = candidateClass(candidate);
    row.innerHTML =
      `<td><span class="badge ${klass}">${label}</span></td>` +
      `<td>${fmt(candidate.score, 3)}</td>` +
      `<td>${mm(candidate.width_m)}</td>` +
      `<td>${mm(candidate.center_distance_m)}</td>` +
      `<td>${fmt(candidate.approach_angle_deg, 1)} deg</td>` +
      `<td>${(candidate.reasons || []).join(", ") || "-"}</td>`;
    row.addEventListener("click", () => {
      model.selected = index;
      updateCandidates(payload);
      drawCloud();
    });
    body.appendChild(row);
  });
}

async function updateCloud(kind, version) {
  if (!version || model.versions[kind] === version) return;
  const endpoint = kind === "product_cloud" ? "/api/cloud/product" : "/api/cloud/scene";
  model.clouds[kind] = await getJson(endpoint);
  model.versions[kind] = version;
  $("canvas-empty").classList.add("hidden");
}

function cloudCenter() {
  const selected = model.selected == null ? null : model.candidates[model.selected];
  if (selected?.center) return selected.center;
  const targetCenter = model.state?.candidates?.target_center;
  if (Array.isArray(targetCenter)) return targetCenter;
  const points = model.clouds.product_cloud?.points || [];
  if (!points.length) return [0, 0, 0];
  return points
    .reduce((sum, point) => sum.map((value, index) => value + point[index]), [0, 0, 0])
    .map((value) => value / points.length);
}

function project(point, center, scale, width, height) {
  const x = point[0] - center[0];
  const y = point[1] - center[1];
  const z = point[2] - center[2];
  const cy = Math.cos(model.view.yaw);
  const sy = Math.sin(model.view.yaw);
  const cp = Math.cos(model.view.pitch);
  const sp = Math.sin(model.view.pitch);
  const rotatedX = cy * x + sy * z;
  const rotatedZ = -sy * x + cy * z;
  const rotatedY = cp * y - sp * rotatedZ;
  const depth = sp * y + cp * rotatedZ;
  return {
    x: width / 2 + rotatedX * scale * model.view.zoom,
    y: height / 2 - rotatedY * scale * model.view.zoom,
    depth,
  };
}

function drawLine(context, start, end, color, width, center, scale, canvas) {
  const first = project(start, center, scale, canvas.width, canvas.height);
  const second = project(end, center, scale, canvas.width, canvas.height);
  context.strokeStyle = color;
  context.lineWidth = width;
  context.beginPath();
  context.moveTo(first.x, first.y);
  context.lineTo(second.x, second.y);
  context.stroke();
}

function rotateVector(vector, quaternion) {
  if (!Array.isArray(quaternion) || quaternion.length !== 4) return vector;
  const [qx, qy, qz, qw] = quaternion.map(Number);
  const [vx, vy, vz] = vector;
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return [
    vx + qw * tx + qy * tz - qz * ty,
    vy + qw * ty + qz * tx - qx * tz,
    vz + qw * tz + qx * ty - qy * tx,
  ];
}

function drawPose(context, pose, color, label, center, scale, canvas, ratio) {
  if (!Array.isArray(pose?.position)) return;
  const origin = pose.position;
  const axisLength = 0.045;
  const axes = [
    [[axisLength, 0, 0], "#ef6868"],
    [[0, axisLength, 0], "#5ce48a"],
    [[0, 0, axisLength], "#68a9ee"],
  ];
  axes.forEach(([axis, axisColor]) => {
    const rotated = rotateVector(axis, pose.orientation);
    const end = origin.map((value, index) => value + rotated[index]);
    drawLine(context, origin, end, axisColor, 2.2 * ratio, center, scale, canvas);
  });
  const point = project(origin, center, scale, canvas.width, canvas.height);
  context.strokeStyle = color;
  context.lineWidth = 2.5 * ratio;
  context.strokeRect(point.x - 5 * ratio, point.y - 5 * ratio, 10 * ratio, 10 * ratio);
  context.fillStyle = color;
  context.font = `${12 * ratio}px sans-serif`;
  context.fillText(label, point.x + 8 * ratio, point.y - 7 * ratio);
}

function drawCloud() {
  const canvas = $("cloud-canvas");
  const rectangle = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rectangle.width * ratio));
  canvas.height = Math.max(1, Math.floor(rectangle.height * ratio));
  const context = canvas.getContext("2d");
  context.fillStyle = "#172328";
  context.fillRect(0, 0, canvas.width, canvas.height);

  const product = model.clouds.product_cloud?.points || [];
  const scene = model.clouds.scene_cloud?.points || [];
  if (!product.length && !scene.length && !model.candidates.length) return;
  const center = cloudCenter();
  let span = 0.25;
  [...product, ...scene].forEach((point) => {
    point.forEach((value, index) => {
      span = Math.max(span, Math.abs(value - center[index]) * 2);
    });
  });
  const scale = (Math.min(canvas.width, canvas.height) * 0.76) / span;
  const projected = [];
  scene.forEach((point) =>
    projected.push({ point, color: "rgba(154,175,178,.38)", size: 1.6 }));
  product.forEach((point) =>
    projected.push({ point, color: "rgba(48,220,210,.92)", size: 2.7 }));
  projected.sort(
    (left, right) =>
      project(left.point, center, scale, canvas.width, canvas.height).depth -
      project(right.point, center, scale, canvas.width, canvas.height).depth,
  );
  projected.forEach((item) => {
    const point = project(item.point, center, scale, canvas.width, canvas.height);
    context.fillStyle = item.color;
    context.fillRect(point.x, point.y, item.size * ratio, item.size * ratio);
  });

  model.candidates.forEach((candidate, index) => {
    if (!Array.isArray(candidate.center)) return;
    const color = candidate.fit_contributor
      ? "#5ce48a"
      : candidate.reasons?.includes("width")
        ? "#ef6868"
        : "#efaa4c";
    const axis = candidate.approach || [1, 0, 0];
    const start = candidate.center.map((value, component) => value - axis[component] * 0.035);
    drawLine(
      context,
      start,
      candidate.center,
      color,
      (model.selected === index ? 4 : 1.5) * ratio,
      center,
      scale,
      canvas,
    );
    const point = project(candidate.center, center, scale, canvas.width, canvas.height);
    context.fillStyle = color;
    context.beginPath();
    context.arc(point.x, point.y, (model.selected === index ? 5 : 2.5) * ratio, 0, Math.PI * 2);
    context.fill();
  });

  const poses = model.state?.path?.poses || [];
  if (poses.length >= 2) {
    drawLine(context, poses[0].position, poses[1].position, "#ffe36d", 3 * ratio, center, scale, canvas);
  }
  const segmentation = model.state?.status?.segmentation?.payload || {};
  const grasp = model.state?.status?.grasp?.payload || {};
  drawPose(
    context,
    grasp.object_pose || segmentation.object_pose,
    "#f7f7f7",
    "OBJECT",
    center,
    scale,
    canvas,
    ratio,
  );
  drawPose(context, grasp.pregrasp_pose, "#f0b84d", "PRE", center, scale, canvas, ratio);
  drawPose(context, grasp.grasp_pose, "#e85f77", "GRASP", center, scale, canvas, ratio);
  [
    ["X", [0.06, 0, 0], "#ef6868"],
    ["Y", [0, 0.06, 0], "#5ce48a"],
    ["Z", [0, 0, 0.06], "#68a9ee"],
  ].forEach(([label, delta, color]) => {
    const end = center.map((value, index) => value + delta[index]);
    drawLine(context, center, end, color, 2 * ratio, center, scale, canvas);
    const point = project(end, center, scale, canvas.width, canvas.height);
    context.fillStyle = color;
    context.font = `${12 * ratio}px sans-serif`;
    context.fillText(label, point.x + 4 * ratio, point.y);
  });
}

async function refresh() {
  try {
    const state = await getJson("/api/state");
    model.state = state;
    updateStatus(state);
    setImage("rgb", "/api/image/rgb.jpg", state.versions.rgb);
    setImage("overlay", "/api/image/overlay.jpg", state.versions.overlay);
    setImage("mask", "/api/image/mask.png", state.versions.mask);
    await Promise.all([
      updateCloud("product_cloud", state.versions.product_cloud),
      updateCloud("scene_cloud", state.versions.scene_cloud),
    ]);
    if (model.versions.candidates !== state.versions.candidates) {
      const payload = await getJson("/api/candidates");
      model.versions.candidates = state.versions.candidates;
      updateCandidates(payload);
    }
    drawCloud();
  } catch (error) {
    const connection = $("connection");
    connection.textContent = "未连接";
    connection.className = "connection error";
  }
}

document.addEventListener("DOMContentLoaded", () => {
  $("reset-view").addEventListener("click", () => {
    model.view = { yaw: -0.72, pitch: 0.35, zoom: 1.0 };
    drawCloud();
  });
  const canvas = $("cloud-canvas");
  let dragging = false;
  let previous = null;
  canvas.addEventListener("pointerdown", (event) => {
    dragging = true;
    previous = [event.clientX, event.clientY];
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    model.view.yaw += (event.clientX - previous[0]) * 0.01;
    model.view.pitch = Math.max(
      -1.4,
      Math.min(1.4, model.view.pitch + (event.clientY - previous[1]) * 0.01),
    );
    previous = [event.clientX, event.clientY];
    drawCloud();
  });
  canvas.addEventListener("pointerup", () => { dragging = false; });
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      model.view.zoom = Math.max(
        0.25,
        Math.min(5, model.view.zoom * (event.deltaY < 0 ? 1.1 : 0.9)),
      );
      drawCloud();
    },
    { passive: false },
  );
  window.addEventListener("resize", drawCloud);
  refresh();
  window.setInterval(refresh, 700);
});
