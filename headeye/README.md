# OpenFlex 左右腕手眼标定结果归档

本目录保存 2026-08-09 得到的左右腕 D405 手眼标定结果，供未来抓取、感知坐标转换或重新验证使用。

本归档不包含任何工作空间专用的 URDF、固定关节或部署参数。未来使用者应根据目标系统实际 TF 树决定如何
加载这两份变换。

## 文件

```text
left_link7_handeye.calib       左腕最终标定结果
left_link7_handeye.samples     左腕求解使用的 25 组样本
right_link7_handeye.calib      右腕最终标定结果
right_link7_handeye.samples    右腕求解使用的 25 组样本
SHA256SUMS                     上述四个原始文件的校验和
README.md                      本说明
```

原始文件位置：

```text
/home/openflex/.ros2/easy_handeye2/calibrations/left_link7_handeye.calib
/home/openflex/.ros2/easy_handeye2/samples/left_link7_handeye.samples
/home/openflex/.ros2/easy_handeye2/calibrations/right_link7_handeye.calib
/home/openflex/.ros2/easy_handeye2/samples/right_link7_handeye.samples
```

本目录中的四个文件是以上原文件的逐字节副本。

## `.calib` 记录了什么

`.calib` 是 Easy HandEye 2 保存的 YAML 文本，主要包含：

- `parameters.name`：标定名称。
- `parameters.calibration_type`：本结果为 `eye_in_hand`。
- `robot_base_frame`：求解时使用的机器人基座 frame。
- `robot_effector_frame`：相机安装端的机器人 frame。
- `tracking_base_frame`：相机光学 frame。
- `tracking_marker_frame`：标定板/ArUco frame。
- `transform.translation`：平移，单位为米。
- `transform.rotation`：四元数，字段顺序为 `x, y, z, w`。

其中真正的标定结果是 `transform`。其语义都是父坐标系到子坐标系的固定变换。

### 左腕

```text
parent: openarmx_left_link7
child:  left_camera_color_optical_frame

translation_xyz_m:
  [0.09555859171638514,
  -0.015860000793077835,
   0.11456113647136229]

quaternion_xyzw:
  [-0.1813082786012135,
  -0.22729225173182688,
   0.6887430937975637,
   0.6641524607795263]
```

### 右腕

```text
parent: openarmx_right_link7
child:  right_camera_color_optical_frame

translation_xyz_m:
  [0.08302836068369625,
  -0.021981950606194654,
   0.0768365654910409]

quaternion_xyzw:
  [-0.16111313833100968,
  -0.2314817307760766,
   0.672448748864902,
   0.6843036205691692]
```

注意：四元数 `[x, y, z, w]` 与整体取反后的 `[-x, -y, -z, -w]` 表示同一个旋转。

## `.calib` 没有记录什么

文件本身没有保存以下信息：

- 相机序列号；
- 最终选择的求解算法；
- 保存时间；
- 标定误差和独立验证结果；
- 相机后来是否被拆装或移动；
- 相机驱动内部 `camera_link -> optical_frame` 的 TF。

本次归档时从采集日志确认的补充信息：

| 侧别 | D405 序列号 | 算法 | 样本数 | 保存时间 |
|---|---|---|---:|---|
| 左腕 | `260522277324` | OpenCV/Park | 25 | 2026-08-09 14:13 |
| 右腕 | `260522279851` | OpenCV/Park | 25 | 2026-08-09 14:28 |

时区为 Asia/Shanghai。

## 样本内质量统计

这些指标通过固定标定板闭环
`base_T_link7 * link7_T_camera * camera_T_marker` 离线计算，仅表示已保存样本的内部一致性，不等同于独立验收。

| 侧别 | 闭环 RMS | 最大误差 | 4 折 Park RMS |
|---|---|---|---|
| 左腕 | `7.84 mm / 1.17 deg` | `21.63 mm / 2.31 deg` | `8.93 mm / 1.22 deg` |
| 右腕 | `14.15 mm / 1.96 deg` | `34.66 mm / 4.40 deg` | `14.66 mm / 2.05 deg` |

左腕样本内一致性优于右腕；右腕最差样本偏大。两侧在真实抓取前都必须做独立姿态和物理验证。

## 未来如何使用

### 1. 先确认结果仍适用

只有同时满足以下条件时才能继续使用：

- 左右相机序列号与上表一致；
- 相机仍安装在标定时对应的一侧；
- 相机、支架和机械臂末端自标定后没有拆装、碰撞或松动；
- 机器人使用的 `base_link -> openarmx_<side>_link7` 运动学定义与标定时兼容；
- frame 名称与 `.calib` 中一致。

任一条件不满足，都应重新标定。

### 2. 在 TF 系统中加载变换

抓取系统最终需要能够查询：

```text
base_link
  -> openarmx_<side>_link7
  -> <side>_camera_color_optical_frame
```

可以使用 Easy HandEye 2 加载对应名字的结果，也可以由目标系统从 `.calib` 读取 transform，再通过自己的
URDF、TF publisher 或配置机制发布。具体加载方式取决于目标系统，本归档不绑定某个工作空间。

如果使用 Easy HandEye 2 的默认加载方式，可将文件放到：

```text
~/.ros2/easy_handeye2/calibrations/left_link7_handeye.calib
~/.ros2/easy_handeye2/calibrations/right_link7_handeye.calib
```

对应的结果名称是：

```text
left_link7_handeye
right_link7_handeye
```

Easy HandEye publisher 会直接发布 `link7 -> color_optical_frame`。如果相机驱动或 URDF 已经为同一个 optical
child 发布了另一条父子关系，不能再重复发布；应先设计好唯一的 TF 树。

### 3. 坐标转换

相机光学坐标中的抓取点应通过 ROS `tf2` 转换，不建议在抓取代码里手工抄写矩阵：

```text
p_base = T(base_link -> openarmx_<side>_link7)
       * T(openarmx_<side>_link7 -> <side>_camera_color_optical_frame)
       * p_camera
```

同样，抓取姿态的旋转也必须按完整刚体变换处理，不能只给位置加平移。

### 4. `.samples` 的用途

`.samples` 保存每个采样姿态的：

- `robot`：机器人基座与末端之间的位姿；
- `tracking`：相机与标定标记之间的位姿。

抓取运行时不需要 `.samples`。应保留它们用于重新求解、交叉验证、排查坐标系方向或比较不同算法。

### 5. 抓取前验证

正式执行前至少应完成：

1. 检查 TF 链完整且每个 child 只有一个发布来源。
2. 在 RViz 检查点云方向、尺度和机器人模型是否一致。
3. 用位置已知的静止物体验证 `camera -> base_link` 转换。
4. 固定标定板，在多个未参与求解的机械臂姿态下检查其 `base_link` 位姿是否稳定。
5. 先只生成和显示抓取候选，再进行低速、可急停的单物体测试。

手眼标定正确不代表深度尺度、抓取网络坐标约定、TCP 定义和碰撞过滤已经正确，这些需要分别验证。

## 校验文件

```bash
cd /home/openflex/old/headeye
sha256sum -c SHA256SUMS
```

左右共四个原始文件都应显示 `OK`。
