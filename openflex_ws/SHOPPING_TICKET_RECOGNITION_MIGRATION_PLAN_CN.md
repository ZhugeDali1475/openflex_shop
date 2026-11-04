# 购物票识别与头部低头动作迁移计划

> 状态：代码迁移已完成，自动测试通过，待实机联调验收  
> 目标场景：商超比赛控制台第一阶段，在 `delivery_counter` 读取购物票并生成可执行订单  
> 示例来源：`/home/openflex/下载/order_receipt_recognizer.py`

## 1. 已确认结论

1. 头部相机使用 RealSense D435i，序列号为 `261922070132`。
2. 当前相机、固件和 librealsense 实机枚举支持彩色流 `1280x720@6fps`。
3. 购物票识别只需要彩色图像，不启用深度和 IMU。
4. 头部低头关节为 `openarmx_head_pitch_joint`，对应底部 RS00-2 电机、CAN ID `0x02`。
5. Pitch 负方向为低头，实际测试确认目标角度使用 `-70 deg`，即 `-1.221730476 rad`。
6. 头部命令话题为 `/head_forward_position_controller/commands`，消息顺序必须是 `[yaw_rad, pitch_rad]`。
7. 头部动作采用软件插值，不向电机一次性发送大角度跳变目标。初始参数为 `20 Hz`、`0.5 rad/s`。
8. 购物票识别必须复用当前抓取感知进程已经加载的 Qwen3.5-4B，不允许启动第二份模型。
9. SKU 白名单来自 `supermarket_scene/shelf_inventory.db`，数据库只读，不在识别过程中修改库存。
10. 删除控制台当前“等待最多 5 秒后使用手动订单”的逻辑。识别请求发出后等待真实结果、明确失败或用户取消。

## 2. 迁移前实现与缺口

当前比赛控制台已经具备以下流程：

```text
起点清障
  -> 导航到 delivery_counter
  -> 交付台精对齐
  -> 发布 /competition/shopping_ticket_request
  -> 等待 /competition/shopping_ticket_result
  -> 校验 SKU 和数量
  -> 生成逐件抓取队列
  -> 后退离开交付台
  -> 依次导航、抓取、交付
```

但目前存在四个实际缺口：

- `/competition/shopping_ticket_request` 只有发布端，没有真实识别节点。
- “已请求低头识别购物票”目前只是状态文字，没有下发头部电机动作。
- 控制台使用固定 5 秒截止时间，超时后可能使用手动订单，与本地模型工作方式不符。
- 示例脚本会独立加载 Qwen，直接迁移会占用第二份显存，并破坏现有模型进程所有权。

仓库中已经定义了正式接口 `openarmx_task_msgs/action/ReadOrder`，比赛协调器也在等待 `/read_delivery_order`。本次迁移应使用这个 Action 作为正式订单读取入口，而不是继续扩展临时 Empty/String 话题协议。

## 3. 目标架构

```text
比赛控制台 / competition_coordinator
             |
             |  ReadOrder Action: /read_delivery_order
             v
shopping_ticket_reader（新增，系统 ROS Python）
  |- 只读查询 shelf_inventory.db 的 sku_catalog
  |- 读取 /joint_states
  |- 插值控制头部 Pitch 到 -70 deg，并保持当前 Yaw
  |- 从 /vision/color/image_raw 冻结低头到位后的最新 RGB 帧
  |- 调用内部 /openflex_grasp/query_vlm Action
  |- 解析并校验 Qwen JSON，生成 OrderLine[]
  `- 恢复识别前 Pitch，返回 ReadOrder 结果
             |
             |  QueryVlm Action（新增内部接口）
             v
openflex_detect_object（现有常驻 perception venv）
  |- 继续持有唯一 Qwen3.5-4B + SAM2 实例
  |- Qwen 检测与购物票推理共用同一互斥锁，严格串行
  `- 返回原始生成文本，不访问数据库、不控制机器人
```

职责边界：

- `openflex_grasp` 只拥有模型和通用图文生成，不依赖商超数据库。
- `openarmx_task_executor` 拥有比赛业务、数据库白名单、头部动作、结果校验和错误码。
- GUI 只发起 Action、显示反馈、接收结构化订单，不直接导入模型或数据库实现。

## 4. 最终运行流程

### 4.1 启动与预检

1. 比赛整机 launch 启动头部 D435i：
   - serial：`261922070132`
   - color：`1280x720x6`
   - depth：`false`
   - IMU：`false`
   - 输出：`/vision/color/image_raw`
2. 启动现有 `openflex_detect_object`，只加载一次 Qwen3.5-4B 和 SAM2。
3. 启动新增 `shopping_ticket_reader`。
4. 控制台启动第一阶段前检查：
   - `/read_delivery_order` Action server ready；
   - `/openflex_grasp/query_vlm` Action server ready；
   - 头部控制器有订阅者；
   - `/joint_states` 中存在 yaw、pitch；
   - 头部 RGB 帧持续到达；
   - SQLite 文件存在且 `sku_catalog` 非空。

任何一项未就绪都禁止进入自动购物票流程，并显示具体缺失项。手动订单仅保留为用户主动选择的联调模式，不再作为自动超时降级。

### 4.2 到达交付台后的动作顺序

1. 底盘完成 `delivery_counter` 精对齐并停止。
2. 控制台发送 `ReadOrder` goal，不设置固定 5 秒识别截止时间。
3. `shopping_ticket_reader` 记录当前头部 yaw 和 pitch。
4. 保持 yaw 不变，以 `0.5 rad/s`、`20 Hz` 插值 Pitch 到 `-70 deg`。
5. Pitch 进入目标容差并稳定后，只接受时间戳晚于“到位时刻”的新 RGB 帧。
6. 冻结单帧 RGB 图像，不写临时图片文件。
7. 从数据库读取允许的 SKU，构造购物票提示词并调用共享 Qwen。
8. 图像已经冻结后即可开始把 Pitch 插值恢复到识别前角度；Qwen 可与恢复动作并行执行。
9. 等待 Qwen 返回，不使用固定 5 秒超时，也不切换到手动订单。
10. 解析 JSON，合并重复 SKU，过滤非法数量，并按数据库白名单进行精确校验。
11. 确认头部已经恢复后返回 `ReadOrder.Result.lines`。
12. 控制台将 `OrderLine[]` 转换为现有逐件队列，然后执行交付台退避和抓取流程。

### 4.3 等待、失败和取消语义

- 模型推理：等待真实 Action 结果，不设置 GUI 本地 5 秒 deadline。
- 用户取消：取消 `ReadOrder` 和下游 Qwen goal，停止继续识别，并恢复原 Pitch。
- Qwen 异常、JSON 无效、数据库为空、未知 SKU：明确返回失败，不使用旧订单或手动订单掩盖错误。
- 头部无法到位或恢复失败：作为硬件动作失败处理，不继续底盘退避和抓取。
- 相机没有低头到位后的新帧：报告相机不可用，不允许使用低头前的旧帧。
- Qwen worker 正在执行商品检测时：请求进入串行等待，不加载第二个模型，也不并发调用同一模型实例。
- 正式比赛总超时仍由上层比赛 Action 管理；删除的是购物票识别内部任意的 5 秒降级，不是取消整场比赛的安全机制。

## 5. 数据契约

### 5.1 数据库输入

扩展现有只读适配器 `ShelfInventoryDatabase`：

```python
def list_catalog_skus(self) -> list[str]:
    """Return sorted non-empty SKU names from sku_catalog in read-only mode."""
```

规则：

- 读取 `sku_catalog.sku`，排除空字符串和 `unknown`。
- 不直接导入 `supermarket_scene` Python 包。
- SQLite 始终以 `mode=ro` 打开。
- 数据库路径继续来自 `competition.yaml -> inventory.database_path`。

### 5.2 Qwen 输入

保留示例脚本的核心提示词和确定性生成参数：

- 输入：一张 PIL/RGB 等价图像和数据库 SKU 清单。
- `do_sample=false`。
- `max_new_tokens=256`。
- `enable_thinking=false`，避免思考文本污染 JSON。
- 要求只输出：

```json
{"items":[{"sku":"数据库中的完全一致 SKU","quantity":1}]}
```

### 5.3 Qwen 输出解析

从示例脚本迁移并测试以下纯函数：

- `normalize_items(items, allowed_skus)`
- `parse_receipt_response(response, allowed_skus)`

增加正式校验：

- 至少 1 行，最多由 `ReadOrder.expected_distinct_skus` 和比赛规则共同约束。
- SKU 必须与数据库完全一致。
- 数量必须是正整数，并遵守比赛上限。
- 重复 SKU 合并数量。
- 结果为空、行数不符或含未知 SKU 时失败。

`OrderLine` 初期字段映射：

| 字段 | 值 |
|---|---|
| `sku` | 数据库精确 SKU |
| `display_name` | 初期与 `sku` 相同 |
| `specification` | 数据库暂无独立字段时为空字符串 |
| `quantity` | 归一化后的正整数 |
| `confidence` | 结构、白名单和行数校验通过后的规则置信度；不伪装成模型概率 |

置信度的固定计算规则需要在实现时写入测试。初期建议：完整 JSON、全部精确命中白名单且行数符合 goal 时使用 `1.0`；否则直接失败，不返回低置信度的可执行订单。

### 5.4 内部 QueryVlm Action

在 `openflex_grasp_interfaces` 新增内部接口，建议字段如下：

```text
sensor_msgs/Image image
string prompt
uint32 max_new_tokens
---
bool success
string text
string model
uint16 error_code
string message
---
string phase
```

该接口只在本机 ROS 图内使用。Qwen worker 对请求串行处理，返回原始文本；业务 JSON 解析留在 `shopping_ticket_reader`。

## 6. 头部控制细节

目标参数：

| 参数 | 初始值 |
|---|---:|
| Pitch 目标 | `-70 deg` / `-1.221730476 rad` |
| 软件限位 | 使用 URDF `[-1.5708, 1.5708]`，目标再次 clamp |
| 插值频率 | `20 Hz` |
| 插值速度 | `0.5 rad/s` |
| 单步最大变化 | `0.025 rad` |
| 到位容差 | `1.5 deg` |
| 稳定时间 | `0.3 s` |

每条命令都发送完整数组：

```text
[当前 yaw, 插值后的 pitch]
```

只改变 Pitch；Yaw 读取自 `/joint_states` 并保持不变。不得假设 yaw 为 0，也不得交换数组顺序。

头部动作使用当前关节反馈闭环判断到位。实现中保留独立的运动安全截止时间，用来识别电机、控制器或反馈故障；这个截止时间与 Qwen 推理等待无关。

恢复策略：

- 成功、识别失败、Qwen 异常和用户取消都必须进入 `finally` 恢复路径。
- 默认恢复到发起 `ReadOrder` 前记录的 Pitch，而不是固定回到 0 deg。
- 恢复失败时不得开始底盘运动。

## 7. 文件修改清单

### 7.1 接口层

- `src/openflex_grasp/openflex_grasp_interfaces/action/QueryVlm.action`
- `src/openflex_grasp/openflex_grasp_interfaces/CMakeLists.txt`
- `src/openflex_grasp/openflex_grasp_interfaces/README_CN.md`

### 7.2 共享 Qwen worker

- `src/openflex_grasp/openflex_grasp/openflex_grasp/perception/providers/qwen_grounder.py`
  - 抽取通用 `generate_text(rgb, prompt, max_new_tokens)`。
  - 商品框检测继续复用该方法。
- `src/openflex_grasp/openflex_grasp/openflex_grasp/detect_object_server.py`
  - 增加 `/openflex_grasp/query_vlm` ActionServer。
  - 与 DetectObject 共用现有 `_busy` 锁和同一个 Qwen 实例。
- 对应 Qwen provider 和 Action server 单元测试。

### 7.3 购物票读取节点

- `src/openflex_integrated/openarmx_task_executor/openarmx_task_executor/shopping_ticket_reader_node.py`（新增）
- `src/openflex_integrated/openarmx_task_executor/openarmx_task_executor/receipt_recognition.py`（新增纯逻辑模块）
- `src/openflex_integrated/openarmx_task_executor/openarmx_task_executor/shelf_inventory_db.py`
- `src/openflex_integrated/openarmx_task_executor/setup.py`
- `src/openflex_integrated/openarmx_task_executor/package.xml`
- `src/openflex_integrated/openarmx_task_executor/config/competition.yaml`
- `src/openflex_integrated/openarmx_task_executor/launch/competition_task_api.launch.py`
- `src/openflex_integrated/openarmx_task_executor/launch/supermarket_full_system.launch.py`

新增节点同时提供：

- `/read_delivery_order`：正式 `ReadOrder` ActionServer。
- 不保留 `/competition/shopping_ticket_request` 和 `/competition/shopping_ticket_result` 兼容桥；GUI 已直接切换到正式 Action，避免维护两套入口。

### 7.4 比赛控制台

- `src/openflex_integrated/openflex_navigation_gui/openflex_navigation_gui/main_window.py`
  - 增加 `ReadOrder` ActionClient、feedback/result/cancel 状态。
  - 删除购物票 topic sequence 轮询状态。
- `src/openflex_integrated/openflex_navigation_gui/openflex_navigation_gui/competition_flow_tab.py`
  - 到达交付台后发送 `ReadOrder` goal。
  - 删除 `_ticket_wait_deadline = now + 5.0`。
  - 删除超时使用手动订单的分支。
  - 根据 Action feedback 显示低头、取帧、推理、恢复和校验状态。
  - Action 成功后直接用 `OrderLine[]` 构建现有订单队列。

### 7.5 头部相机

- `src/openflex_head/openarmx_head_visio_h264/launch/d435i_source.launch.py`
  - 保留通用默认值，比赛 launch 显式覆盖为 `1280x720x6`。
- `src/openflex_integrated/openarmx_task_executor/launch/supermarket_full_system.launch.py`
  - 增加可配置的头部 D435i include，避免要求人工另开终端。
  - 增加 `start_head_receipt_camera` 开关，防止与已启动的相机节点重复。
- `src/openflex_integrated/openarmx_task_executor/launch/supermarket_fullbody_runtime.launch.py`
  - 同步接入头部 D435i；这是当前比赛控制台“启动全身控制”实际使用的 launch。

## 8. 分阶段实施顺序

### 阶段 A：纯逻辑迁移

1. 迁移示例脚本的 prompt、normalize 和 JSON parser。
2. 增加数据库 SKU 目录只读 API。
3. 完成纯 Python 单元测试，不启动 ROS、相机或模型。

### 阶段 B：共享 Qwen 接口

1. 新增 `QueryVlm.action`。
2. 将 Qwen 通用生成从商品框 prompt 中抽出。
3. 在现有 worker 上实现 QueryVlm Action，并确认 Detect/Query 串行。
4. 验证 GPU 中仍只有一份 Qwen3.5-4B 权重。

### 阶段 C：购物票节点与头部动作

1. 实现头部插值控制和关节反馈判断。
2. 实现到位后新帧冻结。
3. 实现 Qwen 调用、JSON 解析、数据库校验和 Pitch 恢复。
4. 实现 `ReadOrder` ActionServer 和取消路径。

### 阶段 D：控制台接线

1. GUI 从临时 topic 协议迁移到 `ReadOrder` Action。
2. 删除固定 5 秒等待与自动手动订单降级。
3. 接入 feedback、失败原因和取消。
4. 保持现有 `_build_order_queue()`、库存数量检查和逐件抓取链不变。

### 阶段 E：整机 launch 与硬件验收

1. [x] 比赛 launch 启动 D435i `1280x720@6fps`。
2. [ ] 实机验证低头、拍照、恢复和订单执行全链路。
3. [x] GUI 直接删除兼容 topic，不维护双入口。

## 9. 测试与验收标准

### 9.1 自动测试

- JSON 外包 Markdown、前后解释文本仍能提取首个有效 JSON。
- 数字字符串数量可转换；布尔、零、负数和小数被拒绝。
- 重复 SKU 正确合并。
- 未在数据库中的 SKU 被拒绝。
- 数据库以只读模式打开，识别流程不能创建或修改表。
- 头部插值不越过目标、不超出 URDF 限位，并保持 yaw 不变。
- 只有低头到位后的新帧可以进入 Qwen 请求。
- Qwen 请求等待超过 5 秒时 GUI 仍保持识别中，不触发手动订单。
- 取消、模型异常、相机异常均执行 Pitch 恢复。
- DetectObject 与 QueryVlm 同时到达时严格串行，不出现模型并发调用。
- ReadOrder 成功结果可以直接生成现有比赛订单队列。

### 9.2 实机验收

1. 相机 profile 显示 `1280x720@6`，图像实际尺寸正确。
2. 从任意安全初始 Pitch 发起识别，头部平滑到达 `-70 deg`。
3. Yaw 在整个动作中不发生可见改变。
4. 低头到位后采集的画面完整包含购物票，文字可读。
5. Qwen 推理即使超过 5 秒，控制台也继续等待真实结果。
6. 识别完成后头部平滑恢复到原 Pitch。
7. 控制台显示的 SKU 和数量与购物票一致，并来自当前数据库白名单。
8. 订单成功进入现有货架定位、抓取、交付队列。
9. `nvidia-smi` 和进程检查确认没有第二份 Qwen 模型。
10. 在推理中点击取消，机器人不开始后退或抓取，头部能够恢复。

## 10. 本次不做的内容

- 不修改购物票照片本身，不将图片持久化到磁盘。
- 不训练或微调 Qwen。
- 不在识别成功时修改库存数据库；库存扣减仍由抓取/交付业务决定。
- 不改变现有货架锚点、腕部相机抓取、SAM2 或 GraspNet 流程。
- 不保留“自动识别失败后静默使用旧手动订单”的行为。

## 11. 评审确认项

执行前需要确认以下方案值：

- [x] Pitch 目标固定为 `-70 deg`。
- [x] 头部插值初值采用 `0.5 rad/s @ 20 Hz`。
- [x] 识别后恢复到动作前 Pitch。
- [x] 比赛相机固定采用 `1280x720@6fps`、color only。
- [x] 使用现有 `ReadOrder` Action 作为正式入口。
- [x] 使用新增内部 QueryVlm Action 复用唯一 Qwen worker。
- [x] 删除 GUI 固定 5 秒等待和自动手动订单降级。
- [x] SKU 白名单来自只读 `shelf_inventory.db`。

## 12. 当前实现结果

已完成：

- 购物票 prompt、JSON 解析、SKU 归一化、订单校验与只读数据库接口。
- 共享 `QueryVlm` Action，商品检测与购物票识别复用同一常驻 Qwen。
- `shopping_ticket_reader` 的头部 Pitch 插值、新帧冻结、Qwen 调用、恢复和取消路径。
- GUI `ReadOrder` Action 状态机、反馈显示、结构化订单接入及旧 5 秒降级删除。
- `competition_task_api`、整机 launch 和控制台实际使用的 fullbody runtime 接线。
- 相关 ROS 包构建、launch 参数展开和自动测试。

仍需实机完成：

- 检查 `/vision/color/image_raw` 实际为 `1280x720@6fps`。
- 检查头部从当前姿态平滑到 `-70 deg` 并恢复原 Pitch，Yaw 保持不变。
- 使用真实购物票验证 Qwen 输出、数据库 SKU 命中和逐件订单队列。
- 推理期间执行取消，确认下游 Qwen 被取消、头部恢复且底盘不退避。
- 用 `nvidia-smi` 确认运行时只有一份 Qwen3.5-4B 权重。
