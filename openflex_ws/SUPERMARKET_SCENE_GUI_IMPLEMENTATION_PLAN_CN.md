# Supermarket Scene 与比赛 GUI 第一版实施计划

## 1. 文档定位

本文将 `SUPERMARKET_SCENE_GUI_MIGRATION_DESIGN_CN.md` 中的讨论结论转为可以直接执行的代码修改计划。

第一版目标是形成完整可用闭环：

```text
启动比赛 GUI
  -> 自动连接场景数据节点和视觉节点
  -> 从 ROS 读取数据库场景
  -> 在原生 2D 地图上人工移动、旋转货架和台子
  -> 点击确认后原子写入数据库
  -> React + Three.js 立即显示同一场景
  -> React 保留全部 CRUD 和视觉页面
  -> 所有正式调用通过 Qt Bridge + ROS 2
```

本文同时作为实施计划和进度记录。未在“当前实施状态”中标记完成的内容，仍按后文章节继续执行。

### 1.1 当前实施状态（2026-08-11）

已经完成并通过构建或测试：

1. SQLite additive migration、WAL、修订号、地图绑定和原子布局更新；
2. `supermarket_scene_core`、`supermarket_scene_interfaces`、`supermarket_scene_ros` 和 `supermarket_scene_web` 包基础；
3. data node 的场景快照、布局提交以及货架、台子、类型、SKU、货位 CRUD；
4. Qt 2D 人工配准页，包括 PGM/YAML、真实尺寸、局部原点和坐标轴、拖动、旋转、数值编辑与撤销重做；
5. React + Three.js 场景管理入口、QtWebChannel transport 和 HTTP 开发兼容 transport；
6. Qt `QFileDialog` 与内存 `MediaStore`，巡检和 SKU 查询原图不经过 QWebChannel Base64；
7. 独立只读 vision node，巡检和 SKU 查询的当前页面参数按单次 ROS Service 请求传入；
8. 比赛 GUI 启动脚本自动托管 data node，并可通过 `--start-scene-vision` / `--no-scene-vision` 控制后台 vision node；
9. 默认 `supermarket_test_05` 地图路径已经适配当前 `/home/openflex/old` 工作区，启动预检通过。

当前剩余的第一版工作：

1. 将人工批量录入、图片拼接、透视校正、grounding 和 SKU prompt generation 完整迁移到 ROS；
2. 为嵌入模式提供商品图片、货架图片和视觉产物资源通道，消除 `file://` 下的 `/api/...` 图片依赖；
3. 将 calibration/vision 传输无关代码完整迁入 core，并参数化 `data_root`；
4. 安装 QtWebEngine 后完成真实嵌入截图与交互验收；
5. 完成第一版端到端验收后再退役旧兼容节点和运行时项目根查找。

## 2. 第一版范围

### 2.1 必须完成

1. 将 `supermarket_scene` 核心逻辑整理成可由 colcon 安装的 Python ROS 包。
2. 将接口包和 ROS 节点包纳入 `openflex_ws/src` 的正常构建。
3. 拆分 `supermarket_scene_data_node` 与 `supermarket_scene_vision_node`。
4. 新增完整场景快照、地图绑定、批量布局和 Web CRUD 所需接口。
5. 在比赛 GUI 中增加全尺寸“场景配准”和“场景管理”工作区。
6. 场景配准页从数据库读取实体尺寸，在 PGM 上人工拖动和旋转。
7. 货架框显示数据库原点、`+X/+Y`、face 标签和每面代表商品。
8. React + Three.js 通过 QtWebEngine 本地嵌入。
9. React 保留货架、台子、货架类型、SKU、固定货位的全部 CRUD。
10. React 保留 `world_x/world_y/yaw` 数值编辑；Three.js 画布不直接拖动物体。
11. Three.js 保留当前货架模型、长方体商品、单面图片、缺货线框和错放红框。
12. 本地图片统一由 Qt `QFileDialog` 选择，不经过 JavaScript Base64。
13. 当前巡检与商品查询复用已有异步 ROS Service；其余视觉能力补齐 ROS Service。
14. data node 是 SQLite 唯一写入者；vision node 只读并返回候选。
15. 保留 HTTP 适配器用于迁移期独立开发，但比赛运行时不依赖 HTTP 服务。

### 2.2 第一版明确不做

1. 不做 PGM 障碍轮廓自动识别或自动对齐。
2. 不做实体碰撞、机器人 footprint、自由空间连通性和导航可达性检查。
3. 不做配置模式、比赛模式、权限锁定和操作员角色。
4. 不做 Three.js 中的货架拖动。
5. 不加载 OBJ、PLY 或复杂商品网格。
6. 不把 PGM 作为 3D 地面纹理，不生成墙体或障碍网格。
7. 不实现完整的多地图 `draft/published/archived` 历史版本。
8. 不强制将所有耗时视觉 Service 在第一版改为 Action。
9. 不删除现有 HTTP API，直到 Qt/ROS 路径通过验收。

## 3. 实施前置条件

### 3.1 系统依赖

当前环境缺少 `PyQt5.QtWebEngineWidgets`。开始 GUI 集成前安装并验证：

```bash
sudo apt install python3-pyqt5.qtwebengine
python3 -c "from PyQt5.QtWebEngineWidgets import QWebEngineView"
```

前端构建继续需要 Node.js 和 npm。`supermarket_scene_web` 的 colcon 构建必须明确检查 `npm`，缺失时给出可操作错误。

### 3.2 数据备份

任何数据库 schema 迁移前：

1. 关闭现有 HTTP/ROS 写入进程；
2. 使用 SQLite backup API 创建带时间戳备份；
3. 在备份副本上执行迁移测试；
4. 迁移成功后才替换运行数据库；
5. 不使用会删除现有库存的 `init_database.py`。

### 3.3 构建基线

修改前保存以下基线结果：

```bash
cd /home/openflex/old/supermarket_scene
uv run python -m unittest tests.test_fixed_slots
cd web && npm run build

cd /home/openflex/old/openflex_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select openflex_navigation_gui
```

## 4. 目标包结构

第一版完成后，工作区内采用：

```text
openflex_ws/src/openflex_integrated/
├── supermarket_scene_core/
├── supermarket_scene_interfaces/
├── supermarket_scene_ros/
├── supermarket_scene_web/
└── openflex_navigation_gui/
```

### 4.1 `supermarket_scene_core`

职责：

- `ShelfDatabase` 和 schema migration；
- 场景快照生成；
- 货架和货位几何计算；
- 校准 JSON 同步；
- 当前视觉算法模块；
- 不包含 ROS callback、Qt 或 React。

从现有仓库迁移：

```text
shelf_database.py
scene_geometry.py
calibration_manager.py
robot_service.py 中的传输无关逻辑
vision/
```

迁移期间在旧模块位置保留薄兼容导入，避免 HTTP API 和现有测试一次性全部失效。兼容层不得继续复制业务逻辑。

### 4.2 `supermarket_scene_interfaces`

从 `supermarket_scene/ros2/supermarket_scene_interfaces` 移入工作区，并扩展完整 GUI 接口。

### 4.3 `supermarket_scene_ros`

从现有单节点包迁移并拆成：

```text
supermarket_scene_ros/
├── data_node.py
├── vision_node.py
├── message_converters.py
├── launch/supermarket_scene_system.launch.py
└── setup.py
```

删除当前通过 `SUPERMARKET_SCENE_ROOT` 修改 `sys.path` 的运行时查找逻辑，改为正常依赖 `supermarket_scene_core`。

### 4.4 `supermarket_scene_web`

包含当前 `web/src`、`package.json`、lockfile和构建产物安装规则。正式运行从包 share 目录加载静态文件，不启动 Vite。

开发模式允许继续使用 Vite + HTTP transport adapter。

### 4.5 运行数据位置

Python源码迁入 ROS 包不等于把可写数据库和用户图片安装进 ROS share 目录。第一版保留现有运行数据位置，并全部通过参数显式传入：

```text
db_path
data_root
vision_config_path
```

当前默认值由启动脚本解析为：

```text
/home/openflex/old/supermarket_scene/shelf_inventory.db
/home/openflex/old/supermarket_scene/data
/home/openflex/old/supermarket_scene/vision/config.local.yaml
```

代码和 launch 文件不得硬编码该绝对路径。启动脚本新增 `SUPERMARKET_SCENE_DB_PATH`、`SUPERMARKET_SCENE_DATA_ROOT` 和对应命令行参数覆盖。`image_dir` 的相对路径统一相对 `data_root` 解析。

数据库、商品图片、视觉输出和本地配置不得写入 `install/*/share`；colcon重新构建不能覆盖运行数据。

## 5. 阶段一：核心包和数据库基础

### 5.1 文件修改

新增或迁移：

```text
supermarket_scene_core/package.xml
supermarket_scene_core/setup.py
supermarket_scene_core/setup.cfg
supermarket_scene_core/resource/supermarket_scene_core
supermarket_scene_core/supermarket_scene_core/shelf_database.py
supermarket_scene_core/supermarket_scene_core/scene_service.py
supermarket_scene_core/supermarket_scene_core/scene_geometry.py
supermarket_scene_core/supermarket_scene_core/calibration_manager.py
supermarket_scene_core/supermarket_scene_core/vision/*
supermarket_scene_core/test/*
```

`scene_service.py` 作为 GUI/ROS 使用的传输无关业务门面，至少提供：

```text
get_scene_snapshot(db_path, map_name, map_fingerprint)
apply_scene_layout(db_path, map_binding, shelf_poses, table_poses)
apply_inspection_results(db_path, expected_revision, changes)
```

### 5.2 数据库第一版迁移

保留现有：

```text
shelf_groups.world_x/world_y/yaw
delivery_tables.world_x/world_y/yaw
```

新增：

```sql
CREATE TABLE scene_metadata (
    id INTEGER PRIMARY KEY CHECK(id = 1),
    schema_version INTEGER NOT NULL,
    scene_revision INTEGER NOT NULL
);

CREATE TABLE scene_map_binding (
    id INTEGER PRIMARY KEY CHECK(id = 1),
    map_name TEXT NOT NULL,
    map_fingerprint TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
```

为了让两个台子的俯视尺寸真正来自数据库，第一版对 `delivery_tables` 增加：

```text
length
width
height
top_thickness
```

迁移默认值使用当前 `scene_geometry.py` 的 `1.20/0.80/0.75/0.03m`。长期再决定是否抽象为 `fixture_types`。

### 5.3 事务与修订号

现有数据库方法大量在方法内部直接 `commit()`。必须先增加事务边界，确保：

- `apply_scene_layout` 在一个事务中更新两个货架、两个台子、地图绑定和修订号；
- 任一实体不存在或数值非法时全部回滚；
- 所有成功写操作将 `scene_revision` 增加一次；
- 批量操作不能因内部方法提前提交而破坏原子性。

建议增加内部无提交方法或统一事务上下文，不在 Service callback 中拼接裸 SQL。

### 5.4 SQLite 并发设置

data node 打开数据库时设置：

```sql
PRAGMA journal_mode=WAL;
PRAGMA busy_timeout=5000;
PRAGMA foreign_keys=ON;
```

vision node 使用只读 URI和：

```sql
PRAGMA query_only=ON;
PRAGMA busy_timeout=5000;
```

### 5.5 阶段验收

- 旧数据库无损迁移；
- 货架、台子、SKU和 slot 数量迁移前后一致；
- 台子尺寸可从数据库读取；
- 批量布局中途失败不会留下部分坐标；
- `scene_revision` 每个成功事务只增加一次；
- 现有固定货位测试继续通过。

## 6. 阶段二：ROS 2 接口补齐

### 6.1 新增消息

在 `supermarket_scene_interfaces/msg` 增加：

```text
ShelfType.msg
ShelfScene.msg
DeliveryTableScene.msg
SkuInfo.msg
SceneSlot.msg
SceneMapBinding.msg
ShelfPoseUpdate.msg
DeliveryTablePoseUpdate.msg
VisionConfig.msg
SkuQueryConfig.msg
StitchPair.msg
StitchSkip.msg
ManualImportSlot.msg
```

关键要求：

- `ShelfScene` 直接携带解析后的全部物理尺寸，同时保留 `shelf_type_id`；
- `DeliveryTableScene` 直接携带数据库尺寸；
- `SceneSlot` 同时返回 `local_x/y/z` 与 `world_x/y/z/yaw`；
- `SceneSlot` 返回 `expected_sku`、可空 `actual_sku`、状态和缩略图修订信息；
- 所有 ID 类型统一，不在 int32/int64 之间随意变化。

### 6.2 场景与布局服务

新增：

```text
GetSceneSnapshot.srv
ApplySceneLayout.srv
GetSceneRevision.srv
```

`GetSceneSnapshot` 一次返回：

```text
scene_revision
map_binding
shelf_types
shelves
delivery_tables
skus
slots
```

`ApplySceneLayout` 请求携带：

```text
map_name
map_fingerprint
expected_scene_revision
shelf_updates[]
delivery_table_updates[]
```

### 6.3 完整 CRUD 服务

保留并扩展现有 Create/Update服务，补齐：

```text
DeleteShelf.srv
DeleteDeliveryTable.srv
DeleteSlot.srv
CreateShelfType.srv
UpdateShelfType.srv
DeleteShelfType.srv
CreateSku.srv
UpdateSku.srv
DeleteSku.srv
ClearShelfInventory.srv
ImportSlotsBatch.srv
ApplyInspectionResults.srv
```

所有写响应至少返回：

```text
success
error
scene_revision
```

不在每个响应中复制完整场景；客户端写成功后重新获取快照。

### 6.4 视觉服务

第一版保留：

```text
InspectShelf.srv
FindSku.srv
```

新增：

```text
GetVisionStatus.srv
GetVisionConfig.srv
UpdateVisionConfig.srv
GetSkuQueryConfig.srv
UpdateSkuQueryConfig.srv
StitchImages.srv
RectifyImage.srv
GroundProducts.srv
GenerateSkuPrompts.srv
```

图片字段使用 `sensor_msgs/CompressedImage`，多图第一版使用数组。所有视觉响应携带产生结果时的 `observed_scene_revision`。

### 6.5 构建文件

更新：

```text
supermarket_scene_interfaces/CMakeLists.txt
supermarket_scene_interfaces/package.xml
supermarket_scene_ros/package.xml
supermarket_scene_ros/setup.py
```

接口包增加 `geometry_msgs`、`sensor_msgs` 等实际依赖。构建顺序固定为 core、interfaces、ros。

### 6.6 阶段验收

- `colcon build --packages-select supermarket_scene_core supermarket_scene_interfaces supermarket_scene_ros` 通过；
- 所有新 `.msg/.srv` 可导入；
- 快照与 HTTP `/api/state` 的核心数据一致；
- CRUD服务覆盖当前 React 的每条 `/api` 写路径；
- `ApplySceneLayout` 可原子更新四个实体；
- 视觉服务保持结构化 bbox 结果。

## 7. 阶段三：拆分 data node 与 vision node

### 7.1 `data_node.py`

提供：

- `GetSceneSnapshot`；
- 所有场景和库存 CRUD；
- `ApplySceneLayout`；
- `ApplyInspectionResults`；
- 数据库 schema migration和唯一写连接。

data node 不导入 torch、transformers、DINO或OWLv2。

### 7.2 `vision_node.py`

提供：

- `InspectShelf`；
- `FindSku`；
- 拼接、透视校正、grounding、提示生成；
- 模型预加载和 `GetVisionStatus`。

第一版使用单个重任务互斥锁：状态为 BUSY 时拒绝新的重任务，不做队列。vision node 不调用任何数据库写方法。

### 7.3 launch

新增 `supermarket_scene_system.launch.py` 参数：

```text
db_path
data_root
vision_config_path
start_vision
provider
model
```

同一 launch 启动 data和vision节点。vision节点失败不能导致 data节点退出。

### 7.4 阶段验收

- vision模型加载期间 data快照和 CRUD仍可调用；
- 杀死 vision node 后 data node仍工作；
- vision node无法写 SQLite；
- 旧 revision 的巡检结果被 data node拒绝应用；
- 统一 launch 可单独关闭视觉节点用于轻量测试。

## 8. 阶段四：Qt 场景客户端与 SceneStore

### 8.1 新增文件

在 `openflex_navigation_gui/openflex_navigation_gui` 增加：

```text
scene_models.py
scene_ros_client.py
scene_store.py
map_fingerprint.py
```

职责：

- `scene_models.py`：ROS消息到纯 Python dataclass/JSON DTO转换；
- `scene_ros_client.py`：创建所有场景 Service client，提交异步请求；
- `scene_store.py`：保存 confirmed snapshot、当前编辑草稿、revision和选中实体；
- `map_fingerprint.py`：对解码后的 PGM栅格和规范化 YAML参数计算 SHA-256。

### 8.2 ROS执行模型

复用当前 Qt定时器中的 `rclpy.spin_once()`，所有调用使用异步 future。禁止：

- 在 GUI线程 `spin_until_future_complete()`；
- 在 Qt slot 中同步等待 Service；
- 从 WebEngine callback 直接访问 ROS生成消息。

`SceneRosClient` 完成请求后只更新 SceneStore或发出 Qt signal。

### 8.3 去除 GUI 直接 SQLite

修改：

```text
competition_flow_tab.py
```

删除：

```text
sqlite3.connect(...)
_query_available_skus()
_query_sku_slots()
GUI 内重复的完整货位几何算法
```

商品和货位选择改读 SceneStore。正式世界坐标使用快照结果，草稿预览只应用后端返回的 slot局部坐标刚体变换。

### 8.4 阶段验收

- GUI源码中不再出现数据库文件查找和 `sqlite3.connect`；
- data node离线时 GUI给出明确状态但导航功能仍可使用；
- 快照刷新不阻塞现有 20/50/200/1000ms 定时器；
- revision变化能触发统一刷新。

## 9. 阶段五：原生 Qt 2D 场景配准页

### 9.1 新增文件

```text
scene_registration.py
scene_registration_canvas.py
```

尽量复用现有：

```text
map_editor_core.parse_map_yaml
map_editor_core.read_pgm
PathPlot 的 world/pixel 变换经验
```

将通用世界坐标/像素坐标变换提取成可测试 helper，避免 PathPlot和配准画布各写一套。

### 9.2 画布能力

只实现人工操作：

- PGM只读显示；
- 货架和台子按数据库物理尺寸等比例绘制；
- 点击选择；
- 拖动实体中心移动；
- 旋转柄绕中心旋转；
- `X/Y/Yaw` 数值输入；
- 原点、`+X/+Y` 箭头；
- `face=0/face=1`；
- 每面3个代表商品；
- 整体布局统一移动/旋转；
- 撤销/重做；
- 取消草稿、确认应用。

不实现自动吸附和自动安全评分。

### 9.3 初始草稿

加载新地图时：

1. 读取当前数据库布局；
2. 计算所有实体物理角点的整体包围框中心；
3. 保持相对位置和yaw；
4. 整体平移到目标地图中心附近；
5. 不缩放、不自动旋转；
6. 用户人工确认后才调用 `ApplySceneLayout`。

### 9.4 代表商品规则

按 `(shelf_id, face)` 分组和 `y_cm` 排序，从起点、中间和末端选择，尽量覆盖不同层并去重 SKU。显示：

```text
正常：actual_sku
缺货：应为 expected_sku
错放：actual_sku（应 expected_sku）
```

### 9.5 GUI 工作区集成

当前比赛 GUI 是“左侧控制标签 + 右侧地图”的单个 central widget，不适合嵌入完整 Web工作台。

在 `CompetitionMainWindow` 初始化后，将现有 central widget包装为顶层工作区：

```text
机器人控制    现有完整界面
场景配准      原生 Qt 全尺寸页面
场景管理      QtWebEngine 全尺寸页面
```

不要把场景配准或 Web页面塞进当前最大宽度510px的 `control_tabs`。

### 9.6 阶段验收

- 1280x800和1540x940下控件不重叠；
- 货架和台子尺寸与数据库一致；
- 旋转后数据库角点原点计算正确；
- 整体旋转保持实体相对变换；
- 确认后四个实体一次提交并刷新3D；
- 取消不会写数据库。

## 10. 阶段六：Web包、QtWebEngine和桥接层

### 10.1 Web静态包

新增 `supermarket_scene_web` ROS包，安装构建后的静态资源到：

```text
share/supermarket_scene_web/web/
```

正式入口使用自定义 scheme：

```text
supermarket://app/index.html
```

开发模式继续允许 `npm run dev`。

### 10.2 Qt新增文件

```text
web_scene_view.py
web_bridge.py
media_store.py
web_scheme_handler.py
```

职责：

- 创建单个 `QWebEngineView`；
- 注册 `QWebChannel`；
- 暴露 `sceneBridge.invoke(request_id, operation, payload_json)`；
- 暴露 `mediaBridge.chooseImages(...)`；
- 管理请求关联、超时和错误；
- 提供 `supermarket://` 和 `supermarket-resource://`。

### 10.3 MediaStore

Qt文件选择器返回临时资产：

```text
asset_id
original_name
mime_type
width/height
compressed_size
content_hash
preview_uri
```

要求：

- React不获得绝对路径；
- 原图字节不经过 QWebChannel JSON；
- Qt保留原始压缩数据供 `CompressedImage`；
- Qt生成单独缩略图用于预览；
- 资源支持引用计数、TTL和退出清理；
- 正在用于拼接或人工录入的资源不能提前释放。

### 10.4 Qt依赖

更新 `openflex_navigation_gui/package.xml`：

```text
exec_depend: python3-pyqt5.qtwebengine
depend: supermarket_scene_interfaces
exec_depend: supermarket_scene_ros
exec_depend: supermarket_scene_web
```

测试环境使用延迟创建 WebEngine或可注入的 stub，避免所有纯逻辑测试启动 Chromium。

### 10.5 阶段验收

- 正式模式不启动 Vite和 HTTP API；
- Web页面本地加载无白屏；
- QWebChannel请求可并发关联并正确超时；
- 大图不以 Base64出现在桥接 payload；
- 退出 GUI 后临时媒体被清理；
- WebEngine崩溃不会破坏导航 ROS节点。

## 11. 阶段七：React transport迁移

### 11.1 新增前端模块

```text
web/src/bridge/qtChannel.ts
web/src/api/sceneApi.ts
web/src/api/visionApi.ts
web/src/api/mediaApi.ts
web/src/store/sceneStore.ts
```

保留两种 transport：

```text
QtRosTransport     正式比赛 GUI
HttpTransport      独立开发和迁移期回归
```

业务组件不得直接调用 `fetch('/api/...')`，只能调用类型化 API模块。

### 11.2 迁移顺序

按风险从低到高：

1. `/api/state` -> `GetSceneSnapshot`；
2. 货架、台子、类型和 SKU CRUD；
3. 固定货位 CRUD、take、restock和清理；
4. 商品图片读取；
5. 巡检和商品查询；
6. AI辅助框选；
7. 人工批量录入；
8. 图片拼接和透视校正；
9. SKU提示生成。

每完成一组，Qt transport和HTTP transport必须返回相同前端 DTO。

### 11.3 App状态和页面切换

将当前多个布尔页面状态收敛为：

```text
scene
manual-import
inspection
sku-query
image-stitch
```

只使用一个 WebEngine和一个 React根实例。图片拼接到人工录入传递 `asset_id`，不重新下载或转Base64。

### 11.4 保留完整功能

React继续支持：

- 新增、修改、删除货架；
- 新增、修改、删除两个台子；
- 货架类型 CRUD；
- SKU CRUD；
- 固定货位 CRUD和批量清理；
- `world_x/world_y/yaw` 数值编辑；
- 所有视觉页面。

React空间数值保存后更新SceneStore并刷新2D。2D确认后刷新React和Three.js。

### 11.5 商品渲染

保留当前 `WarehouseScene.tsx` 的商品逻辑：

- 0.03m厚长方体；
- `width_cm/height_cm`；
- face对应的朝外单面图片；
- 其他面SKU颜色；
- 缺货线框；
- 错放红边。

移除渲染对 `mesh_file/tex_file` 的依赖，但字段可为兼容继续存在。

3D地面使用纯色平面、网格和WORLD轴。地面范围从地图元数据获得，相机按四个实体包围框自动取景。

### 11.6 人工录入改造

当前页面会生成 `dataUrl` 和 `crop_png`。迁移后：

- 原图由 Qt MediaStore持有；
- React只提交原图 `asset_id`、层标定点、商品框和SKU字段；
- data/vision服务根据原图和bbox生成裁剪图并保存；
- 不通过 QWebChannel传商品裁剪Base64。

### 11.7 阶段验收

- 生产构建中没有业务组件直接请求 `/api`；
- React全部 CRUD经 ROS工作；
- 2D、表单和3D数据一致；
- 浏览器开发控制台无未处理Promise和资源404；
- 人工录入、巡检、查询、拼接形成完整闭环。

## 12. 阶段八：启动、生命周期和兼容清理

### 12.1 启动方式

更新：

```text
start_supermarket_fullbody_competition_gui.sh
```

预检增加：

- `supermarket_scene_core/interfaces/ros/web` 已安装；
- QtWebEngine可导入；
- 数据库可读写且备份目录可用；
- Web静态入口存在；
- map.yaml和PGM可计算指纹。

场景系统应有明确进程所有者。第一版建议由比赛 GUI 使用现有 QProcess管理模式启动 `supermarket_scene_system.launch.py`：

- GUI启动时若服务不存在则启动；
- 若发现外部data node则只连接，不重复启动；
- GUI只停止自己启动的场景进程；
- vision node可单独重启；
- 不与后续整机runtime重复启动。

### 12.2 HTTP兼容层

迁移完成前：

- `api_server.py` 改为调用 `supermarket_scene_core`，不保留复制逻辑；
- Vite开发继续使用HTTP adapter；
- 比赛脚本不启动HTTP API；
- ROS/Qt验收完成后再决定是否删除HTTP写入口。

迁移期必须遵守唯一写入者：

- 独立 Web 开发模式启动 `api_server.py` 时，不同时启动指向同一数据库的 data node；
- data node运行时，HTTP API只能只读，或将写请求转发给data node；
- 禁止 HTTP API和data node各自直接写同一个 SQLite 文件。

### 12.3 任务执行器

修改 `openarmx_task_executor`：

- 停止直接读取 `shelf_inventory.db`；
- 通过 `GetSceneSnapshot/GetSkuLocations` 获取库存和坐标；
- 删除或停用GUI内写死的 `SHELF_FACE_TO_STATIONS`；
- 第一版导航站点自动生成可后置，但至少不再从数据库路径直接查询。

如果该改动影响正式比赛链，可作为第一版GUI验收后的独立提交，不阻塞2D/3D闭环上线。

## 13. 测试计划

### 13.1 Core和数据库

- 旧schema迁移；
- 迁移幂等；
- revision递增；
- 批量布局成功和回滚；
- 台子尺寸；
- slot局部/世界坐标；
- WAL下只读视觉连接；
- stale revision拒绝应用。

### 13.2 ROS接口和节点

- 所有msg/srv生成和导入；
- data node不导入视觉模型；
- vision loading时data可用；
- CRUD和snapshot一致；
- vision BUSY行为；
- launch参数和db_path；
- 节点退出和重启。

### 13.3 Qt纯逻辑

- PGM/world双向变换，包含非零origin yaw；
- 地图指纹规范化；
- 角点原点和中心旋转换算；
- 整体布局平移/旋转；
- undo/redo；
- 代表商品选择；
- SceneStore revision和草稿取消；
- Bridge request ID、超时和错误；
- MediaStore清理。

### 13.4 React

- Qt和HTTP transport DTO一致；
- CRUD页面不直接fetch；
- route切换保留SceneStore；
- 图片选择只使用asset_id；
- 视觉结果勾选后才应用；
- 拼接结果进入人工录入。

### 13.5 端到端

至少覆盖：

1. 启动data/vision/GUI；
2. 加载test_08地图；
3. 获取两个货架和两个台子；
4. 2D拖动和旋转后确认；
5. 查询数据库确认原子写入；
6. Three.js立即显示新位置；
7. React数值修改位置后2D刷新；
8. 新增/编辑/删除SKU和slot；
9. Qt选择图片并执行巡检；
10. 视觉节点退出后场景CRUD仍工作。

使用 Playwright或等效自动化检查 Web页面在嵌入式尺寸下：

- Three.js canvas非空；
- 货架、台子和商品实际渲染；
- 1280x800与1540x940无重叠和文本溢出；
- 页面切换和图片资源无404；
- 3D相机正确框住实体。

## 14. 提交顺序

为了保持每一步可构建、可回退，建议按独立提交实施：

1. `supermarket_scene_core` 包装和兼容导入；
2. 数据库增量迁移、revision、台子尺寸和事务；
3. 场景快照与布局消息/服务；
4. CRUD和视觉接口补齐；
5. data/vision双节点和launch；
6. Qt SceneRosClient和SceneStore，移除GUI直读SQLite；
7. Qt 2D配准页；
8. `supermarket_scene_web` 静态ROS包；
9. QtWebEngine、QWebChannel和MediaStore；
10. React场景/CRUD transport迁移；
11. React视觉和图片流程迁移；
12. 启动脚本、生命周期和完整文档；
13. 任务执行器去除SQLite和写死映射；
14. 端到端测试和HTTP兼容入口退役评估。

每个提交必须包含对应测试，不能在最后一次性补测试。

## 15. 第一版完成标准

只有同时满足以下条件才算完成：

- 比赛 GUI中可直接打开场景配准和场景管理；
- 两个货架和两个台子的尺寸来自数据库；
- 2D可人工移动、旋转并显示原点、箭头、face和代表商品；
- 点击确认后通过ROS事务更新数据库；
- React数值编辑和2D编辑双向同步；
- Three.js使用简洁地板并正确显示货架、台子、商品和单面图片；
- React全部CRUD不依赖比赛运行时HTTP API；
- 图片由Qt选择且大图不经过QWebChannel/Base64；
- 巡检、查询、拼接和人工录入可以从嵌入页面完成；
- data/vision节点分离，视觉故障不影响数据库和2D配准；
- GUI和任务侧不再新增直接SQLite访问；
- core、ROS、Qt、React和端到端测试全部通过；
- 当前数据库有可恢复备份，迁移过程可重复执行。

## 16. 第一版之后

第一版稳定后再实施：

- `scene_maps + scene_registrations + placements` 多地图版本；
- `draft/published/archived`；
- 通用 `fixture_types/fixtures`；
- 视觉Service升级Action；
- 媒体会话和asset ID跨进程缓存；
- 自动接近点和导航安全检查；
- 配置模式/比赛模式；
- 操作审计、权限和备份恢复界面；
- HTTP写入口正式退役。
