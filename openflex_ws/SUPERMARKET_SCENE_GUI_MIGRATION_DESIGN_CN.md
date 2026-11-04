# Supermarket Scene 与比赛 GUI 集成设计讨论记录

本文持续记录 `supermarket_scene` 全功能迁移到 OpenFlex 比赛 GUI 的讨论结论，后续用于生成正式修改计划、数据库迁移方案、ROS 2 接口设计和分阶段实施任务。

本文描述目标架构和已达成的设计决定，不代表当前代码已经实现。当前比赛运行方式仍以 `SUPERMARKET_COMPETITION_INTEGRATION_CN.md` 和各 ROS 包 README 为准。

第一版文件级实施顺序、接口清单、测试和验收条件见 `SUPERMARKET_SCENE_GUI_IMPLEMENTATION_PLAN_CN.md`。

## 1. 项目目标

最终只保留一个面向操作员的桌面入口：OpenFlex 比赛 GUI。

比赛 GUI 中同时提供：

- 建图、地图编辑、定位、Nav2 导航和比赛流程控制；
- 在 `map.pgm` 俯视图上配准货架、交付台和补货台；
- Three.js 三维货架、库存和商品场景；
- SKU、固定货位、库存状态和场景实体管理；
- 货架巡检、商品查询、图片拼接和人工批量录入；
- ROS 2 节点、Action、设备和任务状态显示。

界面技术边界确定为：

```text
原生 PyQt
  建图、2D 地图编辑、2D 场景配准、机器人状态、Nav2 和比赛流程

React + Three.js
  3D 场景、库存、SKU、视觉巡检、商品查询、拼图和人工录入

QtWebEngine
  将 React + Three.js 作为比赛 GUI 内部页面嵌入

supermarket_scene_ros
  场景数据、库存和视觉能力的唯一 ROS 2 后端
```

从用户视角看，所有功能都在比赛 GUI；从工程结构看，各模块继续保持独立 ROS 包和清晰职责。

## 2. 当前系统的实际结合方式

当前比赛 GUI 与 `supermarket_scene` 的直接结合主要是读取：

```text
supermarket_scene/shelf_inventory.db
```

GUI 使用 `sqlite3` 只读查询 SKU 和固定货位，根据数据库货架参数计算货位世界坐标，再通过写死的 `SHELF_FACE_TO_STATIONS` 将 `(shelf_id, face)` 映射到 `SHELF_A` 至 `SHELF_H` 等 Nav2 站点。

当前存在以下问题：

1. GUI 没有通过 `supermarket_scene_ros` 查询场景。
2. GUI 和任务执行器存在直接读取 SQLite 的路径。
3. 货架面到导航站点的映射写死在 GUI 代码中。
4. 数据库坐标和 Nav2 地图坐标依靠人工约定，没有可验证的地图绑定。
5. 地图重建或实体移动后，系统不会自动判断旧坐标已经失效。
6. GUI 第一阶段仍包含人工“模拟抓取成功”；第二、第三阶段尚未落地到该快捷页面。
7. `supermarket_scene` 的 React 页面、HTTP API 和比赛 GUI 目前是两个独立入口。

写死映射只保留为现状说明，目标架构中将被动态场景配准和接近点生成替代。

## 3. 目标总体架构

```text
┌──────────────── OpenFlex 比赛 GUI（PyQt）────────────────┐
│                                                          │
│  原生 Qt 页面                                             │
│  ├─ 建图与地图编辑                                        │
│  ├─ 2D 场景配准                                           │
│  ├─ Nav2 导航与比赛流程                                   │
│  └─ ROS 节点、Action 和设备状态                           │
│                                                          │
│  QtWebEngine 嵌入页面                                     │
│  ├─ Three.js 3D 场景                                     │
│  ├─ 库存和 SKU 管理                                      │
│  ├─ 货架巡检与商品查询                                   │
│  ├─ 图片拼接                                             │
│  └─ 人工批量录入                                         │
│              │ QWebChannel                               │
│              ▼                                           │
│       Qt SceneBridge / VisionBridge                      │
└──────────────┬───────────────────────────────────────────┘
               │ ROS 2 Service / Action / Topic
               ▼
┌──────── supermarket_scene_ros ─────────┐
│ 场景查询、事务更新、视觉任务、事件通知  │
└─────────────────┬───────────────────────┘
                  ▼
┌──────── supermarket_scene_core ────────┐
│ SQLite、几何计算、库存规则、视觉算法     │
└─────────────────────────────────────────┘
```

目标规则：

- React 不直接运行 `rclpy`，不直接操作 SQLite。
- React 通过 `QWebChannel` 调用 Qt Bridge。
- Qt Bridge 是 ROS 2 Client，负责 Service、Action 和 Topic 交互。
- 所有正式数据写入都经过 `supermarket_scene_ros`。
- SQLite 保持唯一业务真源。
- HTTP API 可暂时保留用于独立开发，但比赛运行时不依赖它。

## 4. ROS 包边界

建议最终拆分为：

```text
supermarket_scene_interfaces
  msg / srv / action

supermarket_scene_core
  数据库、坐标变换、库存规则、视觉算法
  不依赖 Qt、React 和 ROS 传输层

supermarket_scene_ros
  ROS 2 Service、Action、Topic
  调用 supermarket_scene_core

supermarket_scene_web
  React + Three.js 源码和构建后的静态资源

openflex_navigation_gui
  比赛桌面入口、2D 配准和 QtWebEngine 宿主
```

“全功能集成到比赛 GUI”不等于将所有源码塞入一个 ROS 包。应实现一个用户入口、多个职责清楚且可独立测试的包。

## 5. 场景实体尺寸和局部坐标

### 5.1 货架

货架尺寸从数据库 `shelf_types` 读取：

```text
shelf_width   局部 +X，占地宽度
shelf_length  局部 +Y，占地长度
shelf_height  局部 +Z，高度
```

数据库已有坐标约定：

```text
原点：货架占地范围的局部 min-X / min-Y 角
+X：货架宽度方向
+Y：货架长度方向
+Z：向上
face=0：-X 侧
face=1：+X 侧
yaw：局部 +X 相对 map +X 的逆时针旋转角
```

货位世界坐标继续使用：

```text
wx = world_x + lx*cos(yaw) - ly*sin(yaw)
wy = world_y + lx*sin(yaw) + ly*cos(yaw)
wz = lz
```

2D 配准页面不允许随意拉伸货架框。实体尺寸只读，用户只编辑位姿；尺寸修改必须进入专门的货架类型管理流程。

### 5.2 交付台和补货台

当前交付台尺寸仍固定在 `scene_geometry.py`：

```text
length = 1.20 m，局部 +X
width  = 0.80 m，局部 +Y
height = 0.75 m
```

目标设计不再把该尺寸写死在代码中。建议增加通用固定设施模型：

```text
fixture_types
├─ id
├─ name
├─ kind                 delivery / restock / judging
├─ length
├─ width
├─ height
└─ top_thickness

fixtures
├─ id
├─ name
├─ fixture_type_id
├─ world_x
├─ world_y
└─ yaw
```

现有 `delivery_tables` 数据应通过数据库迁移进入该结构，并在迁移期保留兼容读取接口。

### 5.3 物理框和安全框

2D 页面必须区分：

- `physical footprint`：数据库实体的真实占地尺寸，用于对齐 PGM 障碍轮廓；
- `interaction footprint`：根据机器人尺寸、安全余量和操作需求生成的外扩区域。

数据库保存物理尺寸。安全外扩由导航和比赛配置动态计算，不能混入实体尺寸。

## 6. 2D 编辑中的原点和旋转

数据库原点位于矩形角点，便于稳定计算，但用户围绕角点旋转不自然。因此 2D 页面同时显示：

- 数据库局部原点：矩形角点和局部 `+X/+Y` 坐标轴；
- 操作中心：矩形中心十字，拖动和旋转围绕此点进行；
- `face=0` 和 `face=1` 标签；
- 当前 `world_x/world_y/yaw`；
- 货位、建议接近线和机器人 footprint。

中心和数据库原点的转换为：

```text
center = origin + R(yaw) * [width/2, length/2]
origin = center - R(yaw) * [width/2, length/2]
```

用户围绕中心旋转后，GUI 自动计算并保存新的角点原点，不改变数据库已有坐标契约。

### 6.1 第一版采用人工配准

第一版不实现自动障碍轮廓识别或自动对齐。PGM 中的障碍可能经过膨胀、存在激光噪声、缺口或局部遮挡，自动算法容易产生看似精确但实际错误的结果。

操作员直接完成：

1. 从实体列表选择货架、交付台或补货台；
2. 在俯视地图中拖动实体矩形；
3. 使用旋转柄绕实体中心调整朝向；
4. 查看数据库原点、局部坐标轴和货架面商品提示；
5. 通过数值输入对 `world_x/world_y/yaw` 做精确微调；
6. 人工确认矩形与 PGM 中的实体障碍轮廓重合；
7. 保存草稿并在安全检查通过后发布。

编辑器可以提供网格、角度吸附和数值微调，但不提供“一键自动对齐”。第一版信任人工完成几何配准，不实现地图边界占用评分、实体碰撞、接近点 footprint 或自由空间连通性检查。服务端只做数值有限、实体存在、尺寸为正、版本匹配等基础数据校验。

### 6.2 原点、箭头和货架面商品提示

货架框必须在任何缩放级别下清楚显示：

```text
局部原点 O
局部 +X 箭头
局部 +Y 箭头
face=0（-X 侧）
face=1（+X 侧）
```

由于货架矩形旋转 180 度后占地轮廓完全相同，仅看 PGM 无法判断正反面。GUI 在两条长边旁分别显示该面的少量代表商品，供操作员与现场货架目视比对。

每个面的代表商品从数据库固定货位读取，建议默认显示 3 个：

- 优先显示 `actual_sku`，缺货时回退到 `expected_sku`；
- 尽量覆盖不同层和货架长度方向上的不同位置；
- 显示 SKU 短名称，并可附带层号和沿货架的 `y_cm`；
- 同款商品过多时去重，避免三个提示完全相同；
- 数据不足时明确显示“该面无商品记录”或“该位置缺货”。

示例：

```text
face=0：cracker_box L2 / sugar_box L3 / chips_can L1
face=1：bleach_cleanser L2 / mustard_bottle L3 / banana L1
```

商品提示只用于确认货架朝向，不参与坐标计算。发布前要求操作员显式确认 `face=0/face=1` 与现场一致。

## 7. PGM 像素与 ROS map 坐标

`map.pgm` 使用左上像素原点，ROS 地图使用 `map.yaml` 中定义的米制原点。所有 2D 绘制和拖动必须统一使用完整变换，不能只除以 `resolution`。

世界坐标到像素：

```text
map_local = R(-origin_yaw) * (world - origin_xy)

pixel_x = map_local_x / resolution
pixel_y = image_height - 1 - map_local_y / resolution
```

像素拖动到世界坐标使用逆变换。实现必须支持非零 `map.yaml origin yaw`，即使当前地图通常为零。

## 8. 地图身份和场景配准版本

仅使用 ROS `frame_id=map` 或地图目录名无法证明数据库坐标属于当前地图。不同地图通常都叫 `map`，同名地图也可能已经重新生成。

实体身份和尺寸全局保存；实体在地图中的位姿按地图版本保存。

建议增加：

```text
map_assets
├─ id
├─ map_name
├─ map_fingerprint
├─ pgm_hash
├─ resolution
├─ origin_x / origin_y / origin_yaw
├─ width / height
└─ created_at

scene_registrations
├─ id
├─ map_asset_id
├─ version
├─ status             draft / published / archived
├─ created_at
└─ updated_at

entity_placements
├─ registration_id
├─ entity_type
├─ entity_id
├─ world_x
├─ world_y
└─ yaw
```

不应每次配准都直接覆盖 `shelf_groups.world_x/world_y/yaw`，否则无法回到旧地图、撤销错误配准或支持多地图。

### 8.1 地图指纹

地图指纹应基于语义内容计算：

```text
解码后的 PGM 栅格字节
resolution
origin[x, y, yaw]
negate
occupied_thresh
free_thresh
```

对规范化内容计算 SHA-256。不要直接对 YAML 文本哈希，因为注释、字段顺序和 `image` 路径形式不应改变地图身份。

3D PCD 可另存 `pcd_hash`，但 2D 场景配准的核心身份是栅格内容和地图坐标参数。

### 8.2 配准状态

配准采用版本化工作流：

```text
无配准
  -> 创建 draft v1
  -> 编辑和校验
  -> 发布 published v1

published v1
  -> 基于 v1 创建 draft v2
  -> 编辑
  -> 发布 v2
  -> v1 archived
```

保存草稿和发布配准必须是两个命令：

- 保存草稿允许实体尚未全部放置；
- 发布要求所有必需实体和导航安全检查通过；
- 比赛执行器只能使用与当前地图指纹匹配的 `published` 版本。

地图名称相同但指纹变化时，应创建新配准。允许复制旧配准作为初始草稿，但不能自动视为有效。

### 8.3 新地图的初始草稿

新地图没有已有配准时，默认保留上一份布局中货架、交付台和补货台之间的相对位置与相对朝向，而不是将所有实体分别放入未放置列表。

初始草稿生成顺序：

1. 优先选择用户明确指定的源配准；
2. 未指定时选择最近一次已发布配准；
3. 没有已发布配准时使用旧数据库位姿作为 legacy 源布局；
4. 新增且从未有位姿的实体仍进入未放置列表。

计算源布局所有实体物理 footprint 的整体包围框中心 `C_source`，计算目标地图可视范围的世界坐标中心 `C_map`，然后只施加统一平移：

```text
p_draft = p_source + (C_map - C_source)
yaw_draft = yaw_source
```

初始生成不缩放实体间距，也不自动旋转布局。地图分辨率变化不能导致物理场景缩放。

2D 编辑器提供临时“整体布局”选择，可对全部已放置实体执行统一平移和统一旋转，再进入单个实体微调。整体旋转保持相对布局：

```text
p_new = C_group_new + R(delta_yaw) * (p_old - C_group_old)
yaw_new = yaw_old + delta_yaw
```

整体布局不是数据库实体，只是编辑器中的多选操作。生成结果始终是草稿，必须由操作员人工确认后才能发布。

### 8.4 第一版发布前检查

第一版不实现复杂的自动场景和导航安全检查。发布只检查：

1. 请求中的实体 ID 在数据库中存在；
2. `world_x/world_y/yaw` 是有限数值；
3. 关联类型存在且物理尺寸为正；
4. 请求的草稿或配准版本与数据库当前版本一致；
5. 发布过程使用数据库事务，失败时不产生半更新状态。

货架框是否覆盖正确障碍、实体是否合理重叠、货架面方向是否正确，均由操作员在 2D 和 3D 视图中人工确认。地图范围、机器人 footprint、自由空间连通性和自动导航安全检查作为后续增强项，不阻塞第一版落地。

## 9. 2D 与 3D 的统一状态

2D 和 3D 不各自维护一份场景。比赛 GUI 内建立统一 `SceneStore`：

```text
ROS 数据库确认状态
        │
        ▼
  confirmedScene
        │
   创建编辑草稿
        ▼
    draftScene
      /       \
Qt 2D 视图   React 3D 视图
```

用户在 2D 中拖动货架时：

1. 只更新 `draftScene`；
2. Qt 2D 框实时移动；
3. Qt 通过 QWebChannel 发出 `sceneDraftChanged`；
4. Three.js 实时移动同一实体；
5. 所属货位、接近点和安全框一起重新计算；
6. 未点击发布前不修改正式数据库状态。

点击取消时丢弃草稿；点击发布时比较草稿与确认状态、显示差异、执行服务端事务，成功后重新读取数据库确认。

### 9.1 空间位姿的两个编辑入口

货架、交付台、补货台和其他固定实体保留两种空间编辑方式，并且都修改同一个 SceneStore：

```text
Qt 2D 配准页面
  在地图上用鼠标拖动和旋转实体框

React 场景管理页面
  保留现有 world_x / world_y / yaw 数值编辑
```

Three.js 3D 画布本身不提供鼠标拖动、旋转柄或改变实体高度的操作，避免透视视角下的不精确空间编辑。React 页面仍完整保留：

- 从不同视角观察场景；
- 选择和高亮货架、货架面和固定货位；
- 检查局部坐标轴、朝向、层级和商品布局；
- 通过表单精确编辑货架和台子的 `world_x/world_y/yaw`；
- 编辑 SKU、expected/actual SKU、商品尺寸和商品图片等业务数据。

React 数值编辑和 Qt 2D 图形编辑必须走同一个保存接口和刷新机制。任何一侧保存成功后，Qt SceneStore 更新并同步刷新另一侧。

### 9.2 正式坐标和草稿预览的计算职责

`supermarket_scene_core` 是正式几何计算的唯一权威实现。ROS 场景快照对每个 slot 同时返回：

```text
local_x / local_y / local_z
world_x / world_y / world_z
yaw
```

局部坐标由后端根据 `shelf_type`、`face`、`level`、`y_cm` 和商品尺寸计算；正式世界坐标由后端根据已保存配准计算。

用户在 2D 页面连续拖动时，不为每个鼠标事件调用 ROS 服务。Qt SceneStore 使用后端已经返回的 `local_x/local_y/local_z`，只执行刚体变换生成临时预览：

```text
draft_world = R(draft_yaw) * local + draft_origin
```

Qt 不重复实现层板高度、双面深度和货位中心等业务公式。保存或发布后，服务端重新计算正式坐标并返回新快照，GUI 用服务端结果替换本地预览。

配准写请求只提交实体位姿，不提交每个 slot 的世界坐标。slot 世界坐标始终是派生数据，不能由 GUI 直接写入数据库。

稳定 ID 在所有视图和接口中保持一致，例如：

```text
shelf:1
fixture:delivery:1
slot:1-0-2-135
station:shelf:1:face:0:segment:1
```

## 10. React、Qt 和 ROS 2 的通信

React 不直接依赖 DDS 或 ROS 消息类型。前端只使用普通异步业务接口：

```text
sceneApi.getScene()
sceneApi.saveDraft()
sceneApi.publishRegistration()
inventoryApi.updateSlot()
visionApi.inspectShelf()
```

调用链为：

```text
React
  -> QWebChannel
  -> Qt SceneBridge / VisionBridge
  -> ROS 2 Client
  -> supermarket_scene_ros
```

QWebChannel 只传递 JSON 兼容结构。Qt Bridge 负责：

- ROS 消息和前端 DTO 的转换；
- Service 超时和错误处理；
- Action 进度、取消和结果；
- Topic 事件转为前端信号；
- Qt 线程与 rclpy 执行器之间的线程边界。

## 11. ROS 2 接口使用原则

快速查询和事务更新使用 Service，例如：

```text
/supermarket_scene/get_scene
/supermarket_scene/get_map_registration
/supermarket_scene/save_registration_draft
/supermarket_scene/publish_map_registration
/supermarket_scene/get_inventory
/supermarket_scene/update_slot
```

长时间视觉任务使用 Action，例如：

```text
/supermarket_scene/inspect_shelf
/supermarket_scene/find_sku
/supermarket_scene/stitch_images
/supermarket_scene/import_shelf_face
```

数据变化通知使用 Topic，例如：

```text
/supermarket_scene/scene_changed
/supermarket_scene/inventory_changed
/supermarket_scene/registration_changed
```

### 11.1 场景快照查询

新增一个面向 GUI 的场景快照服务，建议命名：

```text
/supermarket_scene/get_scene_snapshot
```

该服务一次返回数据库中的完整、只读场景快照：

- 当前数据库和场景修订号；
- 当前地图配准 ID、版本和地图身份；
- 所有货架实例及其解析后的货架类型尺寸；
- 所有交付台、补货台等固定设施及其类型尺寸；
- SKU 目录；
- 所有固定货位、expected/actual SKU 和派生状态。

ROS 服务不负责挑选“每个面显示哪三个商品”。代表商品属于 GUI 展示策略，由 Qt SceneStore 从完整货位列表中选择。这样 2D 代表商品、3D 全部商品和库存页面共享同一份权威快照，不会形成多个含义不同的查询接口。

建议请求支持：

```text
string map_fingerprint
bool include_inventory
```

建议响应包含：

```text
bool success
string message
uint64 scene_revision
MapRegistration registration
ShelfScene[] shelves
FixtureScene[] fixtures
SkuInfo[] skus
SlotScene[] slots
```

其中 `ShelfScene` 应直接携带已经解析的 `shelf_length/shelf_width/shelf_height` 等尺寸，而不是要求 GUI 根据 `shelf_type_id` 再做数据库式关联。响应也可以保留 `shelf_type_id`，用于类型管理和追踪来源。

`scene_revision` 在任意场景、库存或配准写入后递增，用于缓存刷新和防止旧界面覆盖新数据。查询服务只返回结构化数据，不返回商品图片二进制或视觉调试产物。

### 11.2 人工配准保存

第一版建议提供两个写接口：

```text
/supermarket_scene/save_registration_draft
/supermarket_scene/publish_map_registration
```

两者都接收地图身份、`expected_version` 和实体位姿数组。服务端只做基础数据校验并使用一个 SQLite 事务保存，不执行障碍匹配、碰撞或导航可达性判断。

大图片不应长期塞入普通 Service。后续需要专门确定 `sensor_msgs/CompressedImage`、Action Goal、临时资源 ID 和结果存储的边界。

### 11.3 视觉功能的 ROS 接口

视觉功能可以全部接入 ROS 2，但不应把所有操作都设计为普通 Service。

短时间、立即返回的操作使用 Service：

```text
获取和保存视觉配置
创建或查询图片资源会话
应用人工确认后的巡检结果
更新 SKU 描述和货位数据
读取模型和视觉节点状态
```

耗时、需要反馈和取消的操作使用 Action：

```text
InspectShelf
FindSku
StitchImages
AnalyzeManualImport
```

Action 统一反馈：

```text
string stage
float32 progress
string message
```

React 发起任务后由 Qt Bridge 创建 ROS Action Goal；进度反馈通过 QWebChannel 返回 React，用户取消时 Qt 调用 Action cancel。视觉任务只生成候选结果，不自动更新库存；用户在界面确认后，再调用独立 Service 应用修改。

### 11.4 图片选择、上传和结果资源

嵌入式 React 页面不再把原始图片编码为 Base64 JSON。建议由 React 请求 Qt 打开原生文件选择器：

```text
React
  -> mediaBridge.chooseImages()
  -> QFileDialog
  -> Qt 读取和验证本地图片
```

该方案已确定为第一版唯一图片选择入口。React 不再使用浏览器 `<input type="file">` 读取大图。

Qt 为每张已选图片创建临时媒体记录：

```text
asset_id
original_name
mime_type
width / height
compressed_size
content_hash
internal_file_path 或内存数据
```

Qt 只向 React 返回：

```text
asset_id
original_name
width / height
compressed_size
preview_uri
```

不向 React 暴露本机绝对路径或原始字节。预览统一使用 Qt 自定义资源 URL，例如：

```text
supermarket-resource://local-media/{asset_id}
```

React 提交视觉任务时只把 `asset_id` 交回 Qt。Qt 从自己的临时媒体存储读取压缩字节并构造 `sensor_msgs/CompressedImage`，因此大图不会经过 JavaScript Base64 或 QWebChannel JSON。

Qt 应保留原始压缩图片供视觉服务使用，同时生成较小缩略图供 WebEngine 预览，不能为了界面预览静默降低视觉算法输入分辨率。沿用现有单图大小限制并校验图片可解码；多图拼接还需检查总数量和总压缩大小。

拼接和视觉任务返回的图片也进入同一个 Qt 临时媒体存储，React 通过新的 `asset_id/preview_uri` 查看，并可将拼接结果直接传给人工录入页面，不再通过 HTTP URL 重新下载。

临时媒体在页面任务结束、用户明确清理、TTL 到期或 GUI 退出时释放。正在被视觉请求或后续人工录入引用的资源必须持有引用，不能提前删除。

单张巡检和商品查询第一版可直接把压缩图片放入 Action Goal 的 `sensor_msgs/CompressedImage`。

图片拼接涉及 2 至 8 张较大图片，不建议在一个 Action Goal 中一次携带全部大图。建议使用临时媒体会话：

```text
CreateMediaSession
  -> 返回 session_id

UploadMedia
  -> 每次上传一张 CompressedImage
  -> 返回 asset_id 和 content_hash

StitchImages Action
  -> 只提交 session_id、asset_id[] 和主图索引
```

视觉结果图片、调试图和拼接图返回稳定 `asset_id`，由 Qt 自定义资源 URL 懒加载。结构化识别结果直接放在 Action Result 中。临时资源需要 TTL、显式清理和大小限制，不能无限积累。

第一版仍可复用当前视觉算法和配置逻辑，只替换 HTTP/Base64 传输层；模型继续在独立视觉进程中预加载。

### 11.5 第一版视觉接口简化

为降低第一版改造量，当前已有的 `InspectShelf.srv` 和 `FindSku.srv` 直接保留。图片拼接、商品框选、SKU 提示生成和透视校正等新增视觉接口也允许第一版先使用异步 Service，而不强制立即改成 Action。

Qt 必须使用 `call_async()` 和现有 rclpy/Qt 非阻塞执行循环，不能在 GUI 主线程同步等待服务完成。React 在等待期间显示忙碌状态；第一版可以暂不提供细粒度进度和服务端取消。

满足以下任一条件后，再将对应接口升级为 Action：

- 单次任务经常超过数秒且需要阶段进度；
- 用户需要可靠取消 GPU 推理或拼接；
- 任务需要连续反馈中间候选；
- 超时后必须确保服务端停止计算；
- 多任务需要排队和明确任务 ID。

第一版图片直接使用 `sensor_msgs/CompressedImage`，由 Qt 在发送前限制分辨率、格式和压缩后大小。图片拼接可以先使用 `CompressedImage[]`；若实际 DDS 传输或内存测试表明多图消息过大，再启用前述媒体会话和 `asset_id` 方案。

任务执行器请求和场景坐标返回应携带：

```text
map_fingerprint
registration_id
registration_version
frame_id
```

Nav2 当前地图与场景发布版本不匹配时，依赖货架坐标的比赛动作必须拒绝执行。

## 12. 动态生成导航接近点

目标架构中删除写死的 `SHELF_FACE_TO_STATIONS`。接近点根据实体配准位姿、货位位置、机器人 footprint 和安全距离生成。

基本思路：

```text
slot 世界坐标
  -> 投影到对应货架面
  -> 沿货架外法向移动 approach_distance
  -> 设置车头朝向货架面
  -> 用 Nav2 栅格和机器人 footprint 做安全检查
```

同一货架面可根据机械臂横向可达范围生成多个分段接近点。候选点不安全时，可沿货架长度方向搜索附近安全点，但不能静默改变目标货位语义。

数据库计算出的 `world_z` 只作为名义检测位置或 IK 初值，不是升降电机指令。真实操作仍需局部 RGB-D 检测、精停和 MoveIt 规划。

## 13. 3D 页面嵌入和部署

React 构建产物随 `supermarket_scene_web` ROS 包安装，例如：

```text
share/supermarket_scene_web/dist/
```

正式运行时由 `QWebEngineView` 加载本地静态资源，建议注册自定义 URL scheme：

```text
supermarket://app/index.html
```

正式比赛不依赖 Vite、Node 开发服务器或固定 localhost 端口。开发环境仍可使用 Vite 热更新。

### 13.1 商品采用现有轻量长方体显示

3D 页面不加载 SKU 的 OBJ、PLY、复杂网格或完整材质。继续沿用现有 `WarehouseScene.tsx` 的轻量表达：

```text
商品几何：Three.js BoxGeometry
局部 X 厚度：固定约 0.03 m
局部 Y 宽度：slot.width_cm
局部 Z 高度：slot.height_cm
朝外表面：显示该 slot 的一张正面图片
其余五面：使用 SKU 颜色
```

`face=0` 时图片贴在长方体局部 `-X` 面；`face=1` 时贴在局部 `+X` 面。商品使用货架配准后计算出的 `world_x/world_y/world_z/yaw` 放置。

状态显示继续保留：

- 正常商品：彩色长方体和单面贴图；
- 缺货：半透明线框位置标记；
- 摆放错误：商品外框使用红色强调；
- 选中：高亮当前商品和所属货架。

`sku_catalog.mesh_file` 和 `tex_file` 可以暂时保留用于数据库兼容，但 Three.js 场景不依赖它们，场景快照也不需要为渲染而传输网格资源。

### 13.2 商品图片资源

当前 Web 页面通过 `/api/item-images/{slot_id}/0.png` 加载图片。嵌入比赛 GUI 后正式运行不应依赖原 HTTP API 路径。

建议由 Qt 注册只读自定义资源 URL：

```text
supermarket-resource://slot/{slot_id}/thumbnail
```

React 和 Three.js 只持有该稳定资源 URI。Qt 资源处理器按需获取、缓存并返回压缩缩略图。图片接口只返回显示用缩略图，不把原始巡检照片或整套 `image_dir` 暴露给前端。

第一版可新增只读 ROS 资源服务，例如：

```text
/supermarket_scene/get_scene_asset
```

请求使用 `slot_id` 和资源类型，响应返回 `sensor_msgs/CompressedImage`、内容哈希和 MIME 类型。Qt 按内容哈希缓存，Three.js 仍通过普通纹理 URL 使用图片，不感知 ROS 传输过程。

不建议在 `GetSceneSnapshot` 中为所有商品内嵌 Base64 图片，避免场景查询响应过大和每次库存刷新都重复传输图片。

### 13.3 3D 地板和场景范围

3D 页面不把 `map.pgm` 作为地面纹理，不根据占用栅格生成墙体、障碍网格或导航场景。当前实体规模只有两个货架组和两个台子，继续使用简洁、低负载的三维展示：

```text
纯色平面地板
轻量网格线
WORLD 坐标轴
两个货架组
两个固定台子
货架上的固定货位和商品长方体
```

地板的物理范围和姿态从当前地图元数据获得，使用 `map.yaml` 的 origin、PGM 宽高和 resolution 计算，但不显示黑白栅格内容。这样 3D 坐标仍与 ROS `map` 一致，同时保持画面简洁。

透视和俯视相机根据已放置实体的整体物理包围框自动取景，不再依赖写死的 `6m x 8m` 地板和固定相机目标。2D 页面仍是地图配准的唯一权威视图，3D 页面不用于判断障碍或导航可达性。

## 14. 进程和故障边界

大模型不能加载在比赛 GUI 进程中。建议保持：

```text
GUI 进程
  PyQt + rclpy + QtWebEngine

场景服务进程
  SQLite + 普通查询和事务更新

视觉服务进程
  DINO / OWLv2 / VLM / 图片拼接
```

视觉进程崩溃或显存不足时，导航 GUI 应继续运行，只把对应 Action 标记为失败并允许重启视觉节点。

数据库正式写入由一个场景服务进程拥有。GUI、React、任务执行器和视觉节点不得绕过服务直接写 SQLite。

### 14.1 第一版节点拆分

第一版即拆分为两个 ROS 2 节点，并由同一个 launch 文件启动：

```text
supermarket_scene_data_node
  GetSceneSnapshot
  货架、台子、类型、SKU 和货位 CRUD
  ApplySceneLayout
  巡检结果确认写入
  SQLite 唯一写入者

supermarket_scene_vision_node
  InspectShelf
  FindSku
  StitchImages
  GroundProducts
  GenerateSkuPrompts
  DINO / OWLv2 / VLM 和图像算法
  不写 SQLite
```

视觉节点可以在任务开始时以只读方式获取数据库快照和参考图片，但不能直接修改库存。视觉结果携带其观测所基于的 `scene_revision`；用户确认应用时由 data node 检查版本，避免把旧识别结果覆盖到已经变化的库存。

建议统一启动入口：

```text
ros2 launch supermarket_scene_ros supermarket_scene_system.launch.py
```

data node 即使在 vision node 模型加载、推理、显存不足或崩溃时也必须继续响应 2D 配准、场景查询和 CRUD。

第一版 data node 作为数据库唯一写入者；vision node 如需直接只读 SQLite，应启用 WAL、只读连接和 busy timeout。长期可进一步改为由 vision node 通过 data node 的内部查询接口获取不可变任务快照。

## 15. 已达成的设计决定

1. 比赛 GUI 是唯一操作员桌面入口。
2. PyQt 负责机器人、地图和 2D 配准；React + Three.js 负责 3D、库存和视觉工作台。
3. React 页面通过 QtWebEngine 嵌入，通过 QWebChannel 连接 Qt ROS Bridge。
4. 正式业务数据全部经过 `supermarket_scene_ros`，不再由 GUI 直接访问 SQLite。
5. 货架尺寸从 `shelf_types` 读取，配准页只编辑位姿。
6. 交付台、补货台等固定设施的尺寸迁移到数据库类型表。
7. 数据库存储角点原点，GUI 围绕实体中心编辑并自动换算。
8. 2D 和 3D 使用同一个 `SceneStore` 和相同稳定 ID。
9. 场景位姿按地图版本保存，不直接覆盖为一套全局坐标。
10. 地图使用语义指纹绑定，草稿和已发布配准分离。
11. 比赛任务只能使用与当前地图指纹匹配的已发布配准。
12. 货架接近点由场景几何和地图安全检查动态生成，不再依赖写死映射。
13. 大模型和视觉算法运行在独立进程，不能拖垮比赛 GUI。
14. 第一版 2D 配准采用人工移动和旋转实体框，不实现自动对齐。
15. 货架框显示局部原点、`+X/+Y` 箭头、face 标签和每个面的代表商品，人工确认正反面。
16. 第一版不实现自动地图碰撞、连通性或接近点安全检查，几何正确性由操作员确认。
17. 新增统一 `GetSceneSnapshot` ROS 2 服务，返回完整尺寸、位姿、SKU 和货位数据；代表商品由 GUI 选择。
18. Three.js 商品继续使用规则长方体、SKU 颜色和朝外单面图片，不加载商品网格或复杂材质。
19. 商品缩略图通过 Qt 自定义资源 URI 懒加载和缓存，不内嵌到完整场景快照。
20. 2D 页面提供实体框拖动和旋转；React 保留 `world_x/world_y/yaw` 数值编辑，但 Three.js 画布不直接拖动实体。
21. 正式货位局部和世界坐标由 `supermarket_scene_core` 计算；Qt 仅对拖动中的草稿做局部坐标刚体变换预览。
22. 新地图的初始草稿保留最近布局的实体相对位置和朝向，整体平移到目标地图中心附近，不自动缩放或旋转。
23. 3D 页面使用简洁纯色地板和网格，不显示 PGM 纹理、墙体或导航障碍；场景范围从地图元数据获得。
24. React 完整保留货架、台子、SKU 和货位的新增、修改、删除功能；第一版不增加比赛模式或配置模式限制。
25. 所有本地图片统一通过 Qt `QFileDialog` 选择；React 只接收临时 asset ID 和预览 URI，不传输 Base64 或本机绝对路径。
26. 第一版即拆分 `supermarket_scene_data_node` 和 `supermarket_scene_vision_node`，由统一 launch 启动；data node 是 SQLite 唯一写入者。

## 18. 第一版简化范围

第一版优先实现可用闭环，不要求一次落地全部长期架构。长期目标仍保留地图版本、正式发布配准、资源服务和完整 ROS 化，但第一版可以按以下范围实施：

1. 比赛 GUI 嵌入现有 React + Three.js 页面；
2. React 保留现有全部 CRUD 和空间数值编辑；
3. Qt 新增 2D 场景配准页，从数据库读取货架和台子尺寸；
4. 2D 页面支持人工拖动、旋转、原点/箭头、face 标签和代表商品；
5. 新增 `GetSceneSnapshot` ROS 服务，统一返回当前数据库场景；
6. 新增批量位姿更新服务，点击确认后在一个事务中更新货架和台子当前位姿；
7. 2D、React 和 Three.js 通过 Qt SceneStore 同步；
8. Three.js 保持当前长方体商品和单面贴图实现；
9. 不实现自动对齐、地图碰撞、可达性、比赛模式或权限限制；
10. 第一版允许继续使用当前 `shelf_groups.world_x/world_y/yaw` 和 `delivery_tables.world_x/world_y/yaw` 作为当前场景位姿；
11. 地图版本化 placement 表、draft/published 工作流和旧字段退役放到后续数据库迁移阶段。

第一版仍要求所有新 GUI 写入经过 ROS 服务，不能新增直接写 SQLite 的路径。现有 HTTP API 可以在迁移期间保留，待 React 接口逐项切换完成后再退役。

## 16. 后续讨论清单

以下项目尚需逐项确定，讨论完成后继续更新本文：

1. 2D 配准编辑器的具体工具栏、网格/角度吸附、数值微调和撤销策略；
2. 自动生成货架、交付台和补货台接近点的精确几何规则；
3. 数据库迁移的最终表结构、兼容策略和版本升级方式；
4. `GetScene`、草稿保存和发布配准等 ROS 2 msg/srv 定义；
5. QWebChannel DTO、事件模型和错误模型；
6. React 现有 HTTP API 到 Qt Bridge 的迁移方法；
7. 图片、调试产物和长时间视觉 Action 的传输方案；
8. Three.js 场景如何显示地图、机器人、TF、接近点和视觉结果；
9. SceneStore 的所有权、Qt/React 状态同步和冲突处理；
10. 地图指纹获取方式及其与 Nav2 map_server 的运行时校验；
11. 场景服务、视觉服务、GUI 和任务执行器的 launch 生命周期；
12. 权限、比赛模式锁定、操作审计、备份和恢复；
13. 自动化测试、数据库迁移测试、ROS 接口测试和 GUI 截图测试；
14. 分阶段实施计划、兼容过渡和旧代码删除条件。

## 17. 预期实施阶段

以下仅作为当前讨论顺序，待上述问题明确后再生成正式实施计划：

```text
阶段 1：场景数据模型、地图身份和 ROS 查询接口
阶段 2：Qt 2D 场景配准页和版本化发布
阶段 3：动态接近点生成与导航安全检查
阶段 4：Three.js 本地嵌入和 SceneStore 实时同步
阶段 5：库存、SKU 和人工录入迁移
阶段 6：巡检、查询、拼图等视觉 Action 迁移
阶段 7：任务执行器去除 SQLite 和写死站点依赖
阶段 8：系统联调、数据迁移、测试和旧入口退役
```

## 19. 2026-08-11 阶段性复盘与导航下一步

### 19.1 当前已经完成的第一版闭环

当前已经完成从数据库场景到比赛 GUI 的第一版可用闭环：

1. 比赛 GUI 已嵌入原生 Qt 2D 场景配准页和 React + Three.js 场景管理页；
2. 2D 配准页可从比赛地图下拉框选择地图，并在 `map.pgm` 上拖动、旋转货架和交付桌；
3. 实体框使用数据库米制尺寸与地图 `resolution` 绘制，显示局部原点、坐标轴、货架面方向和代表商品；
4. `ApplySceneLayout` 通过 data node 在一个事务中保存货架、台子位姿和地图绑定；
5. `GetSceneSnapshot` 返回货架、台子、SKU、货位、局部坐标、派生世界坐标、地图绑定和 `scene_revision`；
6. Three.js 页面当前只展示两个货架、两个交付桌和 `8m x 8m` 简易方格地面，缺少图片时降级为 SKU 彩色方块；
7. 数据库实体已经整理为货架 ID `1、2` 和交付桌 ID `1、2`，35 个货位完整保留；
8. 当前配准绑定到 `supermarket_test_08`，场景修订为 `10`。

商品的 `world_x/world_y/world_z` 不是单独重复保存的一组自由坐标。数据库保存货架局部原点位姿、货架尺寸以及货位的 `face/level/y_cm` 等局部参数，查询快照时由 `supermarket_scene_core` 做刚体变换得到商品在 ROS `map` 坐标系中的世界坐标。这样修改货架配准后，所属商品坐标会一致更新。

### 19.2 修改前尚未打通的导航断点

场景配准已经成为商品位置的权威来源，但本阶段修改前比赛导航尚未消费这套动态几何：

```text
当前实际流程

数据库商品货位
  -> GUI 根据 shelf_id + face 查询写死映射
  -> SHELF_A ... SHELF_H
  -> competition_mission.yaml / stations YAML 固定位姿
  -> Nav2
```

修改前 `competition_flow_tab.py` 中存在 `SHELF_FACE_TO_STATIONS`，而且第二个货架仍使用迁移前的旧 ID `3`。该 GUI 遗留已在 19.7 所述过渡实现中删除。任务执行器的 `shelf_face_stations`、`source_station`、`destination_station` 和站点 YAML 目前仍是固定站点模型；任务协调器虽然把数据库商品世界坐标写入 `observed_pose`，最终导航和操作仍要求一个固定 `source_station`，留待后续阶段处理。

因此，修改前 2D 配准可以正确改变数据库和 3D 中的实体/商品位置，但不会改变比赛流程选择的货架站点。19.7 已先完成基于场景世界坐标选择 A-H 的过渡闭环；连续动态生成 Nav2 接近位姿仍属于后续阶段。

### 19.3 商品坐标与底盘导航目标必须分层

可以开始修改导航点逻辑，但不能把商品的 `world_x/world_y` 直接作为 `NavigateToPose` 目标。商品坐标位于货架内部或层板上，是视觉、抓取和 IK 的目标；Nav2 需要的是货架外部可容纳完整机器人 footprint 的 `base_link` 接近位姿。

```text
商品世界坐标 / 货位语义
  -> 根据货架 face 找到外侧工作面
  -> 沿外法向偏移机器人前半长 + 作业间隙
  -> 沿货架长度方向对齐目标货位，并加入机械臂横向标定偏移
  -> yaw 朝向货架
  -> GoalSafetyMap 检查 footprint、地图边界和障碍净距
  -> 动态 Nav2 粗接近位姿
  -> 现有 shelf_scan 激光精停
  -> 视觉检测与机械臂动作
```

货架局部坐标约定建议保持为：`face=0` 从局部 `-X` 一侧接近，车头朝局部 `+X`；`face=1` 从局部 `+X` 一侧接近，车头朝局部 `-X`。纵向初值使用货位 `local_y`，但应预留 `tool_lateral_offset` 参数，不能在未标定时假定右臂工作中心永远与 `base_link` 中线重合。

交付桌还缺少明确的可操作面语义。第一版建议规定交付桌局部 `+Y` 为操作面，并在 2D 框上显示工作面箭头；若未来台子可能从不同方向接近，再增加持久化的 `approach_side` 字段。两个台子的业务角色也应稳定绑定为交付台和补货台，不能只依靠当前显示名称猜测。

### 19.4 推荐的下一阶段实施顺序

1. 在共享 core 中实现无 ROS、无 Qt 的纯几何 `ApproachPoseResolver`，输入场景实体、货位和机器人接近参数，输出名义 `(x, y, yaw)` 及推导依据；
2. 为货架两面和交付桌操作面添加单元测试，覆盖货架平移、任意旋转、两个 face、纵向货位和角度归一化；
3. 在比赛流程页或比赛导航页显示选中商品对应的接近点、车头方向和完整 footprint，先做人工核对，不立即导航；2D 配准页保持单一职责，只编辑和确认场景实体配准；
4. 复用现有 `GoalSafetyMap` 做静态地图安全检查。不安全时明确拒绝，第一版不自动搜索或静默替换其他点；
5. 新增统一 ROS 查询接口 `ResolveApproachPose`，返回商品坐标、动态底盘接近位姿、实体 ID/face、地图绑定、`scene_revision` 和安全结果；
6. 比赛 GUI 第一阶段改为使用动态接近位姿，移除 `SHELF_FACE_TO_STATIONS`，交付目标也由交付桌配准生成；
7. 保留 `START_CORRIDOR` 等与场景实体无关的人工导航点，保留现有 `shelf_scan` 精停、观察姿态、模拟抓取和安全退避链；
8. 完成实车低速联调后，再让任务协调器通过 ROS 获取 SKU 和接近位姿，逐步移除其直接 SQLite 读取、`shelf_face_stations` 和固定 `source_station` 依赖。

第一步不应直接大范围删除 A-H 站点配置。应先将它们保留为回归基准，对比动态计算出的位姿和现有实测点；动态流程通过比赛导航页预览、地图安全检查和低速实车验证后，再删除失去用途的货架固定点。这样不影响现有可运行的精停和机械臂示教链。

### 19.5 当前配准与旧固定站点的实测差异

以当前数据库配准位姿反算 `supermarket_test_08` 的 A-H 固定站点到货架局部坐标后：

- 两个货架 `face=0` 一侧的底盘中心位于局部 `X=-0.590~-0.633m`；
- 两个货架 `face=1` 一侧的底盘中心位于局部 `X=1.394~1.429m`；
- 当前货架局部占地 `X` 范围为 `0~1.01m`，所以 `face=1` 的外侧距离只有约 `0.384~0.419m`；
- 每个面的两个旧站点位于局部 `Y` 约 `0.38~0.51m` 和 `1.30~1.38m`，是两个离散作业区，不是逐商品对齐。

两侧旧站点相对货架外表面的距离明显不对称。它可能包含旧地图人工测点、底盘参考点、激光位置、机械臂横向可达范围或旧场景几何误差，现阶段不能直接判断哪一项是主因。因此动态接近点第一版应在比赛导航页的运行诊断视图中绘制“旧固定点、动态候选点、机器人 footprint、目标货位”，不向 2D 配准页增加导航功能。使用实际低速精停结果标定：

```text
nav_standoff
tool_lateral_offset_right
tool_lateral_offset_left（后续）
```

`shelf_scan` 的 `target_clearance=0.05m` 是最终激光精停净距，不应直接等同于 Nav2 粗接近点的 `nav_standoff`。后者应留出足够距离让 Nav2 停稳并让精停算法仍能可靠看到目标货架。

### 19.6 先保留 A-H 的过渡导航方案

讨论决定：近期先不生成全新的连续 Nav2 接近位姿，保留 `SHELF_A...SHELF_H` 作为已经测量过的候选导航点，但删除 `(shelf_id, face) -> 固定站点列表` 的写死映射，改为根据场景快照和候选点几何关系选择。

数据库正式货架 ID 只允许 `1、2`。代码中旧货架 ID `3` 是历史数据迁移后的遗漏，不代表数据库仍有 3 号货架；后续实现必须删除所有业务代码中的旧 ID `3` 假设。未来系统按两个货架设计，不为第三个货架保留兼容分支。

选择商品后，不解析 `slot_id` 字符串推断字段。`slot_id` 作为稳定标识使用，通过 ROS 场景查询直接取得结构化的：

```text
shelf_id
face
level
local_x/local_y/local_z
world_x/world_y/world_z
shelf_yaw
```

其中 `face` 和商品世界坐标用于选择导航点，`level/world_z` 用于货架观察、视觉和机械臂层高逻辑。层高不参与 A-H 底盘点的平面距离计算。

比赛流程不再直接打开 SQLite，而是复用 `GetSceneSnapshot`/SceneStore。开始选择前必须确认 `requested_map_matches=true`，保证商品世界坐标、当前 Nav2 地图和 A-H mission 航点处于同一个 `map` 坐标定义下。

A-H 候选选择不能只做无条件欧氏最近邻。当前中间通道的 `SHELF_C/D` 与 `SHELF_E/F` 位置非常接近但朝向相反，只看位置可能选中另一排货架的点。过渡算法应为：

```text
目标 slot + 所属 shelf
  -> 根据 shelf yaw 和 face 计算该面的外法向、期望车头 yaw
  -> 从当前 mission 中取得 role=shelf_approach 的 A-H 候选点
  -> 过滤掉不在目标货架面外侧的点
  -> 过滤掉车头 yaw 与目标工作面不一致的点
  -> 在剩余候选中按商品 world_x/world_y 的平面距离选最近点
  -> GoalSafetyMap 再检查该固定点
  -> 返回 A-H 点、商品层高和选择依据
```

若货架在新地图中移动或旋转过大，导致没有方向和外侧关系都合格的 A-H 点，系统必须报告“当前固定接近点不适用于该配准”，不能退化成无条件最近点。这个限制说明 A-H 最近点法是过渡方案：它消除了货架 ID/face 写死映射并立即消费数据库世界坐标，但仍不能替代后续真正随场景几何连续生成的动态接近位姿。

若一个 SKU 在多个货位都有实际商品，应为每个当前支持层高的货位计算最近合格 A-H 点，再选择总代价最小的“货位 + 航点”组合。第一版继续使用已示教的第 2、3 层；没有示教姿态的层不能仅因平面距离更近而被选中。

本过渡阶段只替换货架 A-H 的选择逻辑。交付流程暂时继续使用当前地图中已经验证的 `delivery_counter`，交付桌动态接近点留到连续接近位姿阶段处理。

选择结果和诊断信息放在比赛流程页，例如：

```text
商品：<SKU / slot_id>
货架：1 或 2
面：face 0 / face 1
层：level
商品 map 坐标：(x, y, z)
选择航点：SHELF_A ... SHELF_H
平面距离、方向误差、安全检查结果
```

2D 配准页不显示、编辑或测试 A-H 导航点，继续只负责货架和台子的地图配准。

### 19.7 A-H 过渡选择已实现

2026-08-11 已完成 19.6 的第一版实现：

1. 新增纯几何 `competition_flow_core.compatible_approach_waypoints`，根据货架配准位姿、`face`、货位局部/世界坐标和 mission 航点计算兼容候选；
2. 删除 GUI 中的 `SHELF_FACE_TO_STATIONS`、`select_station_for_slot` 以及旧货架 ID `3`；比赛场景启动校验要求货架 ID 集合严格等于 `{1,2}`；
3. `CompetitionFlowTab` 不再读取 SQLite，也不再重复计算商品世界坐标，改为异步调用 `GetSceneSnapshot`；
4. 场景查询携带当前地图名称和地图语义指纹，只有 `requested_map_matches=true` 才允许选择商品；切换地图后自动丢弃旧快照并重新查询；
5. SKU 下拉框来自 `actual_sku`，不会把缺货位置或仅有期望 SKU 的货位作为实际可抓商品；
6. 一个 SKU 有多个实例时，只比较当前已示教第 2、3 层的货位，为每个货位先过滤错误面、错误车头方向的航点，再选择世界平面距离最小的“货位 + 航点”组合；
7. 选择结果显示 `slot_id`、货架 1/2、face、层高、商品 `map` 坐标、A-H 点、距离、方向误差和 `scene_revision`；
8. 最近点仍需通过现有 `GoalSafetyMap`，失败时显示原因并禁用第一阶段启动；
9. 2D 配准页没有增加任何导航功能或控件，继续保持配准单一职责；
10. 本阶段只替换比赛 GUI 的货架点选择；交付仍使用 `delivery_counter`，任务执行器的固定站点模型尚未迁移。

验证结果：

```text
纯几何和 GUI 全量测试：74 passed
工作区构建：openflex_navigation_gui 成功
真实 supermarket_test_08 场景联算：35/35 个货位均有方向兼容 A-H 候选
A-H 静态地图安全检查：8/8 通过
真实 ROS 快照握手：地图 supermarket_test_08，scene_revision=10，货架 [1,2]
实际 SKU 示例：可乐 / slot 1-0-3-15 -> SHELF_D，安全检查通过
```
