# 多商品商超场景：离线构建与验收边界

更新：2026-11-12。本次完成的是独立场景基础，不是多商品运动闭环验收。
运行现场的UNKNOWN请求未关闭；不得据此重启、清锁、松爪或替换原场景。

## 已实现

- 复用自研OpenFlex机器人及原始`supermarket_scene/mujoco/supermarket_scene_6x8_competition.xml`，
  保留四组双面多层货架、四个交付台、篮筐、围墙、地板等静态设施及其原有碰撞设置。
- 本阶段显式选择YCB盒装饼干、芥末瓶、番茄汤罐三类，每类在一号货架正X侧第1、2层各放一件，
  共六实例。没有复制原场景全部102件商品，也没有将中文库存SKU擅自映射成这些资产。
- `manifest.json`逐件记录仿真SKU、实例、货位、支撑几何、初始位姿、网格包围盒及资源哈希。
  `inventory_sku=null`，数据库不读取、不修改；仿真货位不冒充数据库货位。
- OBJ坐标原点并不位于网格中心；按真实顶点范围修正XY及Z，网格最低点高于支撑顶面1毫米。
  视觉网格与碰撞网格分离；当前候选碰撞体是由OBJ实测包围盒生成的保守轴对齐盒，仍未完成实物标定。
- 可视网格密度为零，独立碰撞网格承载显式假设质量，避免同一商品重复计入质量。
  三类假设质量分别为0.25、0.35、0.20千克，非实测值，摩擦系数也非实测。
- 单商品默认构建函数接口及场景不变，仅提取共用机器人准备函数；商超资产加独立名称前缀，
  避免与机器人已有地板材质同名。原地板替换为商超地板，而非叠加两个接触地板。
- 商超源文件使用degree，机器人使用radian；导入姿态角显式转换，摄像机fovy仍保留度数。
  只支持已审核的静态设施和轴对齐支撑，不支持的旋转支撑、继承类或接触覆盖直接拒绝。

## 可复现入口

在仓库根目录运行，输出目录必须尚不存在；默认使用保守盒碰撞，可显式切回高细节网格基线。
构建不会连接ROS、发送Action或调用`mj_step`：

```bash
python3 scripts/build_openflex_retail_scene.py --output local_dependencies/openflex_thesis/retail_fixture_new --render
python3 scripts/build_openflex_retail_scene.py --collision-mode mesh --output local_dependencies/openflex_thesis/retail_mesh_baseline_new
```

实现位于`openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/retail_scene_builder.py`。
工具只生成新目录；已有目录一律拒绝覆盖，包括用户修改后的结果。

本次主要产物：`local_dependencies/openflex_thesis/retail_fixture_20261112_v4/`。

|文件|用途|
|---|---|
|`scene.xml`|可独立编译的机器人与商超MJCF，未替换现场模型|
|`manifest.json`|逐商品、货位、支撑、资源与交付台映射及假设|
|`static_audit.json`|编译、初态接触、质量、位姿及可见像素记录|
|`overview.png`、`shelf.png`|全场及商品货架静态预览|
|`head.png`|上游初始构型下的头部相机图像|
|`head_observation.png`|另一独立静态观察构型下的头部图像|
|`*_segmentation.npy`|MuJoCo渲染器实例分割真值，不是模型输出|
|`../retail_moveit_products_20261112_v1/planning_scene_products.json`|六个可移动商品碰撞盒候选导出；坐标未配准，不可供运行时消费|

`retail_fixture_20261112_v1/`是首次静态生成记录，未包含新增观察构型；不覆盖该记录。

## 多商品生命周期参考契约（离线）

`openflex_roboos_bridge/retail_object_lifecycle.py`新增了一个纯Python参考契约，用于在接入
ROS／MoveIt之前验证多商品状态不会被逻辑层丢失。它从明确标记为
`runtime_consumable=false`、`execution_enabled=false`且未注册到实时地图的导出清单加载商品，
并约束以下状态转移：`world → approach → transport/attached → placed → world`。

- 选择目标时保留其他商品在`world`集合中，使其仍可作为规划障碍物；选中的目标必须有唯一实例ID。
- 附着前必须由上层传入完整抓取预检通过标志；释放前必须由上层传入落稳和撤离验证通过标志。
- 当前契约只允许一个目标处于附着态；附着投影会移除该目标的世界障碍，但保留其他商品。
- 放置后保留`placed`历史，下一件商品只能在显式完成上一件交付后选择。
- 该模块只检查状态集合和调用方传入的布尔门，不读取传感器、不核验证据内容、不调用ROS／MoveIt、
  不更新PlanningScene，也不证明碰撞、轨迹、抓取或释放安全；不能作为运行时执行器使用。

因此，生命周期单测通过只表示离线状态机契约成立；真正接入还必须完成坐标配准、目标移出世界障碍、
附着物同步、放置后重新加入世界、结果回读和独立安全证据验证。
离线投影不生成TCP相对位姿；没有运行时实测抓取结果时，附着商品的位姿保持为空，避免把货架初始位姿
误当成持物位姿。

## 实测结果与解释

MuJoCo 3.4.0成功编译；模型173个body、248个geom，`nq=74`、`nv=68`。
仅调用`mj_forward`，动力学步数为0。六实例的质量、初始位姿和碰撞开关检查均通过。
初始存在四条车轮与地板接触，约0.3微米穿入；未发现超过1微米的穿透，商品初始无接触。
这一数值阈值用于描述静态报告，不是运动安全阈值调整，也不是连续无碰撞证明。

上游初始构型下，头部相机朝向与商品货架不匹配，六件商品均为0像素。
在另一份独立`MjData`中设置底盘`x=0.15m`、`y=1.18m`、`yaw=π`，
头部俯仰`-0.50rad`，保持其他关节为上游初值；这只是离线构型取样，未执行导航或瞬移现场。
该快照未发现超过1微米的穿透。960×720图像中的商品可见像素如下：

|仿真SKU|第1层|第2层|
|---|---:|---:|
|cracker_box|9596|16106|
|mustard_bottle|2433|4110|
|tomato_soup_can|1971|3044|

分割真值只能证明当前渲染视图可见；不能声称RoboBrain识别正确，不能证明该观察构型可安全到达，
也不意味着该构型就是最佳操作站位。交付台只映射支撑面和边界，篮筐等遮挡仍需验证。

## 验证命令

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q scripts/tests/test_openflex_retail_scene.py openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/test/test_model_builder.py
```

本次该命令11项通过，涵盖源文件和数据库哈希不变、实例唯一性、支撑范围、绝对资源路径、
碰撞开启、质量不重复、角度单位转换、拒绝覆盖、非法几何拒绝、MuJoCo编译及静态相机分割。
这些是离线软件与静态几何检查，不是11次取送实验。

## 下一步与未完成项

1. 继续解决S0原请求终态证据缺口，未经确认不恢复原运行现场。
2. 独立验证商品动力学落稳、惯性和摩擦假设、全机器人安全停车与场景接触；不放宽现有安全门。
3. 为六实例建立多目标观测/桥接适配与MoveIt碰撞场景同步，目前现场接口仍是单商品契约。
4. 将实际头部图像送入模型，分别评估类别、实例、货位及反馈纠错；真值与模型输出分开记录。
5. 围绕作者原创的任务条件化站位与操作可行性协同，保持手臂/抓姿/升降条件身份，
   加入接近、撤离及预算内轨迹验证；不能用静态可见性代替操作可行性。
6. 围绕作者原创的分层自由度释放，按同预算比较手臂、手臂加升降、全身参与的求解与运动代价。
7. 安全前提满足后，完成多商品条件下逐件完整取送和配对耗时实验，再更新论文结果。

## 动力学审计

静态场景之后又进行了独立副本动力学检查。审计通过`mj_step`推进副本数据，不读取ROS、
不连接控制器、不修改场景文件，也不代表现场机器人运动。入口为：

```bash
python3 scripts/audit_openflex_retail_dynamics.py \
  --scene local_dependencies/openflex_thesis/retail_fixture_20261112_v4/scene.xml \
  --duration 10 \
  --output local_dependencies/openflex_thesis/retail_dynamics_audit_new
```

审计记录每件商品的位移、线/角速度、接触距离和最近窗口样本，并保留机器人初始关节漂移。
当前候选使用保守盒碰撞体，在独立副本中改善了商品接触稳定性；这不是为了掩盖问题而关闭碰撞，
而是显式改变碰撞表示并保留视觉网格。质量、摩擦、接触求解器和盒碰撞近似仍未做实物标定。

对照结果如下：

|版本|碰撞表示|10秒筛查通过|最大穿入|最大位移|结论|
|---|---|---:|---:|---:|---|
|`retail_fixture_20261112_v2`|视觉OBJ同网格碰撞|0/6|约1.46 mm|约0.126 m|接触/物性假设不适合直接评估|
|`retail_fixture_20261112_v4`|OBJ包围盒保守轴对齐盒|6/6|约0.86 mm|约0.001 m|候选改善，仍非整机验收|

两版均使用相同商品、货位、质量假设和货架，差异主要是碰撞表示。当前生成器通过
`--collision-mode mesh`或`--collision-mode box`复现两种MJCF。筛查指标为线速度不高于
0.01m/s、角速度不高于0.05rad/s、接触穿入不超过1mm；它们是诊断指标，不是机器人安全标准。
候选审计在商品—货架子问题中关闭机器人几何碰撞，并将位置执行器目标设为源模型初始值，
因此不能推出全机器人停车、手臂避障或抓取安全。候选记录位于
`local_dependencies/openflex_thesis/retail_dynamics_audit_20261112_v6/dynamic_audit.json`；
可复现的显式网格基线位于
`local_dependencies/openflex_thesis/retail_dynamics_mesh_baseline_20261112_v1/dynamic_audit.json`。
完整机器人碰撞、接近/撤离轨迹和真实商品物性仍需单独验证。

本次没有新完整流程耗时、抓取成功率或加速结论；历史三轮时间及其小样本限制保持不变。

### MoveIt映射进展

已新增`export_openflex_retail_moveit_objects.py`，从MuJoCo编译态读取六个盒碰撞体的中心、尺寸和方向，
生成带场景哈希、SKU/实例/货位和单位四元数的独立JSON。导出通过离线测试，但未调用MoveIt服务。
导出坐标明确标记为`mujoco_sim_world`，未注册到实时`map`；文件标记
`runtime_consumable=false`，不是当前`sim_shelf_scene.yaml`的可加载替代物。

源码核对发现，当前`scene_registry.py`只读取`thesis_object`和一个`object_id`，
MCP技能也把商品ID限制为`thesis_item_01`；执行抓取/放置还依赖该单商品的自由关节、几何和状态字段。
因此还必须设计多商品观察、目标移出世界障碍/附着/放置后重新加入世界的生命周期，以及按目标ID的结果回读；
不能简单把六件商品塞进货架障碍列表，否则目标本身会被规划器当成不可触碰障碍。
