# 真机联合运行前置检查

当前 `ExecuteTransport` 已经提供单一任务 API，但“接口存在”不等于底层已经可以真机执行。根据当前工作区配置，以下问题必须先解决。

外部 `supermarket_scene/shelf_inventory.db` 现在可作为只读 SKU/货架槽位
数据源。启用 `competition.inventory` 前，必须把每个 `shelf_id:face`
映射到经过碰撞掩膜验证的站点；数据库的层号只提供商品名义高度，不能代替
升降 `lift_joint` 的回零和实测标定。

## 已确认的配置冲突

1. `swerve_navigation/full_system.launch.py` 会启动底盘专用 `robot_state_publisher` 和 `controller_manager`；`openarmx_integrated_bringup` 也会启动全身版本。两者不能原样同时运行。
2. 集成 bringup 的 `swerve_drive_controller.enable_odom_tf` 当前为 `true`，导航的 `safe_odom_mux` 也发布 `odom→base_link`。联合导航时前者必须改为 `false`。
3. `openarmx_integrated_moveit_config/demo.launch.py` 会再次启动 `robot_state_publisher`。与集成 bringup 联合时只能保留 bringup 中的一个。
4. MoveIt 的执行映射需要 `left/right_joint_trajectory_controller`、`lift_joint_controller` 和两个 `GripperCommand` 控制器；当前集成 bringup 实际加载的是 `left/right_forward_position_controller`、`lift_manual_position_controller` 等控制器。控制器名称和类型不匹配，不能据此宣称 MoveIt 真机执行已打通。
5. MoveIt SRDF 当前用 `world→base_link` fixed virtual joint；导航使用 `map→odom→base_link`。静态仿真可以固定底盘，导航联合运行需要单独验证移动底盘的规划场景坐标策略，且禁止再发布静态 `world/map→base_link`。

## 首次真机测试顺序

1. 只启动集成 bringup，确认唯一 `controller_manager`、唯一 `robot_state_publisher` 和全身 `/joint_states`；
2. 以“导航组件模式”启动 LIO/ICP/Nav2/安全门，不再加载底盘硬件；
3. 确认唯一 `odom→base_link` 发布者与 `/cmd_vel → collision_monitor → /cmd_vel_safe_in → safety_gate → /cmd_vel_safe`；
4. 只启动 MoveIt `move_group`，不启动第二个机器人描述发布者；
5. 先做当前状态、TF、IK 和只规划检查；
6. 对齐 MoveIt 与实际 ros2_control 执行控制器后，单臂低速执行安全姿态；
7. 最后启动 `competition_task_api.launch.py`，发送任务前检查 Nav2、对应精停、退避、
   检测、真实 Pick 和 Place Action Server；模拟抓取接口只能用于底盘衔接联调。

8. 对每个使用的货架层，在 `stations.yaml` 中加入实测的
   `level_lift_positions`（`lift_joint` 坐标），并确认 `/pick_object` 将
   `max_dof=8` 交给 `left/right_arm_with_lift` 或
   `right_arm_with_lift`；未完成标定时不得把数据库 `world_z` 直接当作电机目标。

执行器现在会在底盘移动前完成 Nav2、对应精停、所需退避、感知、Pick 和 Place
Action 的预检；任一必需服务器缺失时任务会原地失败，而不是导航到货架后才发现
MoveIt 不可用。只有站点 YAML 显式启用的近距离站点才允许自动退避。
