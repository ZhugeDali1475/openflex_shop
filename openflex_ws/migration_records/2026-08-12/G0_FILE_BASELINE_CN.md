# G0 文件与包基线

采集时间：2026-08-12 09:32-09:38 CST。

## 目标 workspace

```text
路径：/home/openflex/old/openflex_ws
Git 根：/home/openflex/old/openflex_ws
分支：main
提交：无，当前分支尚无任何提交
Git porcelain 条目数（含所有未跟踪文件）：33804
src 文件数：2493
ROS package.xml 数：62
src 大小：2.6G
排除 build/install/log/stale build 后的内容摘要：
1da08366201ae7f79875eda6e50708b8cb94efdb034f2973bb077c4789d9ce8a
```

工作空间在迁移开始前已经包含大量未跟踪源码、`build/`、`install/`、`log/` 和旧机器残留目录。它们均视为用户现有内容；后续不使用 Git checkout/reset 恢复，也不把这些内容认作本次迁移产生。

目标内与本迁移直接相关的初始 ROS 包：

```text
openarmx_integrated_bringup
openarmx_integrated_description
openarmx_integrated_moveit_config
openarmx_integrated_moveit_msgs
openarmx_supermarket_sim
openarmx_task_executor
openarmx_task_msgs
openflex_gui
openflex_navigation_gui
openflex_anygrasp
openflex_tc_wam
openflex_thesis_bringup
openflex_thesis_control
openflex_thesis_experiments
openflex_thesis_msgs
openflex_thesis_mujoco
```

完整包清单可用以下只读命令重建：

```bash
find /home/openflex/old/openflex_ws/src -name package.xml -print | sort
```

## 迁移源

```text
路径：/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp
Git：该目录及父目录不是 Git 仓库
文件数（排除 __pycache__）：173
ROS package.xml 数：3
目录大小：24M
内容摘要（排除 __pycache__）：
1cfde9be5e798f6bbddb7f7e86cd2706bf48d0d80b582cebd91b19b99fc6f7ff
```

三个源 ROS 包为 `openflex_grasp`、`openflex_grasp_moveit`、`openflex_grasp_msgs`；`graspnet-baseline` 通过 `COLCON_IGNORE` 作为随附上游源码，不是第四个 ROS 包。

## 磁盘

根文件系统 `/dev/nvme0n1p2` 总计 916G，已用 116G，可用 754G，使用率 14%。模型解压、两个 venv 和重新构建具有充足空间。
