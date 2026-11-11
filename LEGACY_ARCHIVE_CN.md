# 旧研究路线归档说明

清理日期：2026-11-10。按“旧研究材料归档、废弃入口删除、执行底座保留、文档统一新版”处理。
本次不恢复用户此前删除的根目录旧文档，也不修改机器人模型、执行算法或碰撞门控。

## 归档位置

```text
/home/wgb/openflex_shop_legacy_archive/2026-11-10_cleanup_171853/
```

该目录位于项目外，没有纳入 Git；其中包括原来未跟踪的实验数据和脚本，不能仅依靠 Git 恢复。

- `payload/`：从当前项目移出的旧材料，保持原相对路径，可直接浏览。
- `retired-paths.txt`：移出项目的 20 个文件／目录入口清单。
- `payload.sha256`：可读归档内逐文件校验和。
- `pre-cleanup.tar`：清理前快照，包括归档内容及本次修改文档／包元数据的原件。
- `pre-cleanup.tar.sha256`、`snapshot.sha256`：快照及原始文件校验和。
- `git-status-before.txt`、`worktree-before.patch`、`git-head.txt`：清理前已有改动和版本记录。
- `experiments-generated-before.tar` 及其校验和：更新包之前的 experiments 构建／安装产物备份。
- `generated-retired/`、`generated-retired-paths.txt`：单独移出的 4 个旧安装命令、
  2 个失效学习依赖链接及 10 个旧模块／测试缓存；其余包的构建产物不清理。

归档位置是本机保存路径，不是可移植依赖。迁移项目时如需历史证据，应另外复制整个归档目录。

## 移出范围

- `recovery_results/`：恢复研究、开发冒烟、冻结预实验、失败诊断和安全分析原始材料。
- `openflex_ws/migration_records/2026-11-04/`、`2026-11-05/`：旧 G0–G10 论文阶段记录。
- bringup 中旧 ACT 集成、工程实验流程和 7 月实现状态三份文档；ROS 接口契约保留。
- G6 分析／进度脚本、G8 批量验证、恢复研究总入口和重复的可视化固定流程入口。
- experiments 中旧恢复 benchmark、协议生成、训练、报告模块及其专属测试和学习依赖清单；
  对应四个命令注册已移除，不再作为当前项目能力提供。

历史日志内的绝对路径、源码哈希和配置不改写；代码注释中的旧证据路径也按 `payload/`
根目录解析，不能把归档中的旧成功记录当成当前版本成功。

## 明确保留

- MuJoCo、相机、TC-WAM、ROS 消息、抓取／放置、控制权仲裁和碰撞／诊断代码。
- 当前 RoboOS、RoboBrain2.5、只读 MCP 脚本、依赖环境与模型运行记录。
- `run_thesis_moveit_mission.sh` 作为唯一保留的固定取送脚本入口；可视化使用该入口的
  `enable_viewer:=true` 参数及适当渲染环境，不再维护第二份相同流水线脚本。
- `run_thesis_offline.sh` 作为无 MoveIt 的底座开发入口，不作为大模型或真实物理验收入口。
- 通用日志、统计、只规划工具及物理预检。预检依赖的旧 ACT/AnyGrasp 配置和硬件门控暂留，
  只为保持失败关闭和接口兼容，不是当前毕业设计实验要求。
- 8 月模型／硬件／感知迁移记录、比赛工程、模型、地图、驱动及构建产物，不做扩大清理。

未解决的安全问题已迁入 `SAFETY_AND_TODO_CN.md`，归档不代表问题已关闭。

## 校验与恢复

```bash
cd /home/wgb/openflex_shop_legacy_archive/2026-11-10_cleanup_171853/payload
sha256sum --check ../payload.sha256
cd ..
sha256sum --check pre-cleanup.tar.sha256
tar -tf pre-cleanup.tar
```

需要查看已移出内容时直接打开 `payload/`。需要恢复修改前文件时，先把快照解压到新建临时目录，
对比后仅复制需要的路径；**不要直接全量解压覆盖工作区**，以免覆盖后续改动。
已移除的实验模块若需重新运行，还必须单独还原其依赖、入口和当时环境，不能仅复制某个脚本。
