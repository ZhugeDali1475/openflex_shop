# 当前论文研究材料

## 研究主线与作者贡献

作者自研OpenFlex轮臂机器人为系统载体，两项核心方法为：

- 任务条件化站位与操作可行性协同。
- 分层自由度释放的全身求解。

按作者明确说明，这两项方法归属于作者自主研究成果；文献对照界定差异，实验评价效果与代价。
底盘观测表征不再作为论文主线，不要求ACT/π₀.₅训练消融。
已有实现与后续增强分列，不把作者归属等同于全球首次、全部增强已实现或性能已优于基线。

## 当前文档

- `商超轮臂机器人_研究主线与算法实验方案_CN.md`：工作题目、六项要求、行业对照、两项方法、公式、实验设计与局限。
- `商超研究_来源核验与实验待办_CN.md`：来源阅读层级、实现边界及阶段验收。
- `项目优化与提速分析_CN.md`：独立保留的耗时、负结果、UNKNOWN问题与优化分析，不是被替代论文。

当前材料是研究方案，不是已完成全部实验的最终学位论文。旧工程初稿、学术重写稿、
中期衔接稿、旧答辩稿及其导出/调试副本已按作者要求删除，不再提供旧版入口。

## Word文件

桌面目录：`/home/wgb/桌面/毕业设计/OpenFlex商超研究方案_20261112/`。
当前文件为`OpenFlex商超研究主线与算法实验方案_原创归属修订_20261112.doc`，
另附DOCX、PDF、两份源稿和校验清单。`.doc`为真Word97–2003文件，不是改扩展名。
当前本地构建位于`local_dependencies/openflex_thesis/retail_problem_reframe_20261112/current_release/`。

导出时输出目录必须尚不存在，避免覆盖作者修改：

```bash
python3 thesis/build_research_review.py --profile retail --audit local_dependencies/openflex_thesis/research_review_20261112 --source thesis/商超轮臂机器人_研究主线与算法实验方案_CN.md --source thesis/商超研究_来源核验与实验待办_CN.md --output local_dependencies/openflex_thesis/retail_problem_reframe_20261112/current_release
```

导出器要求显式指定源稿，不再默认读取已删除的旧论文。`build_thesis.py`、`build_defense.py`
保留为格式工具，同样要求显式指定源稿；旧PPT不再代表当前研究路线。

## 保留的证据与工具

- `/home/wgb/中期/`：作者原始中期资料，未删除。
- `local_dependencies/openflex_thesis/research_review_20261112/`：只读资产审计、静态预览、中期提取资料及研究文献原文。
- `local_dependencies/openflex_thesis/retail_problem_reframe_20261112/sources/`：官方网页和近似工作原文快照、检索状态与哈希。
- `local_dependencies/openflex_thesis/performance/`及`local_dependencies/openflex_roboos/runs/`：性能、停止审计、冻结实验与视频，未清理。
- 离线反例、原始文献元数据、源码核验与优化报告保留，不把它们算作新增运动实验。

历史冻结工程证据包不重写；包内旧文稿只属于当时封存状态，不是当前文稿入口。
独立删除清单存于`local_dependencies/openflex_thesis/document_cleanup_*/deletion_manifest.json`，
记录路径、大小及哈希，不另打包保存已删除的旧文稿。

## 验证与安全边界

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q thesis scripts/tests/test_openflex_research_assets.py
```

导出与测试不访问模型、不调用ROS、不执行运动。当前UNKNOWN终态与停止未确认问题仍按
`SAFETY_AND_TODO_CN.md`处理；删除文档不解除互斥、不改变现场，也不授权新动态任务。
