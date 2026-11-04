# 抓取迁移外部依赖

## 所有权

- `openflex_grasp`：核心感知、检测缓存、GraspNet 与 Pick 适配，不读取数据库。
- `openflex_grasp_supermarket_scene_adapter`：唯一依赖 `supermarket_scene_interfaces` 的抓取包。
- `/home/openflex/old/supermarket_scene`：外部场景/OWLv2/DINO 服务，不复制进核心抓取包。

部署时通过 `dependencies.repos` 或等价部署清单固定外部组件。`file://` 地址是本机部署
定位，不是核心代码运行契约；运行时只依赖 ROS `/find_sku` 服务与接口包。

数据库路径和 `SUPERMARKET_SCENE_ROOT` 不允许进入 `openflex_grasp` 核心代码。
