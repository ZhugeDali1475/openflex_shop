# D0 左右腕手眼标定执行记录

## P1-01 标定资产

2026-08-12 执行：

```bash
cd /home/openflex/old/headeye
sha256sum -c SHA256SUMS
```

四个原始文件全部校验成功。归档记录为 eye-in-hand、OpenCV/Park、每侧 25 组样本；左腕 D405 `260522277324`，右腕 D405 `260522279851`。

## P1-02 修改前离线 URDF/TF 证据

修改前源文件已有用户未提交改动，SHA256：

```text
f8c6255428d5690de86951f473fcab9eca3ff119f3705b4dfee82e5368c0f496
```

在 source 现有 `install/setup.bash` 后，以 `right_poe_calibrated:=false` 展开成功：

```text
展开 URDF SHA256：4727408019770eb241ff8f745021739aba39b4f0c632d494567dae8411cdff9c
check_urdf：通过
parent：openarmx_right_link7
child：right_camera_link
旧 xyz：0.105147343276 -0.00516820715065 0.0962321917281
旧 rpy：0.227118058834 -0.981436208247 2.89256923148
```

`check_urdf` 树确认 `right_camera_link` 是 `openarmx_right_link7` 的直接子节点。采集时 ROS 业务 graph 未运行，因此没有修改前 live RealSense 内部 TF/完整 `/tf` 可保存；该限制已经记录在 G0 ROS 基线。P1-04 必须在实际相机驱动启动后补齐 live 唯一发布者证据。

## P1-03 右腕 nominal 固定关节

仅修改 `right_poe_calibrated=false` 分支：

```text
xyz：0.0830149457953 -0.0219906437615 0.0768390078403
rpy：-0.181214943034 -0.999759811745 -2.97809648406
```

`right_poe_calibrated=true` 分支及其现有数值保持不变。

修改后分别展开两个分支并执行 `check_urdf`：

```text
nominal false URDF SHA256：9aac00f95dda24827983d45500a1ca91595333d7e69fd1e5853577bf349d3e4c
poe true URDF SHA256：723da6192af5bb5bb4adc7b625afe75b006a1a787a8ae2ce539216635a579db4
nominal check_urdf：通过
poe check_urdf：通过
```

## P1-04 右腕内部 TF 与唯一父链

仅启动 D405 `260522279851` 的 RealSense driver 和 nominal URDF 的 `robot_state_publisher`，没有启动控制器或机械臂运动。RealSense v4.58.2、librealsense 2.58.2 正确选择该 serial，彩色和对齐深度均为 848x480x30。

完整内部 TF：

```text
right_camera_link -> right_camera_color_frame
  xyz = [-9.973759915737901e-06, -8.272625564131886e-06, 9.673832209955435e-06]
  xyzw = [-0.0002850017335731536, 0.00013209530152380466,
          0.0014335180167108774, 0.999998927116394]

right_camera_color_frame -> right_camera_color_optical_frame
  xyz = [0, 0, 0]
  xyzw = [-0.5, 0.4999999999999999, -0.5, 0.5000000000000001]
```

将新 URDF mount 与上述内部 TF 相乘，得到：

```text
translation = [0.0830283606837211, -0.0219819506061968, 0.0768365654910764]
quaternion xyzw = [0.161113138330625, 0.231481730776392,
                   -0.672448748866311, -0.684303620567768]
```

四元数整体取反后与标定文件一致。重组误差：

```text
translation error = 4.3361078744218e-14 m
rotation error = 0 rad
```

使用 transient-local 订阅同时收集两个 `/tf_static` publisher，共得到 35 个不同 child，没有重复 child：

```text
openarmx_right_link7 -> right_camera_link                  robot_state_publisher
right_camera_link -> right_camera_color_frame              RealSense
right_camera_color_frame -> right_camera_color_optical_frame RealSense
```

`tf2_echo openarmx_right_link7 right_camera_color_optical_frame` 返回的位姿与标定文件一致。验证后已正常停止两个临时节点。

## 左腕当前阻塞

`rs-enumerate-devices -s` 只发现右腕标定 D405 `260522279851`；左腕标定 D405 `260522277324` 未连接。不得使用右腕内部 TF 计算左腕 `link7 -> camera_link`。
