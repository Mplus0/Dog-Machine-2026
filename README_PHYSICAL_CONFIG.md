# 机器狗物理配置记录

本文记录机器狗本体、主机、传感器和安装姿态等物理配置。网络地址和账号信息另见本地文件：

```text
README_NETWORK_TOPOLOGY.md
```

## 1. 主机与系统

### 感知主机

```text
角色：运行 ROS Noetic、D435i、导航、识别、rosbag 和上位任务逻辑
系统：Ubuntu 20.04
ROS：Noetic
工程路径：~/comp2026_ws
开发同步方式：开发机与机器狗通过 git 拉取同步
管理 WiFi 网卡：wlan0，USB ID 368b:8d85，具体型号未确认，driver=usb，连接 Bad_Puppy
管理 WiFi 当前地址：192.168.31.174/24，由 DHCP 分配，负责 SSH、Git、Internet 和默认路由
机器狗 AP 网卡：wlan1，USB ID 368b:8d85，具体型号未确认，driver=usb
机器狗 AP 链路：wlan1 连接 `YSC-JYML-dt3tfa-5G`，当前地址 192.168.2.214/24，仅访问 192.168.2.0/24
机器狗 AP 路由约束：ipv4.never-default=yes，ipv4.route-metric=700，ipv6.never-default=yes
内部有线网：eth0=192.168.1.103/24，连接运动主机 192.168.1.120/24
```

### 运动主机

```text
角色：闭源运动控制封装、底层运动接口、厂家广角相机/掌机相关链路
系统：Ubuntu 20.04.5 LTS
内核：Linux 5.10.160 aarch64
硬件：RK3588 板卡
底层控制：QNX 运动控制系统
维护策略：默认只做只读观察和被动抓包；除非万不得已，不在运动主机上提权或修改系统配置
```

## 2. 传感器与安装

### Intel RealSense D435i

```text
用途：导航深度图、depthimage_to_laserscan、识别彩色图像、离线 rosbag 复盘
安装状态：原装支架
标定姿态：机器狗正常静止站立
相机主光轴相对水平地面 pitch 偏角：-18.82 deg
相机距离地面垂直高度：41.5 cm
```

D435i 官方/资料标称 FOV：

```text
资料来源：
  - Intel ARK / D435i 官方规格页
  - Intel RealSense D435i 产品页
  - Intel RealSense D400 Series datasheet

产品页/规格页常用标称：
  Depth FOV: H 87 deg, V 58 deg
  RGB FOV:   H 69 deg, V 42 deg

D400 datasheet 中 D435/D435i 模组视场参数：
  Depth module left/right imager FOV: H 91.2 deg, V 65.5 deg, D 100.6 deg
  RGB sensor FOV:                    H 69.4 deg, V 42.5 deg, D 77.0 deg

说明：
  - depth FOV 通常大于 color FOV。
  - 产品页的 H87/V58 更适合做 D435i 整机宣传标称。
  - datasheet 的 H91.2/V65.5/D100.6 更接近 D435/D435i 深度模组传感器级参数。
  - 实际 ROS CameraInfo 计算出的等效 FOV 会随分辨率、裁剪、对齐和 launch profile 变化。
  - 当前导航、AMCL 和 depth-to-scan 链路按 3.0m 作为 D435i 深度有效使用上限。
```

当前 ROS profile 实测内参与等效 FOV：

```text
测量工具：src/tools/measure_d435i_fov.py
测量来源：ROS CameraInfo
说明：以下数值只对应测量时正在运行的相机 profile。后续若修改分辨率、裁剪、对齐或相机 launch 参数，应重新测量并更新。

RGB / Color:
  topic: /camera/color/camera_info
  resolution: 1280x720
  fx: 907.272400
  fy: 907.151672
  cx: 642.610779
  cy: 376.501648
  hfov_deg: 70.399133
  vfov_deg: 43.290963
  dfov_deg: 59.568128

Depth:
  topic: /camera/depth/camera_info
  resolution: 640x480
  fx: 388.389587
  fy: 388.389587
  cx: 325.711243
  cy: 239.344116
  hfov_deg: 78.971242
  vfov_deg: 63.426843
  dfov_deg: 72.127572
```

RGB / Depth 对齐说明：

```text
当前 color 与 depth 的分辨率、内参和 FOV 不同，这是 D435i 正常现象，不代表相机故障。
导航链路继续使用原始 depth：
  /camera/depth/image_rect_raw
  /camera/depth/camera_info
原因是 depth FOV 更大，适合 depthimage_to_laserscan 和 AMCL。

如果后续需要把 RGB 检测框对应到深度距离，有两种做法：
  1. 启动相机时传入 camera_align_depth:=true，使用 RealSense 发布的 aligned depth topic。
  2. 保持原始 color/depth topic，使用相机外参把 color 像素投影到 depth 坐标系。

当前仪表识别只使用 RGB 图像，不依赖对应像素深度，因此无需强制对齐。
```

倾角来源：

```text
工具：src/tools/calibrate_d435i_ground_tilt.py
结果：src/tools/d435i_ground_tilt_calibration.yaml
说明：第一次趴下姿态数据不参与；后续正常站立数据用于估计原装支架下的 pitch 偏角。
```

视场角/内参测量：

```text
工具：src/tools/measure_d435i_fov.py
说明：现场通过 ROS CameraInfo 读取当前 RGB/Depth 的 width、height、fx、fy、cx、cy，并计算等效水平/垂直 FOV。机器狗上 pyrealsense2 不可用，默认使用 ROS 后端。
```

当前 TF 补偿记录：

```text
默认 TF：src/allmovebase/config/camera2base_tf.yaml
倾斜支架 TF：src/allmovebase/config/camera2base_tf_tilted.yaml
未来打印支架 TF：src/allmovebase/config/camera2base_tf_bracket.yaml
```

注意：

- 现有任务链路通过 `camera2base_tf.launch` 发布 `base_link -> camera_link`。
- 当前 pitch 补偿会影响 TF 坐标解释，但 `depthimage_to_laserscan` 仍然是从深度图生成 `/scan`，不是完整三维点云地面滤除。
- 如果更换相机支架或调整安装高度，应重新运行倾角标定并更新本文。

### 厂家广角相机

```text
用途：厂家掌机画面查看，当前暂未接入比赛主任务链路
已知链路：去年工程曾通过运动主机 RTSP 拉流观察
当前状态：是否可用于今年任务尚未确定，等待实机测试
```

## 3. 机体姿态相关备注

```text
原装 D435i 支架下，相机明显向下俯视。
正常静止站立时，实测 pitch 约 -18.82 deg。
较开阔区域标定中 roll 接近 0，因此当前工程只记录 pitch 主偏移。
```

实机验证重点：

- RViz 中检查 `/scan` 是否主要对应前方障碍，而不是地面噪点。
- 检查 local costmap 障碍物投影是否贴合实际位置。
- 检查 AMCL 是否因为 `/scan` 质量变化出现定位漂移。
- 上机 rosbag 应记录深度图、RGB、`/scan`、TF、AMCL、move_base 状态，便于离线复盘。

## 4. 维护约定

- 更换相机支架、调整 D435i 高度、改动机身姿态或传感器安装位置后，应更新本文。
- 本文只记录物理配置和非敏感硬件信息；账号、密码、临时网络细节放在本地忽略文件中。
- 如果后续机械臂、额外相机或新传感器接入主链路，应在本文新增对应章节。
