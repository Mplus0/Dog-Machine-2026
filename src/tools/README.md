# 脚本工具说明

本目录放机器狗侧的轻量辅助工具。原则是：不接入主任务状态机，不长期运行；用于上机前检查、实机调参、录点和离线复盘。

在 Jetson 上使用：

```bash
cd ~/comp2026_ws/src/tools
```

当前网络结构记录：

```text
开发路由器/WiFi：Bad_Puppy，网关 192.168.31.1
感知主机 wlan0：连接 Bad_Puppy，当前 DHCP 地址 192.168.31.175/24，默认路由
感知主机 wlan1：连接机器狗 AP，192.168.2.213/24，ipv4.never-default=yes
机器狗对外 WiFi/AP 名：YSC-JYML-dt3tfa-5G
机器狗内部感知主机 IP：192.168.1.103
机器狗内部运动主机 IP：192.168.1.120
运动主机热点网段别名：192.168.137.120
运动主机 p2p0/AP 地址：192.168.2.1
掌机 IP：192.168.2.65
感知主机/运动主机 SSH 用户名：ysc
感知主机/运动主机 SSH 密码：设备出厂默认密码，本文不记录明文
```

本目录工具默认假设在感知主机上运行。浏览器访问网页工具时，开发机使用 `http://192.168.31.175:<port>`；该地址是当前 DHCP 租约，变化时先查询 `wlan0`。感知主机访问运动主机时使用 `192.168.1.120`。

私有连接信息使用本地配置文件：

```text
private_robot_access.example.yaml  # 仓库模板
private_robot_access.yaml          # 本地真实配置，已写入 .gitignore
```

当前工作区内的 `private_robot_access.yaml` 已整理为可复制到感知主机的最小运行配置，密码为空。该文件被 Git 忽略，因此必须通过 `scp`、U 盘等方式单独复制，不能依赖 `git pull`。

如果只通过 Git 同步仓库，则在感知主机上从模板生成运行配置：

```bash
cd ~/comp2026_ws/src/tools
cp private_robot_access.example.yaml private_robot_access.yaml
chmod 600 private_robot_access.yaml
```

配置沿用原文件字段，只修改当前网络中已经确认的值：

```yaml
robot_hotspot_ip: 192.168.31.175
developer_wifi_ssid: Bad_Puppy
perception_wifi_adapter: 8188ETV
robot_wifi_ssid: YSC-JYML-dt3tfa-5G
perception_robot_ap_interface: wlan1
perception_robot_ap_ip: 192.168.2.213
perception_host: 192.168.1.103
motion_host: 192.168.1.120
motion_host_hotspot_alias: 192.168.137.120
motion_host_p2p: 192.168.2.1
perception_host_ssh_user: ysc
perception_host_ssh_password: ""
motion_host_ssh_user: ysc
motion_host_ssh_password: ""
motion_host_sudo_password: ""
sudo_password_same_as_ssh: true
sshpass: false
sudo: false
sudo_with_password: false
handheld_ip: 192.168.2.65
tablet_ip: ""
```

`motion_host_packet_capture.py` 默认读取 `private_robot_access.yaml`。默认的 `sshpass: false` 会使用普通 SSH，可交互输入密码，也可使用 SSH 密钥，不需要把密码写进文件。

只有确实需要无人值守密码登录时，才填写 `motion_host_ssh_password` 并改为 `sshpass: true`。此模式下密码通过 `SSHPASS` 环境变量传给 `sshpass -e`，不会打印在命令行里。依赖：

```bash
sudo apt install sshpass
```

如果感知主机没有自动连接 `Bad_Puppy`，但电脑能够连接机器狗 AP，可通过运动主机跳板登录：

```bash
ssh -J ysc@192.168.2.1 ysc@192.168.1.103
```

不要在没有物理控制台或其他备用入口时远程重启 NetworkManager。

## 工具总览

| 文件                                   | 用途                                                      | 是否依赖 ROS     |
| -------------------------------------- | --------------------------------------------------------- | ---------------- |
| `d435i_stream_test.py`               | 预览 D435i 彩色画面并通过 MJPEG 网页推流                  | 否               |
| `wide_angle_stream_test.py`          | 预览厂家广角相机或普通 `/dev/videoX` 相机               | 否               |
| `ros_nav_debug_stream.py`            | 浏览器查看 D435i 深度图、地图、AMCL 位姿和规划路径        | 是               |
| `preflight.py`              | 上机前只读预检 ROS、Docker、磁盘、模型文件和关键 topic   | 部分依赖 ROS     |
| `motion_host_packet_capture.py`      | 被动抓取运动主机/掌机/感知主机网络包并做轻量离线解码      | 否               |
| `collect_robot_env_snapshot.sh`      | 只读采集 Jetson/ROS/CUDA/RealSense/Python/底层库环境快照 | 否               |
| `calibrate_d435i_ground_tilt.py`     | 用深度图拟合地面，估计 D435i 安装倾角                     | 默认否，可选 ROS |
| `d435i_ground_tilt_calibration.yaml` | D435i 倾角标定结果记录                                    | 否               |
| `measure_d435i_fov.py`               | 读取当前 D435i RGB/Depth 内参并计算等效 FOV               | 默认否，可选 ROS |
| `calibrate_d435i_profiles.py`        | 批量记录 D435i 多分辨率 profile 的内参、FOV 和外参线索    | 默认否，可选 ROS |
| `record_task_pose.py`           | 从 `/amcl_pose` 记录任务位姿到 `task_poses.yaml` | 是               |
| `record_rosbag.py`          | 按比赛调试 profile 录制 rosbag，并生成 manifest         | 是               |
| `extract_rosbag_frames.py`           | 从 rosbag 抽取彩色/深度图像，生成时间戳匹配文件           | 是               |

## 1. D435i 网页预览

用于快速检查 D435i 彩色流画面，不启动 ROS。

默认读取：

```text
/dev/realsense_rgb
```

启动：

```bash
python3 d435i_stream_test.py --port 8080
```

浏览器打开：

```text
http://<Jetson-IP>:8080
```

如果要指定普通视频设备：

```bash
python3 d435i_stream_test.py --camera /dev/video4 --port 8080
python3 d435i_stream_test.py --camera-index 4 --port 8080
python3 d435i_stream_test.py 4 --port 8080
```

常用参数：

```bash
python3 d435i_stream_test.py --stream-fps 10 --jpeg-quality 70
```

依赖：

```bash
python3 -c "import cv2"
```

如果缺失，在 Ubuntu/Jetson 上通常安装：

```bash
sudo apt install python3-opencv
```

## 2. 广角相机网页预览

去年工程中广角相机不是本机 `/dev/videoX`，而是从运动主机拉 RTSP：

```text
rtsp://192.168.1.120:8554/test
```

默认使用 GStreamer/RTSP，端口为 `8081`，避免和 D435i 预览冲突：

```bash
python3 wide_angle_stream_test.py
```

浏览器打开：

```text
http://<Jetson-IP>:8081
```

如果要明确指定 RTSP：

```bash
python3 wide_angle_stream_test.py \
  --backend gstreamer \
  --rtsp-url rtsp://192.168.1.120:8554/test \
  --port 8081
```

如果实机发现广角相机其实暴露为普通视频设备：

```bash
python3 wide_angle_stream_test.py --backend opencv --camera /dev/video0 --port 8081
python3 wide_angle_stream_test.py --backend opencv 0 --port 8081
```

常用参数：

```bash
python3 wide_angle_stream_test.py --width 1280 --height 720 --stream-fps 10 --jpeg-quality 70
```

GStreamer 后端依赖 Jetson 上的 GStreamer、NVIDIA 解码插件和 Python GI；OpenCV 后端依赖 `cv2`。

## 3. ROS 导航网页调试

用于在 NoMachine 较卡时，用浏览器轻量查看导航状态。它订阅 ROS topic，不发控制指令，不改变任务链路。

建议先启动任意包含相机、`depthimage_to_laserscan`、地图、AMCL 和 `move_base` 的任务链路，例如：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
```

另开终端启动网页看板：

```bash
cd ~/comp2026_ws/src/tools
python3 ros_nav_debug_stream.py --port 8082
```

浏览器打开：

```text
http://<Jetson-IP>:8082
```

页面包含：

- D435i 深度图，默认 topic 为 `/camera/depth/image_rect_raw`。
- 地图叠加 AMCL 位姿、粒子云、全局路径和局部路径。
- `/health` 文本状态页，用于快速看深度、地图、AMCL、`/scan`、`move_base` 是否有数据。

常用参数：

```bash
python3 ros_nav_debug_stream.py \
  --stream-fps 3 \
  --jpeg-quality 65 \
  --depth-max 3.0 \
  --map-scale 2.0
```

如果 topic 名称不同，可显式指定：

```bash
python3 ros_nav_debug_stream.py \
  --depth-topic /camera/depth/image_rect_raw \
  --map-topic /map \
  --amcl-pose-topic /amcl_pose \
  --particle-topic /particlecloud
```

依赖：ROS Noetic、`cv_bridge`、`cv2`、`numpy`。当前终端需要 source 工作空间：

```bash
source ~/comp2026_ws/devel/setup.bash
```

完整 RViz 调试仍然建议保留，尤其是要看 TF、costmap、手动发初始位姿或目标点时：

```bash
roslaunch allmovebase rviz_nav.launch
```

NoMachine 卡时优先用本网页看板；不要一开始就推整个桌面，整桌面推流通常更吃 CPU/GPU、带宽和编码延迟。

## 4. 上机前预检

用于实机测试前快速扫一遍常见风险：ROS 命令是否可用、运动主机是否能 ping 通、Docker 是否可用、模型文件是否存在、`~/bags` 剩余空间是否足够、关键 ROS topic 是否出现、关键 topic 是否有频率。

在启动任务链路后运行：

```bash
cd ~/comp2026_ws/src/tools
python3 preflight.py
```

如果只是上机前还没启动 ROS 链路，也可以先跑一遍；此时 ROS topic 相关项会失败或告警，但磁盘、Docker、模型文件、运动主机网络仍有参考价值。

常用参数：

```bash
python3 preflight.py --min-free-gb 30
python3 preflight.py --skip-hz
python3 preflight.py --topic /your_extra_topic
python3 preflight.py --json-output ~/bags/preflight.json
```

如果希望在 CI 或严格检查中遇到 FAIL 就返回非零：

```bash
python3 preflight.py --strict
```

默认检查的关键 topic 包括：

```text
/tf
/tf_static
/camera/depth/image_rect_raw
/camera/depth/camera_info
/scan
/map
/amcl_pose
/move_base/status
/cmd_vel
/leg_odom2
/lite3_motion_cmd
/meter_inspection_ready
/meter_status
```

默认频率检查包括：

```text
/camera/depth/image_rect_raw
/scan
/amcl_pose
/move_base/status
```

该脚本不发布任何控制 topic，不会让机器狗运动。

## 5. 运动主机被动抓包

用于观察感知主机、运动主机、掌机之间的网络通信。工具只封装 `tcpdump` 做被动抓包，不发送控制包，不修改运动主机配置。

当前工程已知端口：

```text
43893  感知主机 -> 运动主机，开发者 UDP 控制目标端口
43894  ros2qnx 本地绑定端口
43897  运动主机 -> 感知主机，qnx2ros 状态接收端口
43899  nx2app/掌机相关接收端口
8554   运动主机 RTSP 广角相机流
```

实测掌机链路在运动主机 `p2p0=192.168.2.1/24` 下：

```text
掌机：192.168.2.65
```

这些端口来自今年工程和厂家通信文档，不代表已经确认掌机与运动主机直连通讯端口。如果要发现未知端口，先只按运动主机 IP 抓全端口：

```bash
python3 motion_host_packet_capture.py capture --all-ports --duration 30
```

已确认掌机 IP 时，可同时限制运动主机和掌机两个 host：

```bash
python3 motion_host_packet_capture.py capture \
  --all-ports \
  --host 192.168.2.65 \
  --duration 30
```

感知主机大概率看不到 `p2p0` 上掌机与运动主机的直连流量。建议远程普通权限在运动主机 `p2p0` 上抓：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.65 \
  --all-ports \
  --duration 60 \
  --output ~/packet_captures/handheld_192_168_2_65.pcap
```

在感知主机上抓取和运动主机相关的默认端口，持续 60 秒：

```bash
python3 motion_host_packet_capture.py capture --duration 60
```

只打印命令，不执行：

```bash
python3 motion_host_packet_capture.py capture --dry-run
```

如果要在运动主机上通过 SSH 远程抓包，并把 pcap 保存到当前机器：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --duration 60 \
  --output ~/packet_captures/motion_host_remote.pcap
```

`remote-capture` 默认使用 `--ssh ysc@192.168.1.120`；如果运动主机地址或用户名不同再显式指定：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --ssh ysc@192.168.1.120 \
  --duration 60
```

远程模式默认不使用 sudo，只在运动主机普通用户权限下运行 `tcpdump`。如果没有抓包权限，优先退回感知主机侧抓包；确实万不得已再显式加 `--sudo`。

如果本地配置中设置了 `sshpass: true` 和 `motion_host_ssh_password`，远程模式会自动使用该密码登录 SSH。若万不得已需要 sudo，且 sudo 密码与 SSH 密码相同，再临时使用：

```yaml
sudo: true
sudo_password_same_as_ssh: true
sudo_with_password: true
```

工具会用 `sudo -S` 通过 SSH 标准输入传递 sudo 密码，不会把密码打印进命令行。平时不要在私有配置里长期打开 sudo。

查看 pcap 摘要：

```bash
python3 motion_host_packet_capture.py summarize --pcap ~/packet_captures/motion_host.pcap --count 50
```

按今年工程中已知结构轻量解码 UDP payload：

```bash
python3 motion_host_packet_capture.py decode --pcap ~/packet_captures/motion_host.pcap --count 100
```

解码目前只识别：

- `SimpleCMD`：12 字节，`cmd_code/cmd_value/type`
- `ComplexCMD`：20 字节，`cmd_code/cmd_value/type/data`
- `JoystickChannelFrame`：42 字节，掌机通道帧，包含左右摇杆原始值

注意：

- 该工具用于观察和建立假设，不建议直接根据未知包构造回放或注入控制。
- 运动主机闭源且维修成本高，任何主动发包、改 systemd、改网络配置的操作都应单独评估。
- 如果目标是研究“一边运动一边俯仰”，优先抓包对比“掌机操作”和“代码操作”的差异，再回到官方 UDP 指令或现有 `lite3_motion_cmd.py` 层做安全实现。
- pcap 可能包含网络信息，不建议提交到 git。

依赖：

```bash
sudo apt install tcpdump
```

## 6. 机器狗环境快照采集

用于上机时记录 Jetson、CUDA、ROS、Python、RealSense、OpenCV、底层工具链、Docker、apt 包和网络等环境，避免后续为了装包污染系统后无法回溯。脚本只读采集，不安装、不升级、不删除、不修改系统配置。

推荐先 source 工作空间，这样 ROS 环境变量和 `rospack` 结果会更完整：

```bash
source ~/comp2026_ws/devel/setup.bash
cd ~/comp2026_ws/src/tools
bash collect_robot_env_snapshot.sh
```

默认输出：

```text
~/robot_env_snapshots/env_<时间戳>/
~/robot_env_snapshots/env_<时间戳>.tar.gz
~/robot_env_snapshots/env_<时间戳>.txt
```

指定输出目录：

```bash
bash collect_robot_env_snapshot.sh ~/robot_env_snapshots/before_vins_test
```

如果只想快速复制，把 `.txt` 单文件传回即可；如果要完整分析，把整个目录或 `.tar.gz` 传回开发机。重点文件：

```text
jetson_l4t.txt                 # JetPack/L4T 相关包
cuda.txt                       # CUDA/cuDNN/TensorRT 包和 nvcc
ros_env.txt                    # ROS_DISTRO、ROS_PACKAGE_PATH 等
ros_packages_core.txt          # 已安装 ROS 包
python3_import_versions.txt    # cv2/numpy/torch/pyrealsense2/rospy 等导入情况
pip3_freeze.txt                # Python pip 包快照
realsense_tools.txt            # librealsense/rs 工具
realsense_devices.txt          # RealSense 设备枚举，插着 D435i 时最有价值
native_libs_versions.txt       # OpenCV/Eigen/Boost/PCL/Ceres/yaml-cpp 等底层库
pkg_config_relevant.txt        # pkg-config 可见库
docker.txt                     # Docker 和镜像
apt_installed_all.txt          # 全量 dpkg 包快照
network_basic.txt              # ip addr/route/neigh
```

如果下一次是给 `dog_vins_localization` 做准备，特别关注：

```text
OpenCV
Eigen
Ceres / SuiteSparse
PCL
yaml-cpp
Boost
gcc / g++ / cmake
Python cv2 / numpy
RealSense / ROS image pipeline
```

注意：

- `env_filtered.txt` 只记录 ROS/CUDA/Python 等常见环境变量前缀，不主动记录密码类变量。
- `apt_installed_all.txt` 很长，但对判断“之前到底装过什么”很有用。
- `realsense_devices.txt` 需要 D435i 插着，并且当前没有被其他进程独占，才会记录到完整设备信息。

## 7. D435i 地面倾角标定

用于估计 D435i 安装后相对地面的倾角，方便不拆相机时做视角补偿，或打印新支架后做对比。工具通过深度图中下部区域拟合地面平面。

默认优先使用 `pyrealsense2` 直接打开 D435i，不需要 ROS：

```bash
python3 calibrate_d435i_ground_tilt.py --note bracket_v1
```

D435i 刚开流时可能有少量坏帧，脚本默认先丢弃 30 帧再开始拟合。需要调整时：

```bash
python3 calibrate_d435i_ground_tilt.py --warmup-frames 60 --note warmup_check
```

如果有多台 RealSense：

```bash
python3 calibrate_d435i_ground_tilt.py --serial <D435I_SERIAL> --note bracket_v2
```

如果已经启动 ROS 相机链路，也可用 ROS topic：

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
python3 calibrate_d435i_ground_tilt.py --backend ros --note ros_check
```

结果写入：

```text
d435i_ground_tilt_calibration.yaml
```

文件结构：

```yaml
latest:   # 最近一次测量
history:  # 历史测量列表
```

输出重点：

```text
pitch_about_optical_x_deg
roll_about_optical_z_deg
total_ground_normal_tilt_deg
normal_alignment_quaternion_xyzw
```

如果画面下半部分有腿、障碍物、墙面，缩小 ROI：

```bash
python3 calibrate_d435i_ground_tilt.py \
  --roi-left 0.25 --roi-right 0.75 \
  --roi-top 0.55 --roi-bottom 0.90 \
  --note clean_floor_roi
```

如果地面距离不合适：

```bash
python3 calibrate_d435i_ground_tilt.py --min-depth 0.4 --max-depth 2.5
```

只看结果，不写 YAML：

```bash
python3 calibrate_d435i_ground_tilt.py --no-save
```

注意：

- 深度光学坐标约定为 `+x` 向右、`+y` 向下、`+z` 向前。
- 水平地面的向上法向量应接近 `[0, -1, 0]`。
- `normal_alignment_quaternion_xyzw` 是把观测地面法向量对齐到水平地面的修正量；真正写入 `camera2base_tf.yaml` 前要在 RViz 里检查 `/scan`、TF 和 costmap。
- 多次测量标准差大时，通常是地面 ROI 不干净、深度噪声大或机器狗没有站稳。

相机 TF 配置分为三份：

```text
../allmovebase/config/camera2base_tf.yaml          # 默认配置
../allmovebase/config/camera2base_tf_tilted.yaml   # 当前倾斜安装补偿
../allmovebase/config/camera2base_tf_bracket.yaml  # 未来打印支架补偿
```

三份配置目前都保持空补偿。需要试倾斜安装配置时：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_tf_yaml:=$(rospack find allmovebase)/config/camera2base_tf_tilted.yaml
```

未来更换支架后：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_tf_yaml:=$(rospack find allmovebase)/config/camera2base_tf_bracket.yaml
```

依赖：

```bash
python3 -c "import pyrealsense2, numpy, yaml"
```

如果只用 ROS 后端，还需要 `rospy`、`cv_bridge`、`sensor_msgs`。

## 8. D435i 当前 FOV/内参测量

用于现场读取当前 D435i RGB 和 Depth 的实际分辨率与内参，并计算等效水平、垂直、对角 FOV。它不写死 width/height；机器狗上默认读取当前 ROS `CameraInfo`。

先启动相机或任务链路，再运行：

```bash
python3 measure_d435i_fov.py
```

只测深度或彩色：

```bash
python3 measure_d435i_fov.py --depth-only
python3 measure_d435i_fov.py --rgb-only
```

如果要显式指定 ROS 后端：

```bash
python3 measure_d435i_fov.py --backend ros
```

如果在开发机或其他环境中安装了 `pyrealsense2`，也可以直连相机读取 RealSense profile：

```bash
python3 measure_d435i_fov.py \
  --backend realsense \
  --rgb-width 640 --rgb-height 480 --rgb-fps 5 \
  --depth-width 640 --depth-height 480 --depth-fps 15
```

输出字段：

```text
resolution
fx / fy / cx / cy
hfov_deg / vfov_deg / dfov_deg
```

D435i 标称 FOV 需要区分 depth 和 color。常用资料口径如下：

```text
Intel ARK / D435i 官方规格页、RealSense 产品页：
  Depth FOV: H 87 deg, V 58 deg
  RGB FOV:   H 69 deg, V 42 deg

Intel RealSense D400 Series datasheet:
  Depth module left/right imager FOV: H 91.2 deg, V 65.5 deg, D 100.6 deg
  RGB sensor FOV:                    H 69.4 deg, V 42.5 deg, D 77.0 deg
```

实测等效 FOV 会随当前分辨率、裁剪、对齐和 ROS profile 变化。

当前已记录的一次实测结果如下。它只代表当时正在运行的相机 profile，后续修改分辨率或相机 launch 参数后应重新测量：

```text
RGB / Color:
  topic: /camera/color/camera_info
  resolution: 1280x720
  hfov_deg: 70.399133
  vfov_deg: 43.290963
  dfov_deg: 59.568128

Depth:
  topic: /camera/depth/camera_info
  resolution: 640x480
  hfov_deg: 78.971242
  vfov_deg: 63.426843
  dfov_deg: 72.127572
```

默认 ROS 后端依赖：

```bash
rospy
sensor_msgs
```

可选 RealSense 后端依赖：

```bash
python3 -c "import pyrealsense2"
```

## 9. D435i 多 profile 标定记录

用于下一次上机时批量记录 D435i 在不同 RGB/Depth 分辨率、帧率组合下的实际内参、FOV、畸变参数和外参线索。它和 `measure_d435i_fov.py` 的区别是：`measure_d435i_fov.py` 只读当前正在运行的 profile；本工具可以批量枚举或批量尝试多个 profile。

优先推荐在安装了 `pyrealsense2` 的环境中直连相机枚举全设备：

```bash
python3 calibrate_d435i_profiles.py \
  --backend pyrealsense2 \
  --output d435i_profile_calibration.yaml \
  --note field_full_enum
```

`pyrealsense2` 后端会尽量记录：

```text
color/depth/infrared profile 列表
width / height / fps / format
fx / fy / cx / cy
hfov_deg / vfov_deg / dfov_deg
distortion model / distortion coeffs
depth_to_color 外参
color_to_depth 外参
infrared_1_to_infrared_2 外参和 baseline 线索
```

机器狗上如果仍然没有 `pyrealsense2`，可用 ROS probe 后端。该模式会逐个启动 `allmovebase camera.launch`，读取 `/camera/color/camera_info` 和 `/camera/depth/camera_info`，成功的 profile 会写入结果，失败的 profile 会记录 error：

```bash
source ~/comp2026_ws/devel/setup.bash
cd ~/comp2026_ws/src/tools
python3 calibrate_d435i_profiles.py \
  --backend ros-probe \
  --quiet-roslaunch \
  --output d435i_profile_calibration.yaml \
  --note field_ros_probe
```

ROS probe 默认会尝试一组常用组合，例如 `640x480`、`848x480`、`1280x720` 的 color/depth 组合。也可以手动指定：

```bash
python3 calibrate_d435i_profiles.py \
  --backend ros-probe \
  --profiles 640x480@15:640x480@15,1280x720@5:640x480@15
```

如果要同时记录 RealSense 发布的 color-aligned depth CameraInfo：

```bash
python3 calibrate_d435i_profiles.py \
  --backend ros-probe \
  --align-depth \
  --quiet-roslaunch
```

注意：

- ROS probe 只能验证“你请求的 profile 是否能启动”，不能真正枚举设备支持的全部模式；真正全枚举需要 `pyrealsense2` 后端。
- 运行 ROS probe 前不要另开一个 `realsense2_camera`，否则相机设备会被占用。
- 当前导航仍使用 raw depth：`/camera/depth/image_rect_raw` 和 `/camera/depth/camera_info`。`--align-depth` 只用于记录 aligned depth 内参，不会改变主任务的 depth-to-scan 输入。
- 红外双目 baseline 最好以 `pyrealsense2` 后端记录的 `infrared_1_to_infrared_2` 外参为准；ROS CameraInfo 只能记录内参，通常不能完整给出双红外外参。

## 10. 任务位姿记录

用于实机记录 `task_poses.yaml` 中的位姿，例如：

- `rec_pose_1` 到 `rec_pose_4`
- `obs_start`、`obs_end`
- `pickup_pose`
- `place_pose_A` 到 `place_pose_D`

先启动导航/定位链路，确保 `/amcl_pose` 正常：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
rostopic echo /amcl_pose
```

交互式记录：

```bash
python3 record_task_pose.py
```

默认写入：

```text
~/comp2026_ws/src/allmovebase/config/task_poses.yaml
```

输入示例：

```text
rec_pose_1
goals pickup_pose
waypoints obs_start
waypoints obs_end
goals place_pose_A
```

记录一次后退出：

```bash
python3 record_task_pose.py --namespace goals --name rec_pose_1
python3 record_task_pose.py --namespace waypoints --name obs_start
```

如果记录 `PoseStamped` 类型话题：

```bash
python3 record_task_pose.py --pose-topic /some_pose --pose-type pose_stamped
```

检查结果：

```bash
grep -n "rec_pose_1\|obs_start\|pickup_pose\|place_pose_A" \
  ~/comp2026_ws/src/allmovebase/config/task_poses.yaml
```

依赖：ROS Noetic 环境，且当前终端已经 source 工作空间：

```bash
source ~/comp2026_ws/devel/setup.bash
```

## 11. 比赛过程 rosbag 录制

用于上机时尽量完整保存一次比赛过程，后续在开发机或 Jetson 上用 `rosbag play --clock` 回放定位、导航、识别和图像链路。

建议在任务 launch 已启动、主要 topic 已出现后另开终端：

```bash
cd ~/comp2026_ws/src/tools
python3 record_rosbag.py --profile full --split --split-size 4096
```

默认输出到：

```text
~/bags/
```

每次录制会生成：

```text
task_<时间戳>.bag
task_<时间戳>_manifest.json
```

`manifest.json` 会记录本次 profile、实际命令、topic 列表、启动时存在/缺失的 topic 和 `rostopic info`，方便回来复盘时知道这包到底录到了什么。

manifest 还会记录现场环境快照，包括 hostname、ROS 环境变量、git commit/dirty 状态、`~/bags` 剩余空间、部分命令版本和 `rosparam list`。如果需要完整参数快照：

```bash
python3 record_rosbag.py --profile full --rosparam-dump
```

常用 profile：

```bash
python3 record_rosbag.py --profile full        # 图像、导航、定位、任务状态全录
python3 record_rosbag.py --profile nav         # 不主动录 RGB/深度图，偏导航定位
python3 record_rosbag.py --profile perception  # 偏 D435i 图像和识别触发/结果
python3 record_rosbag.py --profile state       # 低负载状态记录
```

录完整过程但先检查 topic，不真正开录：

```bash
python3 record_rosbag.py --profile full --check-only
```

检查 topic 并额外测关键 topic 频率：

```bash
python3 record_rosbag.py --profile full --check-only --hz-check
```

只打印将要执行的 `rosbag record` 命令：

```bash
python3 record_rosbag.py --profile full --dry-run
```

全量录制负载较高时，可以降负载：

```bash
python3 record_rosbag.py --profile full --no-rgb
python3 record_rosbag.py --profile full --no-costmap
python3 record_rosbag.py --profile state
```

也可以叠加 profile 或追加自定义 topic：

```bash
python3 record_rosbag.py \
  --profile nav \
  --profile perception \
  --topic /your_extra_topic \
  --split \
  --split-size 4096
```

回放建议：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/task_<时间戳>.bag
```

注意：

- RGB + 深度原始图像会非常占磁盘和 I/O，正式上机前先短录 1 分钟估算包大小。
- 如果只复盘导航，优先用 `nav` 或 `state` profile。
- 如果要离线重跑识别，必须录 `/camera/color/image_raw`。
- 如果要复盘 D435i 到 `/scan` 的效果，必须录 `/camera/depth/image_rect_raw` 和 `/camera/depth/camera_info`。
- 如果要复盘 AMCL 和 move_base，必须录 `/tf`、`/tf_static`、`/scan`、`/map`、`/amcl_pose`、`/move_base/*` 相关状态和路径。

依赖：ROS Noetic、`rosbag`、`rostopic`。当前终端需要 source 工作空间：

```bash
source ~/comp2026_ws/devel/setup.bash
```

## 12. rosbag 离线抽帧

用于从实机 rosbag 中抽取图像，方便复盘 D435i 画面、识别采样质量和时间戳对齐。

默认抽取：

```text
color=/camera/color/image_raw:bgr8
depth=/camera/depth/image_rect_raw:passthrough
```

运行：

```bash
python3 extract_rosbag_frames.py \
  --bag ~/bags/test.bag \
  --output-dir ~/bag_extract/test \
  --associate color depth
```

输出：

```text
~/bag_extract/test/color/
~/bag_extract/test/depth/
~/bag_extract/test/color.txt
~/bag_extract/test/depth.txt
~/bag_extract/test/associate_color_depth.txt
```

自定义 topic：

```bash
python3 extract_rosbag_frames.py \
  --bag ~/bags/test.bag \
  --output-dir ~/bag_extract/test \
  --topic color=/camera/color/image_raw:bgr8 \
  --topic depth=/camera/depth/image_rect_raw:passthrough \
  --associate color depth
```

快速抽少量帧：

```bash
python3 extract_rosbag_frames.py --bag ~/bags/test.bag --output-dir ~/bag_extract/quick --limit 20
```

依赖：ROS Noetic、`rosbag`、`cv_bridge`、`cv2`。

## 13. 建议录包命令

优先使用 `record_rosbag.py`，它会检查 topic 并生成 manifest。下面是等价的手写命令，便于临时救急。

导航问题复盘：

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/nav_debug.bag \
  /tf /tf_static \
  /scan \
  /camera/depth/image_rect_raw /camera/depth/camera_info \
  /camera/color/image_raw \
  /amcl_pose /leg_odom2 /odom \
  /move_base/status /move_base/feedback /cmd_vel \
  /meter_inspect_trigger /meter_status /meter_state_json \
  /obstacle_task/report /inspect_report /full_task/report
```

识别问题复盘：

```bash
rosbag record -O ~/bags/meter_debug.bag \
  /camera/color/image_raw \
  /meter_inspect_trigger /meter_status /meter_state_json \
  /meter_inspection_ready
```

## 14. 维护约定

- 新增工具后优先更新本 README。
- 工具默认不要接入主任务 launch，除非确实变成比赛流程的一部分。
- 能单文件运行的工具尽量保持单文件，减少上机时找依赖的摩擦。
- 输出图片、bag、运行样本不要提交到 git。
