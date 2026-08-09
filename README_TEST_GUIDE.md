# 实机测试手册

本文用于上机前后快速检查 `comp2026_ws` 的节点、链路、参数和环境。所有命令默认在机器狗主机执行。

网络拓扑详见：

```text
README_NETWORK_TOPOLOGY.md
```

## 0. 基础环境要求

### ROS 工作空间

```bash
cd ~/comp2026_ws
catkin_make
source devel/setup.bash
```

建议每个新终端都先执行：

```bash
source ~/comp2026_ws/devel/setup.bash
```

### Git 仓库损坏抢修

2026-05-31 实机曾出现 loose object 空文件损坏：

```text
fatal: loose object <hash> is corrupt
```

推荐抢修流程是保留旧目录、重新 clone，再只恢复本地私有/运行文件：

```bash
cd ~
ts=$(date +%Y%m%d_%H%M%S)
mv ~/comp2026_ws ~/comp2026_ws_corrupt_$ts
git clone https://gitee.com/J1angJJ/comp2026_ws.git
cd ~/comp2026_ws
old=~/comp2026_ws_corrupt_$ts
```

只从旧目录恢复必要本地文件，避免复制损坏的 `.git`、`build`、`devel`、`log`：

```bash
cp -av "$old/src/tools/private_robot_access.yaml" ~/comp2026_ws/src/tools/
cp -av "$old/src/tools/d435i_ground_tilt_calibration.yaml" ~/comp2026_ws/src/tools/
cp -av "$old/src/allmovebase/config/camera2base_tf_tilted.yaml" ~/comp2026_ws/src/allmovebase/config/
mkdir -p ~/comp2026_ws/src/dog_motion/runtime
cp -av "$old/src/dog_motion/runtime/README.md" ~/comp2026_ws/src/dog_motion/runtime/
```

如果旧目录里有本地模型/容器辅助文件，也按需单独恢复：

```bash
mkdir -p ~/comp2026_ws/src/dog_motion/models
cp -av "$old/src/dog_motion/models/yuyin.engine" ~/comp2026_ws/src/dog_motion/models/
cp -av "$old/src/dog_motion/docker_tools" ~/comp2026_ws/src/dog_motion/
```

恢复后必须检查：

```bash
git status
catkin_make
source devel/setup.bash
rospack find dog_arm_bridge
```

### 运动主机通讯

运动主机 IP 在：

```text
src/message_transformer/launch/message_transformer.launch
```

当前默认：

```text
remote_ip=192.168.1.120
remote_port=43893
local_port=43894
```

上机前确认感知主机与运动主机网络互通：

```bash
ping 192.168.1.120
```

当前开发/实机网络结构记录：

```text
开发机和感知主机连接开发路由器/WiFi Bad_Puppy
开发路由器网关：192.168.31.1
感知主机 wlan0：192.168.31.174/24，DHCP 当前地址，负责 SSH、Git、Internet 和默认路由
机器狗内部网络：
  感知主机：192.168.1.103
  感知主机 wlan1：192.168.2.214，连接 YSC-JYML-dt3tfa-5G，仅访问 192.168.2.0/24
  运动主机：192.168.1.120
  运动主机热点网段别名：192.168.137.120
  运动主机 p2p0/AP 地址：192.168.2.1
  运动主机对外 WiFi/AP 名：YSC-JYML-dt3tfa-5G
  掌机：192.168.2.65
注意：感知主机 wlan1 配置为 ipv4.never-default yes、ipv6.never-default yes，默认路由只走 wlan0/Bad_Puppy。
SSH 用户名：
  感知主机：ysc
  运动主机：ysc
SSH 密码：设备出厂默认密码，本文不记录明文
```

建议把工具优先放在感知主机上运行。开发机访问网页调试工具、D435i 预览和 rosbag 文件时，使用感知主机 `wlan0` 当前 DHCP 地址；感知主机访问运动主机时使用内部网段 `192.168.1.x`。

如果 `Bad_Puppy` 管理网络不可用，但电脑能够连接机器狗 AP，可通过运动主机跳板登录感知主机：

```bash
ssh -J ysc@192.168.2.1 ysc@192.168.1.103
```

不要在没有物理控制台或其他备用入口时远程重启 NetworkManager。

私有连接信息不写入文档正文，使用本地配置文件管理：

```text
src/tools/private_robot_access.example.yaml  # 仓库模板
src/tools/private_robot_access.yaml          # 本地真实配置，已写入 .gitignore
```

上机时在感知主机上按模板填写真实密码。抓包工具默认读取该文件；如果配置 `sshpass: true`，会通过 `sshpass -e` 自动登录 SSH。

### D435i 相机

同一台 D435i 不要同时启动两个 `realsense2_camera` 进程。工程的总领 launch 已经按单相机进程组织。

建议先单独检查：

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
rostopic hz /camera/depth/image_rect_raw
```

识别需要彩色流时检查：

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
rostopic hz /camera/color/image_raw
```

### Docker 与 YOLO

识别必须在 Docker 镜像 `yolo11` 中运行。当前工程默认使用已加入 docker 组后的非 sudo 调用方式，建议提前确认：

```bash
docker ps
```

模型文件需要放在宿主机：

```text
~/comp2026_ws/src/dog_motion/models/yuyin.engine
```

对应容器内路径：

```text
/workspace/models/yuyin.engine
```

常驻识别容器启动后会发布：

```text
/meter_inspection_ready  std_msgs/Bool
```

值为 `true` 后才建议触发识别。

## 1. 主要总领链路

### 1.1 避障区域单段测试

用途：只测试从避障起点到终点的导航避障能力。

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

默认包含：

- `message_transformer`
- D435i 深度相机
- `depthimage_to_laserscan`
- `map_server`
- `amcl`
- `move_base`
- `obstacle_zone_task.py`

关键检查：

```bash
rostopic echo /lite3_motion_cmd
rostopic hz /scan
rostopic echo /amcl_pose
rostopic echo /move_base/status
rostopic echo /obstacle_task/report
```

### 1.2 正式导航识别链路

用途：导航到四个识别位点，按需打开彩色流，抬头识别，播报并记忆 A/B/C/D 状态。

```bash
roslaunch allmovebase task_2026_navigation.launch
```

如果动态开关彩色流不稳定，使用保守模式，低帧率常开彩色流：

```bash
roslaunch allmovebase task_2026_navigation.launch camera_enable_color:=true manage_color_stream:=false camera_color_fps:=5
```

默认识别顺序来自：

```text
src/allmovebase/config/task_poses.yaml
sequences.recognition: [rec_pose_1, rec_pose_2, rec_pose_4, rec_pose_3]
```

关键检查：

```bash
rostopic echo /meter_inspection_ready
rostopic echo /meter_inspect_trigger
rostopic echo /meter_status
rostopic echo /meter_state_json
rostopic echo /inspect_report
```

### 1.3 硬编码备用链路

用途：不依赖 `move_base`，直接按时间发布 `/cmd_vel`，用于导航负载过高或定位失败时备用。

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

只测试运动，不启动识别：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch run_meter_inspection:=false
```

默认路线假设机器人在避障终点朝向地图 `+Y`：

```text
obs_end
  -> rec_pose_1
  -> rec_pose_2
  -> 前进 0.45m、右转、前进 2m、右转、前进 1m
  -> rec_pose_4
  -> rec_pose_3
```

每个识别点默认右转 90 度进入识别姿态，识别后左转恢复路线方向。

## 2. 单节点调试

### 2.1 底层通讯节点

启动：

```bash
roslaunch message_transformer message_transformer.launch
```

包含：

- `qnx2ros`：运动主机到 ROS
- `ros2qnx`：ROS 到运动主机
- `lite3_motion_cmd.py`：高层动作封装
- `nx2app`
- `sensor_checker`

测试起立自检：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'ensure_stand'"
```

测试停止：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'stop'"
```

测试识别视角：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'inspection_view_pose'"
```

恢复导航视角：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'navigation_view_pose'"
```

### 2.2 速度发送

小速度前进 1 秒后停止：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

另开终端停止：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

### 2.3 深度转扫描

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
roslaunch allmovebase depth2laser.launch
rostopic hz /scan
rostopic echo -n 1 /scan
```

当前扫描范围：

```text
depth2laser range_max=3.0
amcl laser_max_range=3.0
costmap obstacle_range=3.0
costmap raytrace_range=3.0
```

### 2.3.1 D435i 地面倾角估计

如果 D435i 出厂安装后主光轴与地面不平行，可以先用深度图拟合地面平面，估计相机相对地面的倾角。该工具会把测量结果单独写入 `src/tools/d435i_ground_tilt_calibration.yaml`，不会自动修改 TF。

默认优先使用 `pyrealsense2` 直接打开 D435i，不依赖 ROS：

```bash
cd ~/comp2026_ws/src/tools
python3 calibrate_d435i_ground_tilt.py
```

如果同一台机器上接了多台 RealSense，可指定序列号：

```bash
python3 calibrate_d435i_ground_tilt.py --serial <D435I_SERIAL>
```

如果需要通过 ROS topic 标定，先启动深度相机：

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
```

然后运行：

```bash
cd ~/comp2026_ws/src/tools
python3 calibrate_d435i_ground_tilt.py --backend ros
```

输出重点看：

```text
pitch_about_optical_x_deg
roll_about_optical_z_deg
total_ground_normal_tilt_deg
normal_alignment_quaternion_xyzw
```

结果文件：

```text
src/tools/d435i_ground_tilt_calibration.yaml
```

文件中 `latest` 是最近一次测量，`history` 会保留历次结果。调支架时建议用 `--note` 标记版本：

```bash
python3 calibrate_d435i_ground_tilt.py --note bracket_v2
```

默认使用深度图中下部区域拟合地面；如果画面里有腿、障碍物或墙面，可缩小 ROI：

```bash
python3 calibrate_d435i_ground_tilt.py \
  --roi-left 0.25 --roi-right 0.75 \
  --roi-top 0.55 --roi-bottom 0.90
```

如果地面距离较远或较近，可调深度范围：

```bash
python3 calibrate_d435i_ground_tilt.py --min-depth 0.4 --max-depth 2.5
```

注意：

- 深度图光学坐标约定为 `+x` 向右、`+y` 向下、`+z` 向前。
- 水平地面的向上法向量应接近 `[0, -1, 0]`。
- 输出的 `normal_alignment_quaternion_xyzw` 是把观测地面法向量对齐到水平地面的修正量，真正写入 `camera2base_tf.yaml` 前必须在 RViz 中验证 `/scan`、TF 和代价地图是否更合理。
- 如果多次运行的标准差较大，说明 ROI 中地面不干净、深度噪声大或机器人姿态不稳定。

相机 TF 预留了三份配置：

```text
src/allmovebase/config/camera2base_tf.yaml          # 默认配置
src/allmovebase/config/camera2base_tf_tilted.yaml   # 当前倾斜安装补偿
src/allmovebase/config/camera2base_tf_bracket.yaml  # 未来打印支架补偿
```

三个文件目前都保持空补偿。临时切换倾斜安装配置：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_tf_yaml:=$(rospack find allmovebase)/config/camera2base_tf_tilted.yaml
```

未来支架安装后切换：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_tf_yaml:=$(rospack find allmovebase)/config/camera2base_tf_bracket.yaml
```

### 2.4 地图与 AMCL

```bash
roslaunch allmovebase map_server.launch
roslaunch allmovebase amcl.launch
```

地图分工：

- `/map`：导航地图，包含物理障碍和二维禁跨线。
- `/amcl_map`：定位地图，只包含实际物理障碍。

检查：

```bash
rostopic echo -n 1 /map_metadata
rostopic echo -n 1 /amcl_map_metadata
rostopic echo /amcl_pose
rosrun tf tf_echo map base_link
```

### 2.5 move_base

```bash
roslaunch allmovebase movebase.launch
```

当前局部规划器：

```text
teb_local_planner/TebLocalPlannerROS
```

全局规划器：

```text
navfn/NavfnROS
```

检查：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/feedback
```

### 2.5.1 导航可视化

完整诊断优先用 RViz，工程里已经准备了配置：

```bash
roslaunch allmovebase rviz_nav.launch
```

该配置默认固定坐标系为 `map`，会显示 `/map`、`/scan`、`/particlecloud`、`/amcl_pose`、全局/局部路径、全局/局部 costmap 和 TF。第一次用 RViz 时主要看三件事：

- `/scan` 是否贴合地图中的真实障碍。
- `/amcl_pose` 箭头是否和机器狗实际位置、朝向一致，粒子云是否收敛。
- 全局/局部路径和 costmap 是否合理，是否被错误障碍物堵死。

如果 NoMachine 很卡，可先不用 RViz，改用轻量网页看板。先启动包含相机、定位和导航的链路，例如：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
```

另开终端：

```bash
cd ~/comp2026_ws/src/tools
python3 ros_nav_debug_stream.py --port 8082
```

在开发机浏览器打开：

```text
http://<Jetson-IP>:8082
```

这个网页只看图，不发控制指令，默认显示 D435i 深度图和地图叠加 AMCL 位姿、粒子云、规划路径。状态页：

```text
http://<Jetson-IP>:8082/health
```

如果画面仍然卡，先降低网页推流负载：

```bash
python3 ros_nav_debug_stream.py --stream-fps 2 --jpeg-quality 55 --map-scale 1.0
```

整桌面推流现实上可以做，例如 VNC、noVNC、Xpra 或继续用 NoMachine，但不建议作为首选排障方案。它会把桌面、RViz、窗口合成和视频编码一起压到 Jetson 上，通常比只推两路 MJPEG 调试画面更吃资源；导航和推理负载未知时，优先保留算力给任务链路。

### 2.6 常驻 Docker 识别

建议先单独启动相机彩色流：

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
```

启动识别：

```bash
roslaunch dog_motion meter_reader_docker_persistent.launch start_camera:=false
```

等待：

```bash
rostopic echo /meter_inspection_ready
```

触发一次：

```bash
rostopic pub -1 /meter_inspect_trigger std_msgs/String "data: 'rec_pose_1'"
rostopic echo /meter_status
```

调试照片保存目录：

```text
src/dog_motion/runtime/meter_samples/
```

### 2.7 识别播报与状态记忆

单独启动：

```bash
rosrun dog_motion meter_audio_node.py
rosrun dog_motion meter_state_store_node.py
```

手动发布结果测试：

```bash
rostopic pub -1 /meter_status std_msgs/String "data: 'rec_pose_1,A,normal'"
rostopic echo /meter_state_json
rosparam get /meter_states
rosparam get /meter_states_ready
```

状态文件：

```text
src/allmovebase/config/meter_state.yaml
```

### 2.8 位姿记录工具

用于实机记录 `task_poses.yaml` 中的识别点、避障点、抓取点和放置点。

先启动基础导航链路，确保 `/amcl_pose` 正常：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
rostopic echo /amcl_pose
```

交互式记录：

```bash
cd ~/comp2026_ws/src/tools
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

也可以只记录一次后退出：

```bash
python3 record_task_pose.py --namespace goals --name rec_pose_1
python3 record_task_pose.py --namespace waypoints --name obs_start
```

如果记录的是 `PoseStamped` 类型话题，可改：

```bash
python3 record_task_pose.py --pose-topic /some_pose --pose-type pose_stamped
```

记录后检查：

```bash
grep -n "rec_pose_1\|obs_start\|pickup_pose\|place_pose_A" \
  ~/comp2026_ws/src/allmovebase/config/task_poses.yaml
```

### 2.9 rosbag 离线抽帧

用于从实机 rosbag 中抽取图像，方便离线复盘 D435i 画面、识别采样质量和时间戳对齐。

默认抽取 D435i 彩色和深度：

```bash
cd ~/comp2026_ws/src/tools
python3 extract_rosbag_frames.py \
  --bag ~/bags/test.bag \
  --output-dir ~/bag_extract/test \
  --associate color depth
```

默认 topic：

```text
color=/camera/color/image_raw:bgr8
depth=/camera/depth/image_rect_raw:passthrough
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

输出结构：

```text
~/bag_extract/test/color/
~/bag_extract/test/depth/
~/bag_extract/test/color.txt
~/bag_extract/test/depth.txt
~/bag_extract/test/associate_color_depth.txt
```

快速抽少量帧用于检查：

```bash
python3 extract_rosbag_frames.py --bag ~/bags/test.bag --output-dir ~/bag_extract/quick --limit 20
```

## 3. 主要可输入参数

### 3.1 `task_2026_obstacle_test.launch`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_camera` | `true` | 是否启动 D435i |
| `camera_enable_color` | `false` | 避障测试默认不开彩色 |
| `camera_color_fps` | `5` | 彩色流帧率 |
| `camera_depth_fps` | `15` | 深度流帧率 |
| `run_obstacle_task` | `true` | 是否启动避障任务节点 |
| `waypoint_order` | `obs_start,obs_end` | 避障点序列，或 YAML 中的序列名 |
| `nav_timeout` | `35.0` | 单目标导航超时 |
| `require_scan` | `true` | 是否要求 `/scan` 存在 |
| `scan_wait_timeout` | `8.0` | 等待 `/scan` 超时 |
| `prepare_motion_host` | `true` | 导航前是否调用底层任务级准备命令 |
| `motion_cmd_wait_timeout` | `5.0` | 等待 `/lite3_motion_cmd` 订阅者 |
| `motion_prepare_command` | `prepare_navigation` | 避障测试前的底层准备命令 |
| `motion_prepare_wait` | `1.0` | 发送准备命令后的等待 |
| `navigation_map_yaml` | `map/map.yaml` | 导航地图 |
| `amcl_map_yaml` | `map/map_amcl.yaml` | AMCL 定位地图 |

### 3.2 `task_2026_navigation.launch`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_camera` | `true` | 是否启动 D435i |
| `camera_enable_color` | `false` | 启动时是否打开彩色流 |
| `camera_color_fps` | `5` | 彩色流帧率 |
| `camera_depth_fps` | `15` | 深度流帧率 |
| `manage_color_stream` | `true` | 到识别点时动态开关彩色流 |
| `run_meter_audio` | `true` | 是否播报识别结果 |
| `host_workspace` | `$(find dog_motion)` | 挂载进 Docker 的宿主目录 |
| `model_in_container` | `/workspace/models/yuyin.engine` | 容器内模型路径 |
| `prepare_motion_host` | `true` | 导航前是否调用底层任务级准备命令 |
| `motion_prepare_command` | `prepare_navigation` | 正式导航前的底层准备命令 |
| `pre_detect_motion_command` | `inspection_view_pose` | 识别前动作 |
| `post_detect_motion_command` | `navigation_view_pose` | 识别后动作 |
| `detect_pose_settle` | `1.0` | 抬头后稳定时间 |
| `inspection_pitch_value` | `-6553` | 识别抬头俯仰值 |
| `navigation_pitch_value` | `0` | 导航视角俯仰值 |
| `navigation_map_yaml` | `map/map.yaml` | 导航地图 |
| `amcl_map_yaml` | `map/map_amcl.yaml` | AMCL 定位地图 |

内部传给识别任务的固定默认：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `detect_timeout` | `45.0` | 等待识别结果超时 |
| `detect_ready_topic` | `/meter_inspection_ready` | Docker ready 话题 |
| `detect_ready_timeout` | `120.0` | 等待 Docker ready 超时 |
| `detect_trigger_topic` | `/meter_inspect_trigger` | 识别触发 |
| `detect_result_topic` | `/meter_status` | 识别结果 |
| `detect_start_command` | `{goal}` | 触发内容为当前位点名 |

### 3.3 `task_2026_hardcoded_motion.launch`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_motion_bridge` | `true` | 是否启动底层通讯 |
| `start_camera` | `true` | 是否启动识别彩色相机 |
| `run_meter_inspection` | `true` | 是否启动常驻 Docker 识别 |
| `run_meter_audio` | `true` | 是否播报 |
| `host_workspace` | `$(find dog_motion)` | Docker 挂载目录 |
| `model_in_container` | `/workspace/models/yuyin.engine` | 容器内模型路径 |
| `meter_timeout` | `45.0` | 每个识别点等待结果超时 |
| `meter_ready_timeout` | `120.0` | 等待 Docker ready 超时 |
| `inspection_pitch_value` | `-6553` | 识别视角抬头值 |
| `navigation_pitch_value` | `0` | 导航视角恢复值 |
| `prepare_motion_host` | `true` | 运动前是否调用底层任务级准备命令 |
| `motion_prepare_command` | `prepare_hardcoded_motion` | 硬编码运动前的底层准备命令 |
| `motion_prepare_wait` | `1.0` | 发送准备命令后的等待 |
| `linear_speed` | `0.5` | 硬编码直走速度 |
| `turn_speed` | `0.5` | 硬编码转向角速度 |
| `turn_angle_deg` | `90.0` | 硬编码单次转向目标角度 |
| `turn_duration_scale` | `1.5` | 转向时长实机补偿倍率 |
| `closed_loop_motion` | `false` | 是否启用硬编码闭环运动 |
| `closed_loop_straight` | `true` | 启用闭环时，直行是否按 `/leg_odom2` 位移闭环 |
| `closed_loop_turn` | `true` | 启用闭环时，转向是否按 `/leg_odom2` yaw 闭环 |
| `closed_loop_require_feedback` | `false` | 闭环反馈缺失时是否直接失败；默认会回退到开环 |
| `closed_loop_odom_topic` | `/leg_odom2` | 硬编码闭环反馈里程计话题 |
| `closed_loop_distance_tolerance` | `0.05` | 直行闭环距离容差，单位 m |
| `closed_loop_turn_tolerance_deg` | `4.0` | 转向闭环角度容差，单位 deg |
| `closed_loop_max_time_scale` | `2.5` | 闭环单段最大耗时相对开环估计的倍率 |
| `settle_after_motion` | `0.2` | 每段运动后停止等待 |
| `inspect_pose_settle` | `1.0` | 识别姿态稳定等待 |
| `initial_turn_to_y_pos` | `none` | 起点若不朝 +Y，可设 `left/right` |
| `obs_end_to_rec_pose_1_distance` | `1.25` | obs_end 到 rec_pose_1 |
| `rec_pose_1_to_rec_pose_2_distance` | `2.5` | rec_pose_1 到 rec_pose_2 |
| `half_loop_leg_1_distance` | `0.45` | rec_pose_2 后前进 0.45m |
| `half_loop_leg_2_distance` | `2.0` | 横向前进 2m |
| `half_loop_leg_3_distance` | `1.0` | 再前进 1m 到 rec_pose_4 |
| `rec_pose_4_to_rec_pose_3_distance` | `2.5` | rec_pose_4 到 rec_pose_3 |
| `rec_pose_1_inspect_turn` | `right` | 识别点侧转方向 |
| `rec_pose_2_inspect_turn` | `right` | 识别点侧转方向 |
| `rec_pose_4_inspect_turn` | `right` | 识别点侧转方向 |
| `rec_pose_3_inspect_turn` | `right` | 识别点侧转方向 |

### 3.4 `message_transformer.launch`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `inspection_pitch_value` | `-6553` | 识别前抬头值 |
| `navigation_pitch_value` | `0` | 识别后恢复值 |
| `view_pose_step_sleep` | `0.5` | 视角组合动作间隔 |

节点内重要固定参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `stand_settle_sec` | `3.0` | 起立后等待 |
| `enter_move_mode_after_stand` | `true` | 起立后进入移动模式 |
| `robot_basic_state_topic` | `/lite3/robot_basic_state` | 站立状态反馈 |
| `standing_basic_states` | `1,2` | 认为已站立的状态码 |
| `low_pose_height_value` | `-20000` | 降低高度值 |
| `normal_height_value` | `0` | 正常高度值 |

### 3.5 `meter_reader_docker_persistent.launch`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_camera` | `false` | 是否由识别 launch 自己启动相机 |
| `run_audio` | `true` | 是否启动播报 |
| `run_state_store` | `true` | 是否启动状态记忆 |
| `image_topic` | `/camera/color/image_raw` | 彩色图输入 |
| `trigger_topic` | `/meter_inspect_trigger` | 识别触发 |
| `result_topic` | `/meter_status` | 识别结果 |
| `ready_topic` | `/meter_inspection_ready` | 常驻容器 ready |
| `state_topic` | `/meter_state_json` | 状态 JSON |
| `state_file` | `allmovebase/config/meter_state.yaml` | 状态落盘 |
| `state_clear_on_start` | `true` | 启动状态记忆节点时清空上一轮识别结果 |
| `audio_dir` | `dog_motion/audio` | 语音文件目录 |
| `host_workspace` | `$(find dog_motion)` | Docker 挂载宿主目录 |
| `container_workspace` | `/workspace` | Docker 工作目录 |
| `docker_image` | `yolo11` | Docker 镜像 |
| `docker_command` | `docker` | Docker 命令 |
| `model_in_container` | `/workspace/models/yuyin.engine` | 模型路径 |
| `warmup_frames` | `15` | 触发后预热帧数 |
| `sample_count` | `5` | 采样照片数量 |
| `sample_interval` | `0.15` | 采样间隔 |
| `min_confidence` | `0.25` | YOLO 最低置信度 |
| `container_ready_timeout` | `90.0` | 容器启动超时 |
| `infer_timeout` | `45.0` | 单次推理超时 |

## 4. 关键动作封装

通过 `/lite3_motion_cmd` 发布 `std_msgs/String`。

| 命令 | 作用 | 备注 |
| --- | --- | --- |
| `ensure_stand` | 起立自检，不站立才发 toggle | 依赖 `/lite3/robot_basic_state`，无反馈时会 fallback 发 toggle |
| `stand` | 起立/趴下 toggle | 使用前确认当前姿态 |
| `lie` | 趴下 toggle | 同一个底层 toggle，使用前确认当前姿态 |
| `move_mode` | 进入移动模式 | 导航需要 |
| `spot_mode` | 原地姿态模式 | 识别抬头前使用 |
| `stop` | 连续发送零速度 | 停稳 |
| `inspection_view_pose` | 停稳、spot、抬头 | 识别前默认动作 |
| `navigation_view_pose` | 恢复俯仰、move_mode | 识别后默认动作 |
| `height:<value>` | 调整高度 | 例如 `height:-20000` |
| `low_pose` | 进入低姿态组合动作 | 备用 |
| `normal_pose` | 恢复正常高度 | 备用 |
| `pitch:<value>` | 原始俯仰值 | 备用 |
| `yaw:<value>` | 原始偏航值 | 备用 |
| `raw:<code>:<value>:<type>` | 原始 SimpleCMD | 谨慎使用 |

完整封装表见：

```text
src/message_transformer/docs/LITE3_MOTION_CMD.md
```

### 4.1 运动主机被动抓包

如果需要研究掌机和代码控制的差异，优先使用被动抓包，不要直接在运动主机上改配置或主动发未知包。当前工程默认运动主机地址为 `192.168.1.120`，开发者 UDP 控制目标端口为 `43893`，状态回传常见端口为 `43897`，掌机/APP 相关端口为 `43899`，广角 RTSP 为 `8554`。

实测掌机链路位于运动主机 `p2p0=192.168.2.1/24` 下：

```text
掌机：192.168.2.65
```

注意：这些是工程和厂家文档中已知端口，不代表已经确认掌机与运动主机直连通讯端口。若要发现未知端口，先只按 IP 抓全端口：

```bash
cd ~/comp2026_ws/src/tools
python3 motion_host_packet_capture.py capture --all-ports --duration 30
```

已确认掌机 IP 时，可加上掌机 host 过滤：

```bash
python3 motion_host_packet_capture.py capture \
  --all-ports \
  --host 192.168.2.65 \
  --duration 30
```

感知主机大概率看不到 `p2p0` 上掌机与运动主机的直连流量。若需要抓掌机链路，优先远程普通权限在运动主机 `p2p0` 上抓：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.65 \
  --all-ports \
  --duration 60 \
  --output ~/packet_captures/handheld_192_168_2_65.pcap
```

在感知主机上抓 60 秒：

```bash
cd ~/comp2026_ws/src/tools
python3 motion_host_packet_capture.py capture --duration 60
```

只打印命令，人工确认过滤条件：

```bash
python3 motion_host_packet_capture.py capture --dry-run
```

如果确认可以 SSH 到运动主机，远程抓包并把 pcap 写回感知主机：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --duration 60 \
  --output ~/packet_captures/motion_host_remote.pcap
```

`remote-capture` 默认连接 `ysc@192.168.1.120`。如果地址或用户名变化，再显式指定：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --ssh ysc@192.168.1.120 \
  --duration 60
```

如果 `src/tools/private_robot_access.yaml` 中设置了 `motion_host_ssh_password` 和 `sshpass: true`，工具会自动使用 `sshpass -e` 登录；否则会走普通 SSH 交互。远程模式默认不提权，只在运动主机普通用户权限下运行 `tcpdump`。如果没有抓包权限，优先退回感知主机侧抓包；确实万不得已再显式加 `--sudo`。

如果 sudo 密码与 SSH 登录密码相同，且确实需要 sudo，在私有配置里临时设置：

```yaml
sudo: true
sudo_password_same_as_ssh: true
sudo_with_password: true
```

此时远程抓包会用 `sudo -S` 通过 SSH 标准输入传递 sudo 密码，不会把密码打印到命令行。平时不建议长期打开。

离线查看摘要：

```bash
python3 motion_host_packet_capture.py summarize \
  --pcap ~/packet_captures/motion_host_remote.pcap \
  --count 50
```

轻量解码今年工程中已知 UDP 结构：

```bash
python3 motion_host_packet_capture.py decode \
  --pcap ~/packet_captures/motion_host_remote.pcap \
  --count 100
```

解码只作为线索：`SimpleCMD`、`ComplexCMD` 和掌机 `JoystickChannelFrame` 能被打印出来，但未知包不要直接回放。若目标是补偿 D435i 初始俯仰角，建议先对比三组包：静止俯仰、低速运动、掌机一边跑一边俯仰；确认差异后优先回到官方 UDP 指令和 `lite3_motion_cmd.py` 封装层实现。

## 5. 推荐实机测试顺序

0. 上机前预检磁盘、模型、Docker、运动主机网络和关键 ROS topic。

```bash
cd ~/comp2026_ws/src/tools
python3 preflight.py
```

如果任务链路还没启动，ROS topic 相关项会告警；启动任务链路后建议再跑一次：

```bash
python3 preflight.py --min-free-gb 30
```

1. 只启动底层通讯，测试 `ensure_stand`、`stop`。

```bash
roslaunch message_transformer message_transformer.launch
```

2. 测 D435i 深度和 `/scan`。

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
roslaunch allmovebase depth2laser.launch
rostopic hz /scan
```

3. 测避障导航。

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

4. 测硬编码运动，不识别。

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch run_meter_inspection:=false
```

5. 单独测常驻 Docker 识别。

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
roslaunch dog_motion meter_reader_docker_persistent.launch start_camera:=false
```

6. 测完整硬编码识别备用链路。

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

7. 最后测完整导航识别链路。

```bash
roslaunch allmovebase task_2026_navigation.launch
```

## 6. 常见问题定位

### 没有起立

检查：

```bash
rostopic echo /lite3_motion_cmd
rostopic echo /simple_cmd
rostopic echo /lite3/robot_basic_state
```

如果没有 `/lite3/robot_basic_state`，`ensure_stand` 会 fallback 发送起立/趴下 toggle。此时必须人工确认机器狗当前是趴下还是站立。

### 没有速度

检查：

```bash
rostopic echo /cmd_vel
rostopic echo /simple_cmd
```

确认 `ros2qnx` 已启动，运动主机 IP 正确，网络互通。

### 没有 `/scan`

检查：

```bash
rostopic hz /camera/depth/image_rect_raw
rostopic hz /camera/depth/camera_info
rosnode list | grep depthimage_to_laserscan
```

进一步检查扫描数据是否新鲜、频率是否稳定：

```bash
rostopic echo -n 1 /scan
rostopic hz /scan
rostopic hz /camera/depth/image_rect_raw
rostopic hz /camera/depth/camera_info
```

### AMCL 不收敛

检查：

```bash
rostopic echo /initialpose
rostopic echo /amcl_pose
rosrun tf tf_echo map base_link
rosrun tf tf_echo odom base_link
rosrun rqt_tf_tree rqt_tf_tree
```

注意 AMCL 地图 `/amcl_map` 只应包含真实可观测物理障碍。

如果怀疑地图或定位源异常：

```bash
rostopic echo -n 1 /map_metadata
rostopic echo -n 1 /amcl_map_metadata
rostopic hz /map
rostopic hz /amcl_pose
rostopic hz /leg_odom2
```

### move_base 不动作或卡住

检查 action 状态、全局/局部规划和速度输出：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/feedback
rostopic echo /move_base/NavfnROS/plan
rostopic echo /move_base/TebLocalPlannerROS/local_plan
rostopic echo /cmd_vel
```

如果 `/cmd_vel` 有速度但机器狗不动，继续检查底层通讯：

```bash
rostopic echo /simple_cmd
rostopic echo /lite3/robot_basic_state
ping 192.168.1.120
```

### Docker 识别没启动

检查：

```bash
rostopic echo /meter_inspection_ready
rosnode info /meter_persistent_docker_inspection_node
docker ps
```

若 `docker ps` 提示权限不足，重新登录当前用户会话，或临时使用 `docker_command:="sudo docker"`。

### 识别无结果

检查：

```bash
rostopic echo /meter_inspect_trigger
rostopic echo /meter_status
rostopic hz /camera/color/image_raw
ls ~/comp2026_ws/src/dog_motion/runtime/meter_samples
```

确认采样图片已保存，模型文件存在，Docker 日志没有报错。

### 语音不播报

检查：

```bash
rostopic echo /meter_status
rosnode info /meter_audio_node
ls ~/comp2026_ws/src/dog_motion/audio
```

语音节点接收格式：

```text
rec_pose_1,A,normal
A,normal
```

### 状态没有记忆

检查：

```bash
rostopic echo /meter_state_json
rosparam get /meter_states
rosparam get /meter_states_ready
cat ~/comp2026_ws/src/allmovebase/config/meter_state.yaml
```

### 录包与复盘

上机遇到偶现问题时，建议短时间录包，避免只靠终端回忆：

```bash
cd ~/comp2026_ws/src/tools
python3 record_rosbag.py --profile full --split --split-size 4096
```

这会按比赛调试 profile 录制图像、定位、导航、识别和任务状态 topic，并在 `~/bags` 下生成同名 `manifest.json`，记录本次 topic 是否存在。先检查但不录制：

```bash
python3 record_rosbag.py --profile full --check-only
```

如果要连关键 topic 频率也一起检查：

```bash
python3 record_rosbag.py --profile full --check-only --hz-check
```

`manifest.json` 会记录 hostname、ROS 环境变量、git commit/dirty 状态、磁盘剩余空间、topic 信息和 `rosparam list`。如果需要完整参数快照，可加：

```bash
python3 record_rosbag.py --profile full --rosparam-dump
```

如果只排查导航定位，降低负载：

```bash
python3 record_rosbag.py --profile nav --split --split-size 4096
```

如果只想保留低负载状态流：

```bash
python3 record_rosbag.py --profile state
```

全量录制会很占磁盘，尤其是 `/camera/color/image_raw` 和 `/camera/depth/image_rect_raw`。实机前建议先短录 1 分钟估算容量；如果磁盘或 I/O 压力偏高，可加 `--no-rgb`、`--no-costmap`，或改用 `nav/state` profile。

回放时：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/task_<时间戳>.bag
```

等价的手写命令如下，便于脚本不可用时救急：

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

如果只排查识别，减少负载：

```bash
rosbag record -O ~/bags/meter_debug.bag \
  /camera/color/image_raw \
  /meter_inspect_trigger /meter_status /meter_state_json \
  /meter_inspection_ready
```

## 7. 关键文件速查

```text
src/tools/preflight.py
src/tools/record_task_pose.py
src/tools/record_rosbag.py
src/tools/extract_rosbag_frames.py
src/tools/d435i_stream_test.py
src/tools/ros_nav_debug_stream.py
src/tools/motion_host_packet_capture.py
src/tools/calibrate_d435i_ground_tilt.py
src/allmovebase/launch/rviz_nav.launch
src/allmovebase/rviz/nav_debug.rviz
src/allmovebase/launch/task_2026_obstacle_test.launch
src/allmovebase/launch/task_2026_navigation.launch
src/allmovebase/launch/task_2026_hardcoded_motion.launch
src/allmovebase/config/task_poses.yaml
src/allmovebase/config/meter_state.yaml
src/allmovebase/map/map.yaml
src/allmovebase/map/map_amcl.yaml
src/allmovebase/config/amcl.yaml
src/allmovebase/config/teb_local_planner_params.yaml
src/message_transformer/launch/message_transformer.launch
src/message_transformer/scripts/lite3_motion_cmd.py
src/message_transformer/docs/LITE3_MOTION_CMD.md
src/dog_motion/launch/meter_reader_docker_persistent.launch
src/dog_motion/scripts/meter_persistent_docker_inspection_node.py
src/dog_motion/scripts/meter_persistent_infer.py
src/dog_motion/scripts/meter_audio_node.py
src/dog_motion/scripts/meter_state_store_node.py
src/dog_motion/models/yuyin.engine
src/dog_motion/runtime/meter_samples/
README_NETWORK_TOPOLOGY.md
README_FIELD_TEST_CHECKLIST.md
```
