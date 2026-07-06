# comp2026_ws 复现流程

本文基于当前 `comp2026_ws` 工作空间静态梳理生成，适合在机器狗感知主机/Jetson 上复现。当前普通 Windows 工作目录只能做代码阅读和文件检查，不能完整运行 ROS、D435i、Docker YOLO 和运动主机链路。

## 1. 代码结构速览

当前目录是 ROS/catkin 工作空间，核心包如下：

| 包 | 作用 |
| --- | --- |
| `allmovebase` | 比赛主状态机，包含导航、避障、巡检、抓放、全流程入口 |
| `message_transformer` | Lite3 运动主机 UDP/ROS 桥，负责 `/cmd_vel`、`/simple_cmd`、运动状态回传和高层动作命令 |
| `dog_motion` | Docker YOLO 仪表识别、语音播报、识别状态存储 |
| `dog_arm_bridge` | 机器狗 ROS1 到机械臂侧协议的话题适配 |
| `depthimage_to_laserscan` | D435i 深度图转 `/scan` |
| `lite3_description` | Lite3 URDF、mesh、RViz 检查资源 |
| `tools` | 上机预检、录点、录包、相机标定、网页调试等辅助工具 |

主入口集中在：

```text
src/allmovebase/launch/task_2026_obstacle_test.launch
src/allmovebase/launch/task_2026_navigation.launch
src/allmovebase/launch/task_2026_pick_place.launch
src/allmovebase/launch/task_2026_full.launch
src/allmovebase/launch/task_2026_hardcoded_motion.launch
```

共享导航栈入口：

```text
src/allmovebase/launch/stack_nav_base.launch
```

它会启动 D435i、相机 TF、里程计、`depthimage_to_laserscan`、双地图、AMCL 和 `move_base`。

## 2. 复现前必须确认

### 2.1 工作区状态

当前仓库显示有未提交改动，其中最关键的是：

```text
D src/CMakeLists.txt
M src/dog_arm_bridge/scripts/dog_arm_bridge_node.py
M src/message_transformer/launch/message_transformer.launch
?? README.md
```

`src/CMakeLists.txt` 是 catkin 工作空间编译入口。如果它确实缺失，先在机器狗上恢复：

```bash
cd ~/comp2026_ws/src
catkin_init_workspace
```

然后再回到工作空间根目录编译。

### 2.2 运行环境

建议环境：

```text
Ubuntu + ROS Noetic
catkin
RealSense D435i / realsense2_camera
Docker，且当前用户能运行 docker ps
Docker 镜像 yolo11
TensorRT engine: src/dog_motion/models/yuyin.engine
```

运动主机默认网络参数在 `src/message_transformer/launch/message_transformer.launch`：

```text
remote_ip=192.168.1.120
remote_port=43893
local_port=43894
```

上机前先确认：

```bash
ping 192.168.1.120
docker ps
ls ~/comp2026_ws/src/dog_motion/models/yuyin.engine
```

## 3. 基础编译

在机器狗感知主机执行：

```bash
cd ~/comp2026_ws
catkin_make
source devel/setup.bash
```

每个新终端都执行：

```bash
source ~/comp2026_ws/devel/setup.bash
```

如果 `catkin_make` 找不到顶层 CMakeLists，回到第 2.1 节恢复 `src/CMakeLists.txt`。

编译后做包发现检查：

```bash
rospack find allmovebase
rospack find message_transformer
rospack find dog_motion
rospack find dog_arm_bridge
```

## 4. 上机预检

工具目录：

```bash
cd ~/comp2026_ws/src/tools
```

先跑只读预检：

```bash
python3 preflight.py
```

如果任务链路还没启动，ROS topic 相关项可能告警，这是正常的；重点先看磁盘、Docker、模型文件、运动主机网络。

建议再采集一次环境快照，方便后续复盘：

```bash
bash collect_robot_env_snapshot.sh
```

## 5. 分模块复现

按下面顺序执行，前一步稳定后再进入下一步。涉及运动的命令执行前，保证机器狗周围安全、急停可用。

### 5.1 运动主机通讯

启动底层桥：

```bash
roslaunch message_transformer message_transformer.launch
```

另开终端测试：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'ensure_stand'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'stop'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'inspection_view_pose'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'navigation_view_pose'"
```

观察：

```bash
rostopic echo /simple_cmd
rostopic echo /lite3/robot_basic_state
```

### 5.2 D435i 与 `/scan`

启动深度相机：

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
```

另开终端：

```bash
roslaunch allmovebase depth2laser.launch
rostopic hz /camera/depth/image_rect_raw
rostopic hz /scan
rostopic echo -n 1 /scan
```

### 5.3 地图、AMCL、move_base

启动不执行任务的导航链路：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
```

检查：

```bash
rostopic echo /amcl_pose
rostopic echo /move_base/status
rosrun tf tf_echo map base_link
```

可视化：

```bash
roslaunch allmovebase rviz_nav.launch
```

NoMachine 卡顿时，用轻量网页看板：

```bash
cd ~/comp2026_ws/src/tools
python3 ros_nav_debug_stream.py --port 8082
```

浏览器打开：

```text
http://<Jetson-IP>:8082
```

### 5.4 任务位姿录入

当前 `src/allmovebase/config/task_poses.yaml` 中：

```text
rec_pose_1/2/3/4 有示例值
obs_start/obs_end 有示例值
pickup_pose 和 place_pose_A/B/C/D 仍是 [0,0,0] 占位
```

实机复现前必须录入真实赛场位姿：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
cd ~/comp2026_ws/src/tools
python3 record_task_pose.py
```

建议至少录入：

```text
waypoints obs_start
waypoints obs_end
goals rec_pose_1
goals rec_pose_2
goals rec_pose_3
goals rec_pose_4
goals pickup_pose
goals place_pose_A
goals place_pose_B
goals place_pose_C
goals place_pose_D
```

检查：

```bash
grep -n "rec_pose_1\|obs_start\|pickup_pose\|place_pose_A" \
  ~/comp2026_ws/src/allmovebase/config/task_poses.yaml
```

## 6. 单任务复现

### 6.1 避障段

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

观察：

```bash
rostopic echo /obstacle_task/report
rostopic echo /move_base/status
rostopic echo /cmd_vel
rostopic hz /scan
```

该任务默认按 `sequences.obstacle_test: [obs_start, obs_end]` 发送导航目标。

### 6.2 Docker 仪表识别

先启动彩色相机：

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
```

另开终端启动常驻识别：

```bash
roslaunch dog_motion meter_reader_docker_persistent.launch start_camera:=false
```

等待 ready：

```bash
rostopic echo /meter_inspection_ready
```

手动触发：

```bash
rostopic pub -1 /meter_inspect_trigger std_msgs/String "data: 'rec_pose_1'"
rostopic echo /meter_status
```

采样图保存目录：

```text
src/dog_motion/runtime/meter_samples/
```

### 6.3 巡检导航识别

```bash
roslaunch allmovebase task_2026_navigation.launch
```

如果动态开关彩色流不稳定，用保守模式：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_enable_color:=true \
  manage_color_stream:=false \
  camera_color_fps:=5
```

默认识别顺序：

```text
rec_pose_1 -> rec_pose_2 -> rec_pose_4 -> rec_pose_3
```

观察：

```bash
rostopic echo /meter_inspect_trigger
rostopic echo /meter_status
rostopic echo /meter_state_json
rostopic echo /inspect_report
```

识别状态会写入：

```text
src/allmovebase/config/meter_state.yaml
```

### 6.4 抓取放置狗端流程

前提：

```text
1. meter_state.yaml 已经有 A/B/C/D 识别状态
2. pickup_pose 和 place_pose_A/B/C/D 已经录入真实位姿
3. 机械臂端或模拟节点能订阅 /dog_arm/task_cmd 并发布 /dog_arm/task_result
```

启动：

```bash
roslaunch allmovebase task_2026_pick_place.launch
```

观察：

```bash
rostopic echo /dog_arm/task_cmd
rostopic echo /dog_arm/task_result
rostopic echo /pick_place_task/report
```

正式联调建议把机械臂命令要求打开，避免没有机械臂订阅者时误以为流程可用：

```bash
roslaunch allmovebase task_2026_pick_place.launch arm_command_required:=true
```

### 6.5 硬编码备用路线

不使用 `move_base`，按距离和转角发布 `/cmd_vel`：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

只测运动、不启用识别：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch run_meter_inspection:=false
```

注意：硬编码路线对起点、朝向、速度标定和地面摩擦非常敏感，只适合作为定位/导航不稳定时的备用方案。

## 7. 全流程复现

确认以下条件全部满足后，再运行全任务：

```text
1. catkin_make 成功
2. 运动主机 192.168.1.120 可 ping 通
3. /lite3_motion_cmd 能完成 ensure_stand、stop、视角切换
4. D435i 深度图和 /scan 稳定
5. AMCL 已收敛，/amcl_pose 与真实位置一致
6. obs_start/obs_end、rec_pose_1/2/3/4、pickup_pose、place_pose_A/B/C/D 均为真实位姿
7. Docker 镜像 yolo11 可用，yuyin.engine 存在
8. /meter_inspection_ready 能变为 true
9. 机械臂端协议已联调，/dog_arm/task_result 能返回结果
```

启动：

```bash
roslaunch allmovebase task_2026_full.launch
```

观察：

```bash
rostopic echo /full_task/report
rostopic echo /full_task/succeeded
rostopic echo /meter_status
rostopic echo /dog_arm/task_cmd
rostopic echo /dog_arm/task_result
```

常用裁剪运行：

```bash
# 只跑避障和巡检，不跑抓放
roslaunch allmovebase task_2026_full.launch run_pick_place:=false

# 跳过避障，从巡检开始
roslaunch allmovebase task_2026_full.launch run_obstacle:=false

# 只用已有识别结果测试抓放
roslaunch allmovebase task_2026_full.launch \
  run_obstacle:=false \
  run_inspection:=false \
  run_pick_place:=true
```

正式联调建议：

```bash
roslaunch allmovebase task_2026_full.launch arm_command_required:=true
```

## 8. 录包与复盘

执行任务时建议另开终端录包：

```bash
cd ~/comp2026_ws/src/tools
python3 record_rosbag.py --profile full --split --split-size 4096
```

只排查导航：

```bash
python3 record_rosbag.py --profile nav --split --split-size 4096
```

只排查识别：

```bash
python3 record_rosbag.py --profile perception --split --split-size 4096
```

回放：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/task_<时间戳>.bag
```

从 rosbag 抽图：

```bash
python3 extract_rosbag_frames.py \
  --bag ~/bags/task_<时间戳>.bag \
  --output-dir ~/bag_extract/task_<时间戳> \
  --associate color depth
```

## 9. 常见失败点

| 现象 | 优先检查 |
| --- | --- |
| `catkin_make` 失败 | `src/CMakeLists.txt` 是否存在，依赖包是否安装 |
| 机器狗不响应 | `ping 192.168.1.120`、`/simple_cmd`、`/lite3/robot_basic_state` |
| 没有 `/scan` | `/camera/depth/image_rect_raw`、`/camera/depth/camera_info`、`depthimage_to_laserscan` 节点 |
| AMCL 不收敛 | 地图、初始位姿、TF、`/leg_odom2`、`/scan` 是否贴合地图 |
| `move_base` 不动 | `/move_base/status`、`/cmd_vel`、局部 costmap 是否被障碍堵死 |
| Docker 识别无 ready | `docker ps`、`yolo11` 镜像、`yuyin.engine` 路径 |
| 识别无结果 | `/camera/color/image_raw`、`/meter_inspect_trigger`、采样图是否保存 |
| 抓放流程卡住 | `/dog_arm/task_cmd` 是否发出，机械臂端是否回 `/dog_arm/task_result` |

## 10. 推荐复现顺序总结

```text
1. 修复/确认 src/CMakeLists.txt
2. catkin_make + source
3. preflight.py
4. message_transformer 底层通讯
5. D435i 深度和 /scan
6. AMCL + move_base 空跑检查
7. 录入真实任务位姿
8. 单独跑避障
9. 单独跑 Docker 识别
10. 跑巡检导航识别
11. 联调机械臂抓放协议
12. 跑 task_2026_pick_place.launch
13. 最后跑 task_2026_full.launch
14. 全程录包，按 rosbag 和采样图复盘
```

