# comp2026_ws 复现流程

本文用于在机器狗 Jetson（Ubuntu 20.04 + ROS Noetic）上逐步复现 `comp2026_ws`。暂不包含 VINS。

## 0. 使用规则

- 本文命令默认在机器狗感知主机执行，标明“虚拟机”的命令除外。
- 每个新终端先执行 `source ~/comp2026_ws/devel/setup.bash`。
- 同一时间只运行一个总领任务 launch。进入下一步前，先用 `Ctrl+C` 停止上一步启动的节点。
- 同一台 D435i 不能同时启动两个 `realsense2_camera`。
- 所有涉及起立或移动的测试都必须确保急停可用、机器狗周围无人。
- 只在使用远程 RViz 时执行 `ros_master`。单机比赛运行时不需要执行它。

## 1. 代码与 Git 检查

2026-08-08 修订本文前的本地核验结果：

```text
commit: 22fc537 修正IP地址配置内容
git status --short: 无输出
src/CMakeLists.txt: 不存在
```

每次复现仍应以机器狗上的实际输出为准：

```bash
cd ~/comp2026_ws
git status --short
git log -1 --oneline
```

`git status --short` 无输出表示工作区干净。如果有输出，先记录改动，不要直接回退或覆盖。

## 2. 恢复 catkin 入口并编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/comp2026_ws

if [ ! -e src/CMakeLists.txt ]; then
  cd src
  catkin_init_workspace
  cd ..
fi

rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

检查包和 Python 依赖：

```bash
rospack find allmovebase
rospack find message_transformer
rospack find dog_motion
rospack find dog_arm_bridge

python3 -c "import rospy, yaml, cv2, pygame, actionlib; from move_base_msgs.msg import MoveBaseAction; from task_budget import TaskBudget; from dog_arm_task_client import DogArmTaskClient"
```

通过标准：`catkin_make` 成功，四个 ROS 包都能被找到，Python 导入无报错。

## 3. 上机预检

任务链路尚未启动时执行：

```bash
cd ~/comp2026_ws/src/tools
python3 preflight.py --skip-hz
```

重点确认：

```bash
ping -c 4 192.168.1.120
docker ps
docker image inspect yolo11
test -s ~/comp2026_ws/src/dog_motion/models/yuyin.engine
df -h ~/bags
```

通过标准：运动主机可达、Docker 可用、`yolo11` 镜像存在、`yuyin.engine` 非空、磁盘空间充足。

注意：`yuyin.engine` 被 Git 忽略，`yolo11` 镜像也不能通过普通 Git 克隆恢复，必须单独保存。

## 4. 运动主机通信

启动：

```bash
roslaunch message_transformer message_transformer.launch
```

另开终端检查：

```bash
source ~/comp2026_ws/devel/setup.bash
rostopic echo -n 1 /lite3/robot_basic_state
rostopic hz /leg_odom2
rostopic hz /imu/data
```

确保急停可用后，谨慎测试：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'ensure_stand'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'stop'"
```

通过标准：能收到机器人状态、腿部里程计和 IMU，机器狗可响应起立与停止命令。

检查完成后用 `Ctrl+C` 停止该 launch。

## 5. D435i 和 `/scan`

终端 1：

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
```

终端 2：

```bash
source ~/comp2026_ws/devel/setup.bash
roslaunch allmovebase depth2laser.launch
```

终端 3：

```bash
source ~/comp2026_ws/devel/setup.bash
rostopic hz /camera/depth/image_rect_raw
rostopic hz /scan
rostopic echo -n 1 /scan
```

通过标准：深度图和 `/scan` 持续发布，`ranges` 中有有效距离。

此步默认关闭彩色流，没有彩色图像属于正常现象。检查完成后停止两个 launch。

## 6. 地图、TF、AMCL 和 move_base

启动导航链路，但不发送任务目标：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch run_obstacle_task:=false
```

另开终端检查：

```bash
source ~/comp2026_ws/devel/setup.bash
rostopic echo -n 1 /map
rostopic echo -n 1 /amcl_pose
rostopic echo -n 1 /move_base/status
rosrun tf tf_echo map base_link
```

使用虚拟机 RViz 时，必须在启动机器狗 launch 前执行 `ros_master`。虚拟机执行：

```bash
ros_robot
roslaunch dog_dev_rviz dog_comp2026_nav_remote_rviz.launch
```

在 RViz 中使用 `2D Pose Estimate` 设置正确初始位姿。首次检查时不要点击 `2D Nav Goal`。

通过标准：地图、`/scan`、AMCL 位姿和 TF 都正常，RViz 中扫描与地图物理障碍基本重合。

## 7. 录入真实任务位姿

当前 `task_poses.yaml` 中巡检点和避障点是示例值，抓放点仍为零点占位。未录入真实位姿前，不要启动自动导航任务。

保持第 6 步导航链路运行，先备份：

```bash
cp ~/comp2026_ws/src/allmovebase/config/task_poses.yaml \
   ~/comp2026_ws/src/allmovebase/config/task_poses.yaml.bak
```

确认 AMCL 已收敛后运行：

```bash
cd ~/comp2026_ws/src/tools
python3 record_task_pose.py
```

按实际场地依次录入：

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

通过标准：所有位姿已写入 `src/allmovebase/config/task_poses.yaml`，抓放点不再是 `[0,0,0]`。

## 8. 避障任务

注意：以下 launch 会使机器狗起立并自动移动。

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

通过标准：机器狗按 `obs_start -> obs_end` 完成导航，任务报告成功，过程中无长时间定位丢失或代价地图堵死。

## 9. Docker 仪表识别

先确保第 8 步的导航 launch 已完全停止。

终端 1：

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
```

终端 2：

```bash
source ~/comp2026_ws/devel/setup.bash
roslaunch dog_motion meter_reader_docker_persistent.launch start_camera:=false
```

终端 3：

```bash
source ~/comp2026_ws/devel/setup.bash
rostopic echo -n 1 /meter_inspection_ready
rostopic pub -1 /meter_inspect_trigger std_msgs/String "data: 'rec_pose_1'"
rostopic echo /meter_status
```

通过标准：`/meter_inspection_ready` 为 true，手动触发后能收到类似 `rec_pose_1,A,normal` 的结果，并能正常播放语音。

## 10. 巡检导航与识别

先停止第 9 步启动的彩色相机和 Docker 节点。

注意：以下 launch 会使机器狗自动导航到四个巡检点。

```bash
roslaunch allmovebase task_2026_navigation.launch
```

如果彩色流动态开关不稳定，改用：

```bash
roslaunch allmovebase task_2026_navigation.launch \
  camera_enable_color:=true \
  manage_color_stream:=false \
  camera_color_fps:=5
```

观察：

```bash
rostopic echo /meter_status
rostopic echo /meter_state_json
rostopic echo /inspect_report
```

通过标准：机器狗按 `rec_pose_1 -> rec_pose_2 -> rec_pose_4 -> rec_pose_3` 完成巡检，A/B/C/D 状态齐全，`meter_state.yaml` 中 `complete: true`。

## 11. 机械臂抓放联调

当前仓库只包含机器狗侧协议适配，不包含机械臂 ROS2 执行程序或模拟节点。满足以下条件后才能执行：

```text
1. meter_state.yaml 中 complete: true，且 A/B/C/D 状态齐全
2. pickup_pose 和 place_pose_A/B/C/D 均为真实位姿
3. 机械臂端能订阅 /dog_arm/task_cmd
4. 机械臂端能发布 /dog_arm/task_result
```

正式联调：

```bash
roslaunch allmovebase task_2026_pick_place.launch arm_command_required:=true
```

通过标准：机器狗到达抓取点和对应异常区域，机械臂的 `pick_success` 和 `place_success` 能被狗端正确接收。

## 12. 全任务

只有第 4 至 11 步都已分别通过时，才运行全任务。

```bash
roslaunch allmovebase task_2026_full.launch arm_command_required:=true
```

观察：

```bash
rostopic echo /full_task/report
rostopic echo /full_task/succeeded
rostopic echo /meter_status
rostopic echo /dog_arm/task_cmd
rostopic echo /dog_arm/task_result
```

通过标准：在 300 秒预算内完成避障、四点巡检、异常区域判断、抓取和放置，`/full_task/succeeded` 为 true。

## 13. 录包与离线复盘

任务测试时可另开终端轻量录包：

```bash
cd ~/comp2026_ws/src/tools
python3 record_rosbag.py --profile state --split --split-size 2048
```

需要图像时再短时使用：

```bash
python3 record_rosbag.py --profile full --split --split-size 4096
```

`full` 包含 RGB、深度图和代价地图，体积增长很快。

离线回放必须使用独立的本地 ROS Master，不要连接正在运行的机器狗主链路：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/<bag_name>.bag
```

回放完成后恢复：

```bash
rosparam set use_sim_time false
```

## 14. 最终顺序

```text
1. Git 和 catkin 检查
2. 依赖安装与 catkin_make
3. preflight.py --skip-hz
4. 运动主机通信
5. D435i 和 /scan
6. 地图、TF、AMCL 和 move_base
7. 录入真实任务位姿
8. 避障任务
9. Docker 仪表识别
10. 四点巡检识别
11. 机械臂协议与抓放
12. 全任务
13. rosbag 离线复盘
```

任一步未通过时，先停止该步骤，保留终端日志和话题输出，不要直接进入下一阶段。
