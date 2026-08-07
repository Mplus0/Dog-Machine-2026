# 2026 高校智能机器人大赛四足大型组机器狗端代码状态说明

本文档基于当前 `comp2026_ws` 工作空间、仓库内已有说明文档，以及比赛要求 PDF 进行整理。当前环境不是机器狗实机环境，未运行正式 ROS 程序；以下结论来自静态阅读与文件结构分析。

## 1. 比赛要求摘要

国赛线下挑战任务分为三部分：

1. 避障任务：四足机器人需要从出发区域自主通过障碍区域。障碍区域包含橡胶路沿坡，用于模拟崎岖障碍路面。
2. 巡检识别：检测区放置配电柜和变压器，侧面张贴 A、B、C、D 区域字母及仪表盘。仪表盘状态分为 `偏低/异常`、`正常`、`偏高/异常`。机器人需要识别四个区域的字母和仪表盘状态，并语音播报。
3. 长条抓取：抓取区有高台，放置红色和绿色长条。红色代表异常，绿色代表正常。机器人需要根据巡检识别结果，抓取红色长条并放到对应异常区域 A/B/C/D。

重要约束：

- 全过程需要自主完成。
- 机器狗控制方式需要使用 VR 设备或代码控制。
- 当前用户说明中，本工作空间只包含机器狗端代码，不包含机械臂端实现。
- 本次分析只关注国赛要求，不展开预选赛视频提交链路。

## 2. 工作空间概览

当前目录是一个 ROS/catkin 工作空间，顶层包含：

- `src/`：源码包。
- `build/`、`devel/`：已有构建产物。
- `README_TEST_GUIDE.md`、`README_NETWORK_TOPOLOGY.md`、`README_PHYSICAL_CONFIG.md`、`README_FIELD_TEST_CHECKLIST.md`：已有实机测试、网络、物理配置、现场检查文档。

`src/` 下主要 ROS 包如下：

| 包名 | 作用 | 完成度判断 |
| --- | --- | --- |
| `allmovebase` | 比赛主程序包，包含导航、避障、巡检、抓放和全任务状态机 | 重点关注 |
| `message_transformer` | Lite3 运动主机 UDP/ROS 桥接，封装 `/cmd_vel`、`/simple_cmd`、运动状态回传 | 平台封装包，只需确认功能 |
| `dog_motion` | Docker YOLO 仪表识别、语音播报、识别状态记忆 | 比赛识别链路关键包 |
| `dog_arm_bridge` | 机器狗 ROS1 到机械臂协议的话题适配 | 狗端协议已接入，机械臂端不在本工作空间 |
| `depthimage_to_laserscan` | 深度图转 LaserScan | ROS 功能包/封装包 |
| `lite3_description` | Lite3 URDF、mesh、RViz 检查 | 机器人模型资源 |
| `tools` | 上机检查、录点、录包、相机标定、网页调试等辅助工具 | 调试工具，不是主状态机 |

当前 `git status --short` 显示工作区已有改动：

```text
D src/CMakeLists.txt
 M src/dog_arm_bridge/scripts/dog_arm_bridge_node.py
 M src/message_transformer/launch/message_transformer.launch
```

这些改动不是本文档生成过程产生的，后续提交或回退前应先确认来源。

## 3. 主程序入口

比赛相关主入口集中在 `src/allmovebase/launch/`：

| launch | 用途 |
| --- | --- |
| `task_2026_full.launch` | 全任务入口：底层准备 -> 避障 -> 四点巡检识别 -> 根据异常区域抓取放置 |
| `task_2026_obstacle_test.launch` | 单独测试避障区域导航 |
| `task_2026_navigation.launch` | 单独测试四点巡检识别 |
| `task_2026_pick_place.launch` | 单独测试读取识别结果后的抓取放置狗端流程 |
| `task_2026_hardcoded_motion.launch` | 不依赖 `move_base` 的硬编码备用路线，用于定位/导航不稳定时备用 |

共享导航栈入口：

```text
src/allmovebase/launch/stack_nav_base.launch
```

它启动：

- D435i 相机。
- `base_link -> camera_link` 静态 TF。
- 里程计处理。
- depth-to-scan。
- 双地图 map server。
- AMCL。
- `move_base`。

## 4. 当前已实现功能

### 4.1 运动主机通讯

已实现。

相关包：

```text
src/message_transformer
```

主要功能：

- `ros2qnx`：订阅 `/cmd_vel`、`/simple_cmd`、`/complex_cmd`，通过 UDP 发给运动主机。
- `qnx2ros`：接收运动主机 UDP 数据，发布 `/leg_odom2`、`/imu/data`、`/lite3/robot_basic_state`、电量、关节状态等。
- `lite3_motion_cmd.py`：封装高层动作命令，例如起立、停止、切换移动模式、切换原地姿态模式、识别视角抬头、恢复导航视角。

默认运动主机地址：

```text
192.168.1.120:43893
```

常用控制话题：

```text
/cmd_vel
/simple_cmd
/lite3_motion_cmd
```

### 4.2 导航与避障

已实现主链路，但实机效果依赖地图、相机外参、AMCL 和场地位姿。

相关文件：

```text
src/allmovebase/launch/stack_nav_base.launch
src/allmovebase/launch/task_2026_obstacle_test.launch
src/allmovebase/scripts/obstacle_zone_task.py
src/allmovebase/map/map.yaml
src/allmovebase/map/map_amcl.yaml
src/allmovebase/config/task_poses.yaml
```

实现内容：

- 使用 D435i 深度图转 `/scan`。
- 使用 `map.yaml` 作为导航地图，包含物理障碍和二维禁跨线。
- 使用 `map_amcl.yaml` 作为 AMCL 定位地图，只包含可观测物理障碍。
- 使用 AMCL 定位。
- 使用 `move_base`，全局规划器为 `navfn/NavfnROS`，局部规划器为 `teb_local_planner/TebLocalPlannerROS`。
- `obstacle_zone_task.py` 会按 `task_poses.yaml` 中 `obstacle_test: [obs_start, obs_end]` 顺序发送导航目标。

当前风险：

- `task_poses.yaml` 中位姿是示例/占位性质，需要在真实赛场重新录点。
- 避障能否稳定通过橡胶路沿坡，必须在实机上验证步态、速度、障碍检测范围和局部规划参数。

### 4.3 巡检识别与播报

已实现主链路。

相关文件：

```text
src/allmovebase/launch/task_2026_navigation.launch
src/allmovebase/scripts/sequential_nav_inspect.py
src/dog_motion/launch/meter_reader_docker_persistent.launch
src/dog_motion/scripts/meter_persistent_docker_inspection_node.py
src/dog_motion/scripts/meter_persistent_infer.py
src/dog_motion/scripts/meter_audio_node.py
src/dog_motion/scripts/meter_state_store_node.py
```

实现流程：

1. 导航到 `rec_pose_1`、`rec_pose_2`、`rec_pose_4`、`rec_pose_3`。
2. 停稳，发送 `inspection_view_pose`，切到识别视角。
3. 按需打开 D435i 彩色流。
4. 向 `/meter_inspect_trigger` 发布当前位置名称。
5. 常驻 Docker 容器采集 5 张彩色图。
6. 容器内 YOLO 模型识别区域字母 `A/B/C/D` 和仪表状态 `low/normal/high`。
7. 输出 `/meter_status`，格式如：

```text
rec_pose_1,A,normal
```

8. `meter_audio_node.py` 根据结果播放语音。
9. `meter_state_store_node.py` 记忆识别状态，写入：

```text
/meter_state_json
/meter_states
/meter_states_ready
src/allmovebase/config/meter_state.yaml
```

当前风险：

- 依赖 Docker 镜像 `yolo11`。
- 依赖模型文件：

```text
src/dog_motion/models/yuyin.engine
```

- 该模型文件虽然当前目录中存在，但实际可用性需要在 Jetson/TensorRT 环境验证。
- 当前推理逻辑直接对整张图检测，不做仪表盘/字母区域裁剪；如果赛场背景复杂，误检风险较高。
- PDF 要求识别四个区域且有 2 正常、2 异常；当前程序会记录任意识别到的状态，但没有显式校验“必须正好两个异常”。

### 4.4 识别状态记忆

已实现。

`meter_state_store_node.py` 会把识别结果汇总成 A/B/C/D 状态表。例如：

```yaml
states:
  A: low
  B: normal
  C: high
  D: normal
complete: true
expected_regions:
  - A
  - B
  - C
  - D
```

抓放流程会读取这些状态，并选择前两个非 `normal` 区域作为异常目标。

### 4.5 抓取放置狗端流程

狗端状态机已实现，机械臂端执行不在当前工作空间内。

相关文件：

```text
src/allmovebase/launch/task_2026_pick_place.launch
src/allmovebase/scripts/pick_place_task.py
src/allmovebase/scripts/full_task.py
src/allmovebase/scripts/dog_arm_task_client.py
src/dog_arm_bridge
```

已实现内容：

- 从 `meter_state.yaml` 或运行时识别结果中找出异常区域。
- 默认最多处理 2 个异常区域。
- 导航到 `pickup_pose`。
- 向 `/dog_arm/task_cmd` 发布抓取命令：

```json
{"task_id":"...","cmd":"pick"}
```

- 等待 `/dog_arm/task_result`。
- 导航到异常区域对应放置点，例如 `place_pose_A`。
- 向机械臂发布放置命令：

```json
{"task_id":"...","cmd":"place_to_zone"}
```

- 支持机械臂请求底盘左右微调的话题协议：

```text
/dog_arm/base_adjust_req
/dog_arm/base_adjust_event
```

当前明显未完成点：

- `pickup_pose`、`place_pose_A/B/C/D` 在 `task_poses.yaml` 中仍是 `[0,0,0]` 占位，无法直接用于比赛。
- `dog_arm_bridge` 只是狗端协议适配，机械臂 ROS2 侧、抓取视觉、实际夹爪控制不在此工作空间。
- 默认 `arm_command_required=false`，表示没有机械臂订阅者时不一定立即失败；正式联调建议改成 `true`，避免“假成功/空等”。
- 底盘微调默认 `enable_base_adjust_execution=false`，不会真的移动底盘，只记录和转发事件。

### 4.6 全任务状态机

已实现总体框架。

入口：

```bash
roslaunch allmovebase task_2026_full.launch
```

默认流程：

1. 启动 `message_transformer`。
2. 启动导航栈。
3. 启动 `dog_arm_bridge`。
4. 启动常驻 Docker 识别、语音播报、状态记忆。
5. 发送 `prepare_navigation`，准备运动主机。
6. 执行避障段。
7. 执行四点巡检识别。
8. 找出异常区域。
9. 执行两轮 `pickup -> place_X`。
10. 发布 `/full_task/succeeded` 和 `/full_task/report`。

当前状态：

- 代码框架完整。
- 依赖实机导航、识别模型、机械臂端响应和真实位姿。
- 不能在当前非机器狗环境完整运行验证。

### 4.7 硬编码备用路线

已实现。

入口：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

用途：

- 不使用 `move_base`。
- 按距离和转角参数发布 `/cmd_vel`。
- 可在每个识别点触发仪表识别。
- 支持可选闭环，默认 `/leg_odom2`。

当前风险：

- 只覆盖避障终点到四个巡检点的固定路线。
- 不包含避障前段和抓放完整闭环。
- 对起点朝向、地面摩擦、速度标定非常敏感，需要实机调参。

## 5. 当前未完成或必须实机补齐的内容

按国赛需求排序：

1. 真实场地位姿未录入。
   - `rec_pose_1/2/3/4` 需要按赛场地图重录。
   - `obs_start/obs_end` 需要按真实避障区域重录。
   - `pickup_pose`、`place_pose_A/B/C/D` 仍是零点占位，必须补齐。

2. 机械臂端不在当前工作空间。
   - 当前只实现狗端 `/dog_arm/task_cmd`、`/dog_arm/task_result` 协议。
   - 真正抓取红色长条、判断抓取成功、放置到对应区域，需要机械臂端实现和联调。

3. 抓取任务缺少狗端对“红色长条”的直接感知。
   - 当前狗端不会识别高台上红/绿长条。
   - 逻辑是假设机械臂端执行 `pick` 时能抓取正确红色长条，或抓取区设计/机械臂策略能保证抓红色。

4. 巡检识别模型需要实机验证。
   - 当前识别类别应包含 `A/B/C/D` 和 `low/normal/high`。
   - 需要确认模型在赛场光照、距离、视角下可稳定识别。
   - 建议补充离线测试集和混淆矩阵，技术文档中也需要体现。

5. 语音播报格式需要现场确认。
   - 程序会按音频片段播报区域和状态。
   - 比赛要求示例是中文播报“某区域仪表盘显示偏低/偏高/正常，状态异常/正常”。
   - 需要实机听音量、清晰度、播报内容是否符合裁判要求。

6. 避障能力需要实机验证。
   - 当前依赖 D435i 深度转 `/scan` 和 `move_base`。
   - 橡胶路沿坡属于地形障碍，不一定能完全靠二维 `/scan` 表达。
   - 需要验证机器狗步态、速度、相机安装角度、代价地图膨胀半径。

7. 全任务 5 分钟预算需要实测。
   - `task_2026_full.launch` 默认启用 300 秒预算。
   - 识别等待、机械臂抓取、放置动作超时时间偏保守，实机可能超时。

8. 当前工作区有已有未提交改动。
   - 后续比赛代码定版前，应先确认这些改动是否需要保留。

## 6. 如何复现当前已实现功能

以下命令应在机器狗感知主机上执行，当前普通电脑环境无法完整运行。

### 6.1 基础编译

```bash
cd ~/comp2026_ws
catkin_make
source devel/setup.bash
```

每个新终端都需要：

```bash
source ~/comp2026_ws/devel/setup.bash
```

### 6.2 检查运动主机通讯

启动底层桥：

```bash
roslaunch message_transformer message_transformer.launch
```

检查运动主机网络：

```bash
ping 192.168.1.120
```

测试站立准备：

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

### 6.3 单独测试 D435i 和 `/scan`

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
```

另开终端：

```bash
roslaunch allmovebase depth2laser.launch
rostopic hz /scan
rostopic echo -n 1 /scan
```

### 6.4 单独测试避障导航

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

关键观察：

```bash
rostopic echo /move_base/status
rostopic echo /obstacle_task/report
rostopic echo /amcl_pose
rostopic hz /scan
```

### 6.5 单独测试巡检识别

```bash
roslaunch allmovebase task_2026_navigation.launch
```

如果动态开关彩色流不稳定，使用常开彩色低帧率：

```bash
roslaunch allmovebase task_2026_navigation.launch camera_enable_color:=true manage_color_stream:=false camera_color_fps:=5
```

关键观察：

```bash
rostopic echo /meter_inspection_ready
rostopic echo /meter_inspect_trigger
rostopic echo /meter_status
rostopic echo /meter_state_json
rostopic echo /inspect_report
```

### 6.6 单独测试识别 Docker

先启动彩色相机：

```bash
roslaunch allmovebase camera_meter_only.launch color_fps:=5
```

另开终端启动识别：

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

### 6.7 单独测试抓放狗端流程

前提：

- `src/allmovebase/config/meter_state.yaml` 中已有 A/B/C/D 状态。
- `pickup_pose` 和 `place_pose_A/B/C/D` 已经录入真实位姿。
- 机械臂端或模拟节点会订阅 `/dog_arm/task_cmd` 并发布 `/dog_arm/task_result`。

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

### 6.8 全任务运行

```bash
roslaunch allmovebase task_2026_full.launch
```

常用调试裁剪：

```bash
roslaunch allmovebase task_2026_full.launch run_pick_place:=false
roslaunch allmovebase task_2026_full.launch run_obstacle:=false
roslaunch allmovebase task_2026_full.launch run_inspection:=false run_pick_place:=true
```

观察：

```bash
rostopic echo /full_task/report
rostopic echo /full_task/succeeded
rostopic echo /meter_status
rostopic echo /dog_arm/task_cmd
```

## 7. 推荐后续开发顺序

1. 先确认代码能在机器狗上 `catkin_make` 并正常 source。
2. 只启动 `message_transformer`，确认起立、停止、`/cmd_vel` 到运动主机的链路。
3. 单独验证 D435i 深度、`/scan`、TF、AMCL。
4. 录入真实 `obs_start/obs_end`，跑通避障段。
5. 录入真实四个巡检点，跑通 `task_2026_navigation.launch`。
6. 用实拍图验证 `yuyin.engine` 对 A/B/C/D 与 `low/normal/high` 的准确率。
7. 补齐 `pickup_pose`、`place_pose_A/B/C/D`。
8. 与机械臂端联调 `/dog_arm/task_cmd` 和 `/dog_arm/task_result`。
9. 把 `arm_command_required` 改为 `true` 进行正式联调。
10. 最后跑 `task_2026_full.launch`，按 5 分钟总时长优化超时参数、速度和等待时间。

## 8. 结论
当前工作空间已经具备国赛机器狗端的主框架：底层运动桥、导航避障、巡检识别、状态记忆、语音播报、狗端抓放协议、全任务状态机和备用硬编码路线都已经存在。

但它还不是可以直接上国赛的完整闭环。最关键的缺口是：真实赛场位姿未录入，抓取/放置位姿仍是占位，机械臂端实现缺失，识别模型与避障效果未在实机场景验证。下一阶段应围绕实机录点、识别准确率、机械臂联调和全流程 5 分钟内稳定完成来推进。
