# 启动链路说明

本文档说明 `allmovebase/launch` 下当前推荐使用的启动入口。当前工程已经弃用旧的预选赛兼容识别链路，识别接口统一为：

- `/meter_inspect_trigger`：识别触发，消息类型 `std_msgs/String`
- `/meter_status`：识别结果，消息类型 `std_msgs/String`，格式如 `rec_pose_1,A,normal`
- `/meter_state_json`：已记忆的 A/B/C/D 区域状态快照

YOLO 推理只通过 Docker 镜像 `yolo11` 运行。D435i 只允许一个 `realsense2_camera` 进程占用。

## 总领任务入口

### `task_2026_obstacle_test.launch`

避障区域单段测试入口。

- 启动 `message_transformer`
- 启动导航栈：相机、TF、里程计、depth2laser、map server、AMCL、move_base
- 启动 `node_obstacle_zone_task.launch`
- 默认只开深度流，适合低负载避障调试

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

### `task_2026_navigation.launch`

正式导航识别入口，也是当前唯一的导航识别链路。

- 启动 `message_transformer`
- 启动导航栈
- 启动常驻 Docker 仪表识别节点 `meter_persistent_docker_inspection_node.py`
- 启动语音播报和状态记忆
- 启动 `node_sequential_nav_inspect.launch`
- 平时只开深度流，到识别位点后按需打开彩色流，采集 5 张图并交给常驻 Docker 推理
- 到识别位点后会先停稳、切原地姿态模式、抬头约 30 度，再触发识别；识别后恢复俯仰并回到移动模式

```bash
roslaunch allmovebase task_2026_navigation.launch
```

如果 RealSense 实机不适合运行时开关彩色流，可以常开低帧率彩色流，并关闭动态开关：

```bash
roslaunch allmovebase task_2026_navigation.launch camera_enable_color:=true manage_color_stream:=false camera_color_fps:=5
```

识别前姿态动作默认调用语义封装：

```text
inspection_view_pose
```

该命令由 `message_transformer/scripts/lite3_motion_cmd.py` 封装，内部会停稳、切原地姿态模式，并按 `inspection_pitch_value` 调整俯仰。去年资料中俯仰值域约为 `[-6553, 6553]`，正值低头，因此默认 `inspection_pitch_value=-6553` 近似抬头 30 度。实机需要微调时建议改运动封装参数：

```bash
roslaunch allmovebase task_2026_navigation.launch inspection_pitch_value:=-3000
```

模型默认位于容器内：

```text
/workspace/models/yuyin.engine
```

对应宿主机路径：

```text
comp2026_ws/src/dog_motion/models/yuyin.engine
```

模型文件通常被 ignore，不会随本地工程同步；实机部署时请确认该文件存在。

### `task_2026_hardcoded_motion.launch`

硬编码备用路线入口，不依赖 move_base 导航。

- 启动 `message_transformer`
- 启动 meter-only 彩色相机
- 启动常驻 Docker 仪表识别节点
- 启动 `hardcoded_motion.py`
- 在硬编码路线中的识别点发布 `/meter_inspect_trigger`，等待 `/meter_status`

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

只测试硬编码运动、不做识别：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch run_meter_inspection:=false
```

当前硬编码路线假设机器人在避障终点已经朝向地图 `+Y` 方向：

```text
obs_end
  -> 直走到 rec_pose_1，右转 90 度进入识别姿态，识别后左转恢复
  -> 直走到 rec_pose_2，右转 90 度进入识别姿态，识别后左转恢复
  -> 从 rec_pose_2 离开后直走 0.45m、右转、直走 2m、右转、再直走 1m，完成从 +Y 到 -Y 的小半圈换道
  -> 到 rec_pose_4，右转 90 度进入识别姿态，识别后左转恢复
  -> 直走到 rec_pose_3，右转 90 度进入识别姿态，识别后左转恢复
```

实机调参时优先改 launch 参数，不需要改 Python：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch \
  linear_speed:=0.5 \
  turn_speed:=0.5 \
  turn_angle_deg:=90.0 \
  turn_duration_scale:=1.5 \
  obs_end_to_rec_pose_1_distance:=1.25 \
  rec_pose_1_to_rec_pose_2_distance:=2.5 \
  half_loop_leg_1_distance:=0.45 \
  half_loop_leg_2_distance:=2.0 \
  half_loop_leg_3_distance:=1.0 \
  rec_pose_4_to_rec_pose_3_distance:=2.5
```

闭环硬编码测试可加：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch \
  run_meter_inspection:=false \
  closed_loop_motion:=true
```

闭环默认使用 `/leg_odom2`，由 `message_transformer/qnx2ros` 回传。`closed_loop_motion:=false` 时仍使用原来的按距离/角度换算时间的开环 `/cmd_vel`。如实机负载较高，可只开转向闭环：`closed_loop_motion:=true closed_loop_straight:=false closed_loop_turn:=true`。

默认速度为 `0.5m/s`，因此当前默认时长近似对应：`obs_end -> rec_pose_1` 为 1.25m，`rec_pose_1 -> rec_pose_2` 为 2.5m，小半圈三段为 0.45m、2m、1m，`rec_pose_4 -> rec_pose_3` 为 2.5m。

如果避障终点出来时不是朝向 `+Y`，可用 `initial_turn_to_y_pos:=left/right` 先转到路线方向。若某个识别点的仪表不在运动方向右侧，可单独改 `rec_pose_1_inspect_turn`、`rec_pose_2_inspect_turn`、`rec_pose_4_inspect_turn`、`rec_pose_3_inspect_turn` 为 `left` 或 `none`。

## 任务位姿文件

导航识别点、避障起终点和后续任务位姿统一写在：

```text
comp2026_ws/src/allmovebase/config/task_poses.yaml
```

当前预留：

- `rec_pose_1`、`rec_pose_2`、`rec_pose_3`、`rec_pose_4`：四个识别姿态位点
- `obs_start`、`obs_end`：避障段起点和终点
- `task_pose_1`、`task_pose_2`：后续任务占位

识别导航默认读取：

```yaml
recognition: [rec_pose_1, rec_pose_2, rec_pose_3, rec_pose_4]
```

避障测试默认读取：

```yaml
obstacle_test: [obs_start, obs_end]
```

## Docker 仪表识别流程

默认使用 `dog_motion/launch/meter_reader_docker_persistent.launch`。该链路在 launch 启动时只启动一次 Docker 容器，并在容器内只加载一次 YOLO 模型；四个识别点复用同一个容器。

流程：

1. 启动 ROS 节点时，自动把 `meter_persistent_infer.py` 复制到 `comp2026_ws/src/dog_motion/docker_tools`
2. 启动一个常驻 Docker 容器
3. 容器加载 `/workspace/models/yuyin.engine`，输出 `READY`
4. ROS 节点订阅 `/camera/color/image_raw`，等待 `/meter_inspect_trigger`
5. 每个识别点触发后，预热若干帧
6. 保存 5 张图片到 `comp2026_ws/src/dog_motion/runtime/meter_samples/...`
7. ROS 节点通过 stdin 把图片目录发送给常驻容器
8. 容器对该目录内图片投票并输出 `RESULT:rec_pose_1,A,normal`
9. ROS 节点发布 `/meter_status`
10. `meter_audio_node.py` 播放语音
11. `meter_state_store_node.py` 启动时清空上一轮状态，随后写入 `/meter_states`、`/meter_state_json` 和 `config/meter_state.yaml`

当前 `meter_batch_infer.py` 不做裁剪，直接对采样的整张彩色图推理。

常驻容器的 Docker 形式等价于：

```bash
docker run --rm -i \
  --runtime=nvidia \
  --privileged \
  --network host \
  -v ~/comp2026_ws/src/dog_motion:/workspace \
  -v /dev:/dev \
  -w /workspace \
  --entrypoint /bin/bash \
  yolo11 \
  -lc "python3 -u /workspace/docker_tools/meter_persistent_infer.py --model /workspace/models/yuyin.engine --min-confidence 0.25"
```

默认 `state_clear_on_start:=true`，用于防止上一轮 `meter_state.yaml` 被抓放状态机误用。排障时如果确实要复用历史识别结果，可显式传入 `state_clear_on_start:=false`。

备用链路仍保留：

```bash
roslaunch dog_motion meter_reader_docker_on_demand.launch
```

该备用链路会每次触发都重新 `docker run --rm` 并重新加载模型，启动慢但隔离性更强，适合排查常驻容器异常。

手动进入调试环境：

```bash
docker run -it --rm \
  --runtime=nvidia \
  --privileged \
  --network host \
  -v ~/comp2026_ws/src/dog_motion:/workspace \
  -v /dev:/dev \
  -w /workspace \
  --entrypoint /bin/bash \
  yolo11
```

## 局部组件入口

这些 launch 通常由总领任务包含，不建议作为比赛入口直接启动：

- `stack_nav_base.launch`
- `node_obstacle_zone_task.launch`
- `node_sequential_nav_inspect.launch`
- `camera.launch`
- `camera_nav_only.launch`
- `camera_nav_with_meter.launch`
- `camera_meter_only.launch`
- `amcl.launch`
- `map_server.launch`
- `movebase.launch`
- `depth2laser.launch`
- `odom.launch`
- `camera2base_tf.launch`

## 注意事项

- 不要同时启动两个 `realsense2_camera` 进程占用同一台 D435i。
- 默认导航入口按需打开彩色流；如果实机动态开关不稳定，使用 `camera_enable_color:=true manage_color_stream:=false`。
- 导航、AMCL 和 depth-to-scan 默认按 D435i 深度 3.0m 内较可靠范围使用。RGB 与 depth 默认不强制对齐；需要 RGB 检测框取深度时，可传 `camera_align_depth:=true` 并改用 RealSense 的 aligned depth topic。
- 当前工程默认使用 `docker` 启动识别容器，适合当前用户已加入 docker 组的机器狗。若 `docker ps` 提示权限不足，重新登录当前用户会话，或临时传入 `docker_command:="sudo docker"`。
- 当前工程不再维护旧的预选赛兼容识别接口。如需查旧实现，只看去年的 `robot_motion` 工程，不在 `comp2026_ws` 中继续维护。

## 抓取放置主链路

### `task_2026_pick_place.launch`

后续机械臂抓取任务的机器狗端主链路入口，和避障测试、导航识别、硬编码路线并列。

```bash
roslaunch allmovebase task_2026_pick_place.launch
```

当前流程：

1. 启动 `message_transformer`
2. 启动导航栈
3. 读取 `config/meter_state.yaml` 中已经记忆的 A/B/C/D 状态
4. 按 `region_order:=A,B,C,D` 找到前两个非 `normal` 区域
5. 导航到 `pickup_pose`
6. 向 `/dog_arm/task_cmd` 发布机械臂协议抓取命令，默认 `pick`
7. 导航到异常区域对应放置点，例如 `place_pose_A`
8. 向 `/dog_arm/task_cmd` 发布机械臂协议放置命令，默认 `place_to_zone`
9. 回到 `pickup_pose` 执行第二次抓取，再去第二个异常区域对应放置点放置

位姿复用：

```text
comp2026_ws/src/allmovebase/config/task_poses.yaml
```

新增占位：

- `pickup_pose`
- `place_pose_A`
- `place_pose_B`
- `place_pose_C`
- `place_pose_D`

机械臂通讯：

- 发布话题：`/dog_arm/task_cmd`
- 结果话题：`/dog_arm/task_result`
- 底盘微调请求：`/dog_arm/base_adjust_req`
- 抓取命令：`pick`
- 放置命令：`place_to_zone`
- `arm_command_required`：默认 `false`，表示没有机械臂订阅者时也继续发布并等待结果；两端联调稳定后建议改为 `true`。
- `pick_failed + need_base_adjust` 会等待 `/dog_arm/base_adjust_event` 后重试一次抓取；底盘实际微调默认关闭。

常用调参：

```bash
roslaunch allmovebase task_2026_pick_place.launch \
  region_order:=A,B,C,D \
  max_abnormal_count:=2 \
  arm_wait:=5.0
```

## 全任务主链路

### `task_2026_full.launch`

全任务链路入口，用于从站立准备开始，顺序执行避障、四点识别、按异常区域抓取放置。它不会替代单段调试入口；避障测试、导航识别、硬编码路线、抓取放置仍然保留。

```bash
roslaunch allmovebase task_2026_full.launch
```

默认流程：

1. 启动 `message_transformer`
2. 启动导航栈、相机、深度转扫描、地图、AMCL、move_base
3. 启动常驻 Docker 识别、语音播报、状态记忆
4. 发送 `prepare_navigation`，进行站立自检并切入自主速度控制模式
5. 按 `obstacle_test` 序列导航通过 `obs_start -> obs_end`
6. 按 `recognition` 序列导航至 `rec_pose_1 -> rec_pose_2 -> rec_pose_4 -> rec_pose_3`
7. 每个识别点执行 `inspection_view_pose`，打开彩色流，触发 `/meter_inspect_trigger`，等待 `/meter_status`
8. 根据识别得到的 A/B/C/D 状态，挑选前两个非 `normal` 区域
9. 执行 `pickup_pose -> place_pose_X -> pickup_pose -> place_pose_Y`
10. 通过 `/dog_arm/task_cmd` 发布机械臂协议命令，并等待 `/dog_arm/task_result`

常用调试参数：

```bash
roslaunch allmovebase task_2026_full.launch run_pick_place:=false
roslaunch allmovebase task_2026_full.launch run_obstacle:=false
roslaunch allmovebase task_2026_full.launch run_inspection:=false run_pick_place:=true
```

完整链路的位姿仍统一读取：

```text
comp2026_ws/src/allmovebase/config/task_poses.yaml
```

相机 TF 现在只读取 YAML 文件：

```text
comp2026_ws/src/allmovebase/config/camera2base_tf.yaml
```

`camera2base_tf.launch` 会启动 `camera_static_tf_from_yaml.py`，读取 `camera_to_base_link_transform` 并发布 `base_link -> camera_link`。实机调整相机位姿时，直接改 `translation` 和 `rotation` 即可，不需要再改 launch。

