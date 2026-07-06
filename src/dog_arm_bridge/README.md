# dog_arm_bridge

`dog_arm_bridge` 是机器狗 ROS1 侧的机械臂协议适配包。机械臂侧预计由 RDK X5 运行 ROS2 Humble，并通过 `ros1_bridge` 或后续确定的网络方式与机器狗通信。

当前协议文件：

```text
R:\Temp\机械狗端与机械臂端通信说明(1).txt
```

## 1. 对外协议话题

这些话题需要与机械臂 ROS2 侧互通：

```text
狗端 ROS1  -> /dog_arm/task_cmd        -> 机械臂端 ROS2
狗端 ROS1  <- /dog_arm/task_result     <- 机械臂端 ROS2
狗端 ROS1  <- /dog_arm/base_adjust_req <- 机械臂端 ROS2
```

类型均为：

```text
std_msgs/String
```

`/dog_arm/task_cmd` JSON 示例：

```json
{"task_id": "t001", "cmd": "pick"}
{"task_id": "t002", "cmd": "place_to_zone"}
```

`/dog_arm/task_result` JSON 示例：

```json
{"task_id": "t001", "result": "pick_success"}
{"task_id": "t001", "result": "pick_failed", "error": "need_base_adjust"}
{"task_id": "t002", "result": "place_success"}
{"task_id": "t002", "result": "place_failed", "error": "xxx"}
```

`/dog_arm/base_adjust_req` JSON 示例：

```json
{"task_id": "t001", "direction": "left", "step_m": 0.05, "reason": "target_left"}
{"task_id": "t001", "direction": "right", "step_m": 0.05, "reason": "target_right"}
```

## 2. 本地狗端入口

主任务可以向本地入口发布简单命令，由本包补齐 `task_id` 并转成协议 JSON：

```text
/dog_arm/local_task_cmd
```

示例：

```bash
rostopic pub -1 /dog_arm/local_task_cmd std_msgs/String "data: 'pick'"
rostopic pub -1 /dog_arm/local_task_cmd std_msgs/String "data: 'place_to_zone'"
rostopic pub -1 /dog_arm/local_task_cmd std_msgs/String "data: '{\"task_id\":\"t001\",\"cmd\":\"pick\"}'"
```

本包会发布到：

```text
/dog_arm/task_cmd
```

## 3. 底盘微调请求

机械臂侧通过 `/dog_arm/base_adjust_req` 请求机器狗横向微调。当前包默认只记录并转发事件，不真正驱动底盘：

```text
enable_base_adjust_execution=false
```

收到请求后会发布本地事件：

```text
/dog_arm/base_adjust_event
```

若后续确认实机安全，可打开执行：

```bash
roslaunch dog_arm_bridge dog_arm_bridge.launch enable_base_adjust_execution:=true
```

默认执行模式为 `cmd_vel`，会向 `/cmd_vel` 发布短时横移速度，底层仍复用 `message_transformer` 的 `ros2qnx` 链路。V0 协议只接受 `left/right`，不接受前后微调。也保留 `lite3_motion_cmd` 模式作为后续实验接口，但默认不使用。

## 4. 启动

单独启动：

```bash
roslaunch dog_arm_bridge dog_arm_bridge.launch
```

配合 pick/place 任务时，应先启动 `message_transformer`、导航栈和本包。当前 `pick_place_task.py` 与 `full_task.py` 会直接向 `/dog_arm/task_cmd` 发布协议 JSON，并等待 `/dog_arm/task_result`。`/dog_arm/local_task_cmd` 只保留给手工调试和临时脚本。

独立链路测试：

```bash
roslaunch dog_arm_bridge dog_arm_bridge.launch
rosrun dog_arm_bridge dog_arm_task_cli.py pick --task-id manual_pick_001 --timeout 180
rosrun dog_arm_bridge dog_arm_task_cli.py place_to_zone --task-id manual_place_001 --timeout 60
```

`dog_arm_task_cli.py` 会直接向 `/dog_arm/task_cmd` 发布协议 JSON，并等待同 `task_id` 的 `/dog_arm/task_result`。

## 5. 当前边界

- 网络连接方式尚未最终确定，本包只固定 ROS 话题协议。
- 机械臂结果在 `pick_place_task.py` 和 `full_task.py` 中已经接入等待逻辑。
- `pick_failed + need_base_adjust` 会等待底盘微调事件后重试一次 `pick`，重试次数由任务 launch 参数控制。
- 底盘微调默认不执行，避免在机械臂协议未闭环前产生意外运动。
