# dog_arm_bridge

`dog_arm_bridge` 是机器狗 ROS1 侧的机械臂协议与 TCP 传输包。RDK X5 机械臂运行 ROS2 Humble，两端通过一条经过 HMAC 认证的全双工 TCP 连接通信，不依赖 `ros1_bridge`。

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

首次启动前，两台设备必须各自创建内容完全相同的密钥文件：

```bash
mkdir -p ~/.ros
umask 077
openssl rand -hex 32 > ~/.ros/dog_arm_shared_secret
chmod 600 ~/.ros/dog_arm_shared_secret
```

只在其中一台设备生成，然后通过可信方式复制到另一台设备；不要提交到 Git，也不要通过 ROS 话题发送。默认网络参数为：

```text
机械臂服务端：192.168.31.56:47001
允许的机器狗：192.168.31.192
```

TCP 的客户端/服务端只表示建连方向。连接建立后，机器狗和机械臂都能在同一连接中主动发送和接收消息。

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

- `dog_arm_tcp_client_node.py` 负责连接机械臂、双向收发、心跳、自动重连、确认重发和重复消息抑制。
- `/dog_arm/transport_connected` 为锁存的链路状态；只有完成 IP 检查和 HMAC 双向认证后才为 `true`。
- `/dog_arm/transport_status` 提供 JSON 格式的连接诊断信息。
- 机械臂结果在 `pick_place_task.py` 和 `full_task.py` 中已经接入等待逻辑。
- `pick_failed + need_base_adjust` 只会在收到 `executed=true` 的微调完成事件后重试。
- 底盘微调默认不执行，避免在机械臂协议未闭环前产生意外运动。
- TCP 连接提供 HMAC 双向身份认证和逐帧完整性校验，但不加密业务内容；仅应在受控比赛局域网中使用。
