# Lite3 识别视角姿态说明

本文档记录识别点“抬头/恢复视角”相关封装。文档编码为 UTF-8。

## 背景

Lite3 官方接口中，`0x21010130` 是复用轴指令：

| 模式 | 含义 |
| --- | --- |
| 原地模式 | 调整身体俯仰，正值低头，负值抬头 |
| 移动模式 | 前后移动轴指令，正值向前 |

因此识别前不能直接裸发 `0x21010130`，必须先停稳并切到原地模式。否则该指令可能被运动主机解释为前后移动。

## 当前封装

代码入口位于 `message_transformer/scripts/lite3_motion_cmd.py`。

| 命令 | 动作流程 |
| --- | --- |
| `inspection_view_pose` | 连续发布零 `/cmd_vel`，发送原地模式，等待状态新鲜，按 `inspection_pitch_value` 重复发送俯仰轴值 |
| `navigation_view_pose` | 发送原地模式，按 `navigation_pitch_value` 恢复俯仰轴值，然后发送移动模式 |

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `inspection_pitch_value` | `-6553` | 识别姿态俯仰值，负值抬头 |
| `navigation_pitch_value` | `0` | 导航姿态恢复值 |
| `view_pose_step_sleep` | `0.5` | 每次俯仰指令之间的间隔 |
| `view_pose_pitch_repeat_count` | `3` | 俯仰指令重复次数 |
| `standing_basic_states` | `6,9` | 认为已站立或姿态调整中的状态 |
| `spot_basic_states` | `6,9` | 原地模式切换后的可接受状态 |
| `move_basic_states` | `6,9` | 移动模式切换后的可接受状态 |

## 实机确认方法

1. 启动底层通讯：

```bash
roslaunch message_transformer message_transformer.launch
```

2. 确认状态回传：

```bash
rostopic echo /lite3/robot_basic_state
rostopic echo /lite3/robot_gait_state
rostopic echo /lite3/robot_motion_state
```

3. 确认站立：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'ensure_stand'"
```

4. 测试识别姿态：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'inspection_view_pose'"
```

5. 测试恢复导航姿态：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'navigation_view_pose'"
```

如果抬头仍无明显效果，建议逐步测试：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'spot_mode'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'pitch -3000'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'pitch -6553'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'pitch 0'"
```

注意观察 `/lite3/robot_basic_state` 是否进入 `9=姿态调整状态`。如果状态始终不变化，说明运动主机可能没有接受原地姿态控制，下一步应结合官方状态三元组和运动主机模式继续排查。

