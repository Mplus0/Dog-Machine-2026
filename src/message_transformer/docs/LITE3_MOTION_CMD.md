# Lite3 运动命令封装手册

本文档说明当前工程 `message_transformer/scripts/lite3_motion_cmd.py` 暴露给上层任务的命令。文档编码为 UTF-8。

## 节点与话题

节点：

```bash
roslaunch message_transformer message_transformer.launch
```

命令入口：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/lite3_motion_cmd` | `std_msgs/String` | 上层任务向底层发送语义化运动命令 |

底层输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/simple_cmd` | `message_transformer/SimpleCMD` | 简单 UDP 指令，由 `ros2qnx` 转发给运动主机 |
| `/cmd_vel` | `geometry_msgs/Twist` | 连续速度控制，由 `ros2qnx` 转发给运动主机 |

状态输入：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/lite3/robot_basic_state` | `std_msgs/Int32` | 基本运动状态 |
| `/lite3/robot_gait_state` | `std_msgs/Int32` | 当前步态 |
| `/lite3/robot_motion_state` | `std_msgs/Int32` | 当前动作状态 |

## 推荐任务命令

这些命令是上层任务优先使用的接口。

| 命令 | 别名 | 说明 |
| --- | --- | --- |
| `prepare_navigation` | `prepare_nav`, `nav_prepare` | 导航/避障前准备：确认站立、切自主速度控制模式、切默认步态、短暂停稳 |
| `prepare_hardcoded_motion` | `prepare_hardcoded`, `hardcoded_prepare` | 硬编码备用路线前准备，目前等同于 `prepare_navigation` |
| `inspection_view_pose` | 无 | 识别前姿态：停止速度、切原地模式、调整身体俯仰 |
| `navigation_view_pose` | 无 | 识别后恢复：恢复身体俯仰并切回自主速度控制模式 |
| `stop` | `zero_vel`, `zero_velocity` | 连续发布零 `/cmd_vel` |
| `ensure_stand` | `ensure_standing`, `stand_if_needed` | 根据状态自检后起立 |

示例：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'prepare_navigation'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'inspection_view_pose'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'navigation_view_pose'"
```

## 状态与模式命令

| 命令 | 说明 |
| --- | --- |
| `ensure_stand` | 未站立时发送起立/趴下 toggle，并等待站立状态 |
| `stand_lie_toggle_raw` | 裸起立/趴下 toggle，仅建议调试使用 |
| `lie_down_if_standing` | 仅在确认站立时发送趴下 toggle |
| `spot_mode` | 发送原地模式指令 |
| `move_mode` | 发送移动模式指令 |
| `velocity_control_mode` | 先发送自主模式，再发送移动模式；别名 `cmd_vel_mode`、`nav_motion_mode` |
| `auto_mode` | 发送自主模式指令 |
| `manual_mode` | 发送手动模式指令 |
| `zero_position` | 回零 |
| `estop` | 软急停 |

兼容旧命令：

- `stand`、`stand_toggle`、`stand_up` 现在都映射到裸 toggle。
- `lie`、`lie_down`、`sit_down` 映射到 `lie_down_if_standing`。

## 姿态与轴命令

原地模式下：

| 命令 | 指令码 | 说明 |
| --- | --- | --- |
| `pitch <value>` | `0x21010130` | 调整身体俯仰，正值低头，负值抬头 |
| `roll <value>` | `0x21010131` | 调整横滚 |
| `yaw <value>` | `0x21010135` | 调整偏航 |
| `height <value>` | `0x21010102` | 调整身体高度，正值抬高 |

移动模式下：

| 命令 | 指令码 | 说明 |
| --- | --- | --- |
| `forward_axis <value>` | `0x21010130` | 轴指令前后移动 |
| `side_axis <value>` | `0x21010131` | 轴指令左右平移 |
| `turn_axis <value>` | `0x21010135` | 轴指令左右转向 |

兼容旧命令：

- `forward`、`move_forward` 映射到 `forward_axis`。
- `side`、`strafe`、`move_side` 映射到 `side_axis`。
- `turn`、`raw_turn` 映射到 `turn_axis`。

注意：导航和硬编码主流程优先使用 `/cmd_vel`，不要混用轴指令进行连续运动，除非是在做底层单项测试。

## 步态与高度

| 命令 | 指令码 | 说明 |
| --- | --- | --- |
| `flat_low_gait` | `0x21010300` | 平地低速步态 |
| `flat_medium_gait` | `0x21010307` | 平地中速步态 |
| `flat_high_gait` | `0x21010303` | 平地高速步态 |
| `crawl_gait_toggle` | `0x21010406` | 正常/匍匐步态 toggle |
| `grip_obstacle_gait` | `0x21010402` | 抓地越障步态 |
| `general_obstacle_gait` | `0x21010401` | 通用越障步态 |
| `high_step_gait` | `0x21010407` | 高踏步越障步态 |
| `height_low` | `0x21010102` | 发送 `low_pose_height_value` |
| `height_normal` | `0x21010102` | 发送 `normal_height_value` |

`low_pose`、`crawl_pose`、`prone_pose` 保留为兼容命令，内部会执行 `height_low` 和 `crawl_gait_toggle`。实机调试时建议先分别测试高度和匍匐步态。

## 动作、语音、扬声器与 AI

| 命令 | 指令码 | 说明 |
| --- | --- | --- |
| `twist_body` | `0x21010204` | 扭身体 |
| `flip` | `0x21010205` | 翻身 |
| `space_step` | `0x2101030C` | 太空步 |
| `backflip` | `0x21010502` | 后空翻 |
| `greet` | `0x21010507` | 打招呼 |
| `forward_jump` | `0x2101050B` | 向前跳 |
| `twist_jump` | `0x2101020D` | 扭身跳 |
| `speaker_on` | `0x2101030D` | 打开扬声器 |
| `speaker_off` | `0x2101030D` | 关闭扬声器 |
| `speaker_query` | `0x2101030D` | 查询扬声器状态 |
| `voice <value>` | `0x21010C0A` | 发送官方语音指令值 |
| `continuous_motion_on` | `0x21010C06` | 开启持续运动 |
| `continuous_motion_off` | `0x21010C06` | 关闭持续运动 |
| `ai_off` | `0x21012109` | 关闭所有 AI 选项 |
| `obstacle_stop` | `0x21012109` | 开启停障 |
| `follow` | `0x21012109` | 开启跟随 |

## 裸指令调试

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'raw 0x21010102 -20000 0'"
```

格式：

```text
raw <cmd_code> [cmd_value] [type]
simple <cmd_code> [cmd_value] [type]
```

裸指令不会做状态保护，测试时需要人工确认机器人姿态和周围安全。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `command_topic` | `/lite3_motion_cmd` | 命令入口 |
| `robot_basic_state_topic` | `/lite3/robot_basic_state` | 基本状态输入 |
| `robot_gait_state_topic` | `/lite3/robot_gait_state` | 步态状态输入 |
| `robot_motion_state_topic` | `/lite3/robot_motion_state` | 动作状态输入 |
| `robot_state_timeout` | `2.0` | 状态超时时间 |
| `standing_basic_states` | `6,9` | 认为已站立的基本状态 |
| `lie_basic_states` | `1` | 认为已趴下的基本状态 |
| `spot_basic_states` | `6,9` | 原地模式切换后的可接受状态 |
| `move_basic_states` | `6,9` | 移动模式切换后的可接受状态 |
| `mode_switch_timeout` | `3.0` | 模式切换确认超时 |
| `mode_switch_retry_count` | `3` | 模式切换重试次数 |
| `stand_timeout` | `8.0` | 起立/趴下状态确认超时 |
| `stand_settle_sec` | `1.0` | 起立确认后的稳定等待 |
| `default_prepare_gait` | `flat_low_gait` | `prepare_navigation` 使用的默认步态 |
| `inspection_pitch_value` | `-6553` | 识别姿态俯仰值，负值抬头 |
| `navigation_pitch_value` | `0` | 导航姿态俯仰恢复值 |
| `view_pose_step_sleep` | `0.5` | 识别姿态每次俯仰命令间隔 |
| `view_pose_pitch_repeat_count` | `3` | 识别姿态俯仰命令重复次数 |
| `low_pose_height_value` | `-20000` | 降低身体高度的轴值 |
| `normal_height_value` | `0` | 恢复身体高度的轴值 |

## 实机测试建议

1. 先启动 `message_transformer.launch`，观察 `/lite3/robot_basic_state` 是否持续发布。
2. 发送 `ensure_stand`，确认机器狗从趴下进入站立状态。
3. 发送 `prepare_navigation`，确认没有异常模式切换日志。
4. 发送 `inspection_view_pose`，确认站定后身体俯仰动作是否生效。
5. 发送 `navigation_view_pose`，确认能恢复移动模式。
6. 再测试 `/cmd_vel` 直走和转向，不建议一开始混测轴指令。
