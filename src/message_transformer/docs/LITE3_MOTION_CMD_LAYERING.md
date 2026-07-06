# Lite3 底层运动封装分层说明

本文档记录 `message_transformer/scripts/lite3_motion_cmd.py` 的重构结构。依据文件为 `docs/lite3_motion_host_udp_interface_beta_v1_0_7.md`，状态值和指令码优先以该手册为准。

## 总体原则

底层封装不再把“机器码”和“任务动作”混在一起。上层任务可以临时继续发布旧命令名，但底层内部按以下四层组织：

1. 协议常量层
2. ROS 原始发送层
3. 状态反馈与模式守卫层
4. 比赛任务动作配方层

这样做的目的有三个：

- 避免 `0x21010130` 这类复用轴指令在错误模式下被误用。
- 避免把 `0x21010202` 误认为单向起立命令，它实际是“起立/趴下 toggle”。
- 让导航、避障、硬编码备用链路都调用同一套准备动作和识别姿态动作。

## 第一层：协议常量层

代码位置：`Lite3Protocol`

该层只记录官方接口，不做任何策略判断。

主要内容：

| 类别 | 内容 |
| --- | --- |
| 基本状态 | `1=趴下`，`4=准备起立`，`5=正在起立`，`6=力控站立`，`7=正在趴下`，`8=失控保护`，`9=姿态调整` |
| 模式指令 | `0x21010D05=原地模式`，`0x21010D06=移动模式`，`0x21010C03=自主模式`，`0x21010C02=手动模式` |
| 姿态/轴指令 | `0x21010130`、`0x21010131`、`0x21010135` 在原地模式和移动模式下含义不同 |
| 步态指令 | 平地低速、中速、高速、正常/匍匐、抓地越障、通用越障、高踏步越障 |
| 动作指令 | 扭身体、翻身、太空步、后空翻、打招呼、向前跳、扭身跳 |
| 附加功能 | 持续运动、扬声器、语音指令、AI 选项 |

注意：协议常量层不提供 `ensure_stand()`、`inspection_view_pose()` 等任务语义。

## 第二层：ROS 原始发送层

代码位置：

- `raw_send_simple(code, value=0, cmd_type=0)`
- `raw_send_zero_velocity(duration=None)`

该层只负责向 ROS 话题发布，不判断机器人是否处于正确状态。

输出话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/simple_cmd` | `message_transformer/SimpleCMD` | 由 `ros2qnx` 转发为 Lite3 简单 UDP 指令 |
| `/cmd_vel` | `geometry_msgs/Twist` | 由 `ros2qnx` 转发为 Lite3 复杂速度指令 |

`ros2qnx.cpp` 中速度映射：

| ROS 输入 | Lite3 指令码 | 含义 |
| --- | --- | --- |
| `Twist.linear.x` | `0x0140` | 前后平移速度，单位 m/s |
| `Twist.angular.z` | `0x0141` | 旋转角速度，单位 rad/s |
| `Twist.linear.y` | `0x0145` | 左右平移速度，单位 m/s |

## 第三层：状态反馈与模式守卫层

代码位置：

- `has_fresh_robot_state()`
- `is_lie_down()`
- `is_standing()`
- `wait_for_basic_state()`
- `ensure_standing()`
- `ensure_spot_mode()`
- `ensure_move_mode()`
- `ensure_velocity_control_mode()`
- `lie_down_if_standing()`
- `command_axis_spot()`
- `command_axis_move()`

状态来源：

| 话题 | 说明 |
| --- | --- |
| `/lite3/robot_basic_state` | 机器人基本状态 |
| `/lite3/robot_gait_state` | 当前步态 |
| `/lite3/robot_motion_state` | 当前动作状态 |

默认状态参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `standing_basic_states` | `6,9` | 认为机器人已经站立或处于姿态调整状态 |
| `lie_basic_states` | `1` | 认为机器人处于趴下状态 |
| `spot_basic_states` | `6,9` | 原地模式切换后的可接受反馈状态 |
| `move_basic_states` | `6,9` | 移动模式切换后的可接受反馈状态 |

这里 `spot_basic_states` 和 `move_basic_states` 暂时都使用 `6,9`，原因是官方回传的 `robot_basic_state` 不一定能直接区分“原地模式”和“移动模式”。底层仍会发送模式切换指令并等待新鲜状态，只是用基本状态确认机器人没有掉线或趴下。若后续实机确认有更精确的状态组合，可再加入 `robot_gait_state` 或 `robot_motion_state` 条件。

## 第四层：比赛任务动作配方层

代码位置：

- `prepare_navigation()`
- `prepare_hardcoded_motion()`
- `enter_inspection_view_pose()`
- `restore_navigation_view_pose()`
- `set_low_body_height()`
- `restore_body_height()`
- `toggle_crawl_gait()`

推荐上层任务优先调用这些命令，而不是自己拼 `auto_mode,move_mode,flat_low_gait`。

| 命令 | 作用 |
| --- | --- |
| `prepare_navigation` | 确认站立，切自主速度控制模式，切默认步态，发布短零速度 |
| `prepare_hardcoded_motion` | 当前等同于 `prepare_navigation`，用于硬编码备用链路 |
| `inspection_view_pose` | 停止速度，切原地模式，按 `inspection_pitch_value` 调整身体俯仰 |
| `navigation_view_pose` | 按 `navigation_pitch_value` 恢复身体俯仰，并切回自主速度控制模式 |
| `height_low` | 在原地模式下发送降低身体高度轴值 |
| `height_normal` | 在原地模式下恢复身体高度轴值 |
| `crawl_gait_toggle` | 在移动模式下发送正常/匍匐步态 toggle |

## 关键风险

### 起立/趴下是同一个 toggle

`0x21010202` 不是单向起立命令。现在封装中：

- `ensure_stand` / `ensure_standing` 会先看状态，未站立时才发送 toggle。
- `stand_lie_toggle_raw` 是裸 toggle，仅用于调试。
- `lie_down_if_standing` 只有在确认站立时才发送 toggle。

### 俯仰和前进共用指令码

`0x21010130` 在不同模式下语义不同：

| 模式 | 封装命令 | 含义 |
| --- | --- | --- |
| 原地模式 | `pitch <value>` | 调整身体俯仰，正值低头，负值抬头 |
| 移动模式 | `forward_axis <value>` | 轴指令前后移动 |

因此任务链路中不要直接发布裸 `0x21010130`，除非正在做底层调试。

### 身体高度和匍匐步态不是同一件事

`height_low` 使用 `0x21010102` 调整身体高度。  
`crawl_gait_toggle` 使用 `0x21010406` 切换正常/匍匐步态。

旧命令 `low_pose` 仍保留兼容，但内部会执行“降低高度 + 匍匐步态 toggle”，实机使用前应单独确认两个动作的效果。
