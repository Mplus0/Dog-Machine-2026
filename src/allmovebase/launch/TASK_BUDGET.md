# 任务预算器说明

预算器用于把全流程控制在指定时间内。它只在阶段边界和等待循环中读取系统时间，不订阅图像、不查询 ROS 图，也不做高频计算，对导航负载影响可以忽略。

## 默认策略

- `task_2026_full.launch` 默认启用预算器，`task_budget_total=300.0`，也就是 5 分钟。
- 全流程默认 `task_budget_start_after_prerequisites=true`，表示完成起立准备、导航栈和识别节点就绪后才开始计时。
- 避障、导航识别、硬编码、抓放分块链路也接入了预算器接口，但默认 `task_budget_enable=false`，避免影响单项调试。

## 通用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `task_budget_enable` | full 为 `true`，分块为 `false` | 是否启用预算器 |
| `task_budget_total` | `300.0` | 总预算秒数 |
| `task_budget_reserve` | `5.0` | 预留时间，等待和超时会避开这部分余量 |
| `task_budget_warn_interval` | `5.0` | 预算不足日志的最小重复间隔 |

## 全流程参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `task_budget_start_after_prerequisites` | `true` | 是否在基础链路就绪后才开始计时 |
| `min_obstacle_remaining` | `30.0` | 进入避障阶段前要求的最少剩余时间 |
| `min_inspection_remaining` | `60.0` | 进入四点识别阶段前要求的最少剩余时间 |
| `min_pick_place_remaining` | `45.0` | 进入抓放阶段前要求的最少剩余时间 |

示例：

```bash
roslaunch allmovebase task_2026_full.launch task_budget_total:=300
```

如果比赛计时从 launch 启动瞬间开始：

```bash
roslaunch allmovebase task_2026_full.launch task_budget_start_after_prerequisites:=false
```

## 分块链路参数

避障测试：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch task_budget_enable:=true task_budget_total:=60
```

导航识别：

```bash
roslaunch allmovebase task_2026_navigation.launch task_budget_enable:=true task_budget_total:=180
```

硬编码备用路线：

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch task_budget_enable:=true task_budget_total:=180
```

## 行为边界

预算器会压缩或截断等待时间，例如等待识别 ready、等待颜色帧、等待 move_base 结果等。它不会跳过起立准备，也不会主动修改路径点。硬编码链路在预算不足以完成下一段直行或转向时会停止进入下一段，避免走一半继续执行后续识别逻辑。

