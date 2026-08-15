# AMCL 坐标跳跃诊断与围挡验收

## 当前证据与边界

2026-08-15 开放场地只读检查：

- Git HEAD：`68c0f04e30ff87b37f991a99bce7b1c38f0b10c2`。
- AMCL 实际订阅 `/amcl_map` 和 `/scan_ground_filtered`，没有订阅原始 `/scan`。
- `/amcl_map` 与 `/map` 都是 120×120、0.05 m/格、原点 `[0,0,0]`。
- AMCL 图只有顶部大型障碍和三个箱子；外边缘占用为
  `Top=41, Bottom=0, Left=0, Right=0`。
- 导航图四边均占用，但还包含右侧约 1 m 宽的导航禁行区域，不能整张复制给 AMCL。
- 静止时 `map→odom` 和 `odom→base_link` 均稳定，滤波诊断为 OK，暂不支持
  odom/EKF 或地面滤波持续异常。
- 配置文件写了 `do_beamskipping: true`，但 Noetic AMCL 实际参数名是
  `do_beamskip`；运行时 `do_beamskip=false`。A 组完成前不修改此参数。

Gate6 保持 `PAUSED`。在捕获跳跃瞬间之前，不修改重力滤波参数，不用导航图直接
替换 AMCL 图，也不把虚拟边界画入 AMCL 图。

### A1 开放场地记录结论（2026-08-15）

首次 `open_oldmap` 已捕获到真实定位跳变。它不是一次孤立显示闪烁，而是 AMCL 在约
0.4 s 内连续切换了三个假设后回到原位置：

| 时间戳 | `map→odom` 单步变化 | 说明 |
|---|---:|---|
| 1786749525.748 | 4.175 m / 55.66° | 跳到场内另一组假设 |
| 1786749525.882 | 2.039 m / 55.60° | 再次切换假设 |
| 1786749526.148 | 2.468 m / 113.41° | 回到原位置附近 |

跳变前 AMCL 的 x/y/yaw 协方差已经明显增大，随后才发生假设切换。同期证据为：

- `odom→base_link` 最大单步平移约 0.0039 m，角度连续，不支持 odom/EKF 离散跳变。
- 地面滤波高度、倾角、地面内点和 clearing bins 均正常，不支持滤波器瞬时翻转。
- `/scan_ground_filtered` 的有限点数随朝向由 539 平滑下降到 88，量程也连续变化，
  没有突然的扫描几何翻转。
- 因此现阶段最符合“开放场地中场外回波/缺失边界造成扫描与旧地图不匹配”，
  Gate6 仍为 `PAUSED`，暂不修改滤波器和 AMCL likelihood-field 参数。

首次记录的 TF 反推旋转角速度常在 40–100°/s，峰值约 100°/s。由于手柄最低档速度
不易精确控制，这不使 A1 失效：A1 可作为“实际可达转速”的失败基线。B/C 组应使用
相同手柄档位、旋转方向和近似整圈耗时，并由记录中的 TF 实测角速度做事后对照。

## 诊断工具

构建并加载：

```bash
cd ~/comp2026_ws
git pull
catkin_make --pkg allmovebase
source devel/setup.bash
```

工具只订阅和录制，不发布 `/cmd_vel`，不调用 `clear_costmaps`、
`request_nomotion_update`、`global_localization` 或 Initial Pose。

```bash
# 开始；标签建议使用 open_oldmap、fence_oldmap、fence_newmap
rosrun allmovebase run_amcl_jump_diagnostic.py start open_oldmap

rosrun allmovebase run_amcl_jump_diagnostic.py status
rosrun allmovebase run_amcl_jump_diagnostic.py logs

# 完成动作后停止，确保 rosbag 正常写完索引
rosrun allmovebase run_amcl_jump_diagnostic.py stop
```

默认输出到：

```text
~/amcl_jump_diagnostics/YYYYMMDD_HHMMSS_标签/
```

其中包含：

- `amcl_jump_diag.bag`：TF、AMCL、两种地图、三种扫描、IMU、里程计和诊断。
- `events.jsonl`：关键 TF、AMCL pose、odometry 文本时间线。
- `jumps.jsonl`：超过阈值的单次变化及同步扫描/协方差/诊断快照。
- `runtime_snapshot.txt`：启动时 Git、节点、AMCL 参数和真实订阅关系。
- 两个 console log。

告警分类只是抓证据，不直接代替人工结论：

```text
AMCL_CANDIDATE  -> map→odom 跳跃附近没有 odom→base_link 跳跃
ODOM_OR_SHARED  -> 同期 odom→base_link 也发生跳跃，优先检查 odom/EKF
```

诊断器对 `map→odom` 仍使用 0.05 m 或 2° 的单步阈值；对
`odom→base_link` 的角度判定同时要求角度单步超过 2°且角速度超过 180°/s，以免把
正常连续转向误报为 odom 跳跃。平移仍使用 0.05 m 单步阈值。旧版 A1 中的
`ODOM_OR_SHARED` 角度告警应按本节重新解释，不作为 odom 异常证据。

## A/B/C 顺序

三个箱子始终保持在 AMCL 图对应位置。每组采用相同初始位置、方向和动作。使用手柄
可稳定复现的最低旋转档位；A/B/C 应保持相同档位，整圈耗时尽量控制在基线的 ±15%。
每次启动记录后先静止 5 s，再旋转约 360°，然后静止 5 s 并停止记录。工具本身不会
让机器狗旋转。

若需要额外区分“高速转向影响”，可以做可选慢速诊断：短按旋转约 10–15°，松开并
等待约 1 s，重复到一整圈。该分段动作不是 Gate6 强制验收项，也不能与连续旋转组
直接混作 A/B 对照。

### A：开放场地 + 旧地图

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start open_oldmap
```

目标：记录场外杂物可见时 `map→odom`、`odom→base_link` 和扫描的同步证据。不要在
记录过程中调用静止更新、重定位或清图服务。

A1 已可作为实际手柄速度下的失败基线。若希望增加上述分段慢速诊断，可另录一轮，
标签使用：

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start open_oldmap_slow
```

### B：真实围挡 + 旧地图

测试围挡应尽量复现正式场地：除右下起点开口外其余边界连续，围挡高约 0.5 m，
内侧位置和开口宽度固定，并能被 D435i 稳定观察。地图仍用
`arena_amcl_manual_1.pgm`。这一组区分“围挡遮住场外杂物的收益”和“旧地图缺少
围挡形成的新 mismatch”。在记录中写下测试围挡的内尺寸、右下开口宽度及开口两端
相对地图角点的位置，供 C 组制图使用。

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start fence_oldmap
```

2026-08-15 的有效滤波重测结果为 `FAIL`：

- `/scan_ground_filtered`、`/scan_ground_clearing` 和滤波诊断均正常录制。
- `map→odom` 从约 `(5.217, 1.187, -179.7°)` 漂到
  `(4.656, 1.003, 152.1°)`，累计约 0.59 m / 28°。
- 共捕获 5 次 `AMCL_CANDIDATE` 修正，最大单次约 0.165 m / 12.26°。
- `odom→base_link` 最大单步平移约 0.0041 m，转向连续。

结论：物理围挡可被深度扫描稳定观察，但旧 AMCL 地图缺少围挡，新的 scan-map
mismatch 足以阻止定位正确收敛。该结果支持进入 C 组，不支持修改 odom 或地面滤波。

### C：真实围挡 + 新 AMCL 地图

只有确认围挡位置后才生成新地图。新地图应保留现有固定障碍，并只增加真实可观测
围挡；不得复制导航图右侧约 1 m 宽的虚拟禁行区域。

当前测试围挡与 6.0 m×6.0 m 场地对齐。右下开口两端分别与箱子 3 的右侧面和
下侧面对齐，因此：

- 下边界占用 `x=[0.00, 4.70) m`，`x=[4.70, 6.00] m` 开放。
- 右边界占用 `y=[1.65, 6.00] m`，`y=[0.00, 1.65) m` 开放。
- 上边界和左边界连续。
- 三个箱子保持不变。旧图顶部 `2.05 m×0.55 m` 的粗块不对应第四个场内障碍，
  在 v2 中替换为一格厚的连续上围挡。

候选地图为 `map/map_amcl_fence_v2.yaml`，默认 `map_amcl.yaml` 尚未切换。启动 C 组：

```bash
roslaunch allmovebase task_2026_obstacle_test.launch \
  run_obstacle_task:=false \
  start_ground_filter_experimental:=true \
  navigation_scan_topic:=/scan_ground_filtered \
  amcl_map_yaml:=/home/ysc/comp2026_ws/src/allmovebase/map/map_amcl_fence_v2.yaml
```

验证 `/amcl_map` 已加载 v2；期望 `occupied=881`：

```bash
python3 -c 'import rospy; from nav_msgs.msg import OccupancyGrid; rospy.init_node("check_amcl_map",anonymous=True); m=rospy.wait_for_message("/amcl_map",OccupancyGrid,5); print("size=%dx%d resolution=%.2f occupied=%d"%(m.info.width,m.info.height,m.info.resolution,sum(v==100 for v in m.data)))'
```

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start fence_newmap
```

## 新 AMCL 地图生成前必须确认

- 围挡内侧的实际长宽（不要默认一定是 6.0 m×6.0 m）。
- 右下起点开口的宽度，以及开口两端相对物理角点的位置。
- 地图原点对应哪个物理角点，x/y 正方向是否与现场一致。
- 四边内侧表面在地图中应落在哪一格。
- 围挡高度、材质和连续性是否保证 D435i 在正常步态和转向时稳定得到深度。
- 围挡是否为比赛期间固定且不会移动的结构。

未确认这些信息时，不创建或切换默认 AMCL 地图。

## Gate6 判定

每一配置至少重复三轮。通过要求：

- `/scan_ground_filtered` 持续不低于 10 Hz，扫描几何无翻转或地面扇形。
- `odom→base_link` 连续，无单次异常跳跃。
- `map→odom` 无超过 0.05 m 或 2° 的非预期单次修正。
- ParticleCloud 不突然长期发散，协方差不持续扩大。
- Costmap 不累积地面假障碍。
- 返回后定位误差满足任务要求。

如果 C 组仍发生跳跃，再根据 `jumps.jsonl` 和 rosbag 决定是否修正
`do_beamskip` 参数名或进入 odom/滤波排查，不先放宽 likelihood-field 参数。
