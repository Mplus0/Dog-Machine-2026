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
- `jumps.jsonl`：超过 0.05 m 或 2° 的单次变化及同步扫描/协方差/诊断快照。
- `runtime_snapshot.txt`：启动时 Git、节点、AMCL 参数和真实订阅关系。
- 两个 console log。

告警分类只是抓证据，不直接代替人工结论：

```text
AMCL_CANDIDATE  -> map→odom 跳跃附近没有 odom→base_link 跳跃
ODOM_OR_SHARED  -> 同期 odom→base_link 也发生跳跃，优先检查 odom/EKF
```

## A/B/C 顺序

三个箱子始终保持在 AMCL 图对应位置。每组采用相同初始位置、方向、旋转速度和动作。
每次启动记录后先静止 5 s，再人工安全慢速旋转约 360°，然后静止 5 s 并停止记录。
工具本身不会让机器狗旋转。

### A：开放场地 + 旧地图

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start open_oldmap
```

目标：记录场外杂物可见时 `map→odom`、`odom→base_link` 和扫描的同步证据。不要在
记录过程中调用静止更新、重定位或清图服务。

### B：真实围挡 + 旧地图

围挡必须四边连续，内侧位置固定，并能被 D435i 稳定观察。地图仍用
`arena_amcl_manual_1.pgm`。这一组区分“围挡遮住场外杂物的收益”和“旧地图缺少
围挡形成的新 mismatch”。

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start fence_oldmap
```

### C：真实围挡 + 新 AMCL 地图

只有确认围挡位置后才生成新地图。新地图应保留现有固定障碍，并只增加真实可观测
围挡；不得复制导航图右侧约 1 m 宽的虚拟禁行区域。

```bash
rosrun allmovebase run_amcl_jump_diagnostic.py start fence_newmap
```

## 新 AMCL 地图生成前必须确认

- 围挡内侧是否正好形成 6.0 m×6.0 m 矩形。
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
