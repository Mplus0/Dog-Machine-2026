# 机器狗导航地面误检与 AMCL 坐标偏移 Debug

> **用途：** 交给 Codex 在机器狗主机上进行实机排查。  
> **当前阶段：只 Debug，不修改代码。**  
> **禁止：修改源码 / YAML / launch / TF、commit、push。**

---

## 1. Debug 目标

当前机器狗导航存在两个现象：

1. D435i 相机向下俯视，地面被转换成 `/scan` 中的障碍物，并写入 local/global costmap。
2. 出现地面假障碍时，RViz 中机器狗在 `map` 坐标系下可能发生位置偏移。

本轮需要确认：

```text
问题 1：
地面是否首先进入 /scan？

问题 2：
错误 /scan 是否进入 Costmap？

问题 3：
同一个错误 /scan 是否同时影响 AMCL？

问题 4：
RViz 中的坐标偏移究竟来自：
map -> odom
还是
odom -> base_link？
```

---

# 2. 已确认项目架构

项目使用：

```text
ROS1
move_base
costmap_2d
AMCL
D435i
depthimage_to_laserscan
robot_localization / EKF
```

当前导航数据链已经确认：

```text
D435i
  │
  ▼
/camera/depth/image_rect_raw
  │
  ▼
depthimage_to_laserscan
  │
  ▼
/scan
  │
  ├──────────────────────┐
  │                      │
  ▼                      ▼
Costmap                  AMCL
  │                      │
  ▼                      ▼
local/global          map -> odom
costmap                  │
  │                      ▼
  ▼                  RViz 位姿
路径规划异常
```

因此本轮最重要的原则：

> **先检查 `/scan`，不要先修改 Costmap 或 AMCL。**

---

# 3. 当前最高概率根因

当前实现使用固定扫描行：

```text
scan_height = 1
scan_row_offset = -132
range_min = 0.1
range_max = 3.0
output_frame_id = scan_link
```

重点文件：

```text
src/allmovebase/launch/depth2laser.launch
```

核心思想实际上是：

```text
Depth Image
    ↓
固定选择一行像素
    ↓
这一行转换成 LaserScan
    ↓
发布 /scan
```

而不是：

```text
Depth / PointCloud
    ↓
转换到机器人坐标系
    ↓
判断真实高度
    ↓
过滤 ground
    ↓
生成 LaserScan
```

---

# 4. `scan_row_offset=-132` 的意义

仓库中记录的 D435i 深度内参约为：

```text
fy = 388.389587
cy = 239.344116
```

当前：

```text
scan_row_offset = -132
```

因此扫描行约为：

```text
v = cy - 132
  ≈ 107.34
```

对应相对相机主光轴的角度：

```text
atan(-132 / 388.389587)
≈ -18.77°
```

仓库记录相机实际下俯角约：

```text
-18.82°
```

因此：

> `-132` 在机器狗静止、机身姿态与标定姿态一致时，基本正确。

所以暂时不要把：

```text
scan_row_offset=-132
```

直接判断成错误参数。

真正的问题可能是：

```text
固定 -132
+
四足机器人动态 pitch / roll
```

---

# 5. 为什么机器狗运动会重新扫到地面

机器狗运动过程中会发生：

```text
pitch
roll
机身高度变化
步态周期性摆动
```

而：

```text
scan_row_offset=-132
```

不会随姿态变化。

所以它只能保证：

```text
某一个静态姿态下近似水平
```

不能保证：

```text
机器狗走路时始终水平
```

---

## 5.1 Pitch 风险

相机高度约：

```text
0.415 m
```

最大 scan range：

```text
3.0 m
```

如果静止时射线刚好水平，那么额外低头：

```text
asin(0.415 / 3.0)
≈ 7.95°
```

即可让地面进入 3 m 扫描范围。

因此需要重点验证：

> 机器狗额外低头约 8° 后， `/scan` 是否开始出现地面。

例如额外低头约 10°：

```text
ground distance
≈ 0.415 / sin(10°)
≈ 2.39 m
```

如果 RViz 中出现约 2～2.5 m 的连续假障碍，这与该模型高度一致。

---

## 5.2 Roll 风险

当前固定扫描线：

```text
-----------------------
```

但机器狗发生 roll 后，真实水平线在图像中应类似：

```text
///////////////////////
```

或：

```text
\\\\\\\\\\\\\\\\\\\\\\\
```

因此固定 row 无法补偿 roll。

重点观察：

```text
左倾 → 是否一侧先扫到地面

右倾 → 是否另一侧先扫到地面
```

如果呈镜像变化，则基本可以确认固定 row 对 roll 不适用。

---

# 6. P0：检查 `depthimage_to_laserscan`

重点文件：

```text
src/depthimage_to_laserscan/include/depthimage_to_laserscan/DepthImageToLaserScan.h
```

重点检查类似：

```cpp
const int offset =
    center_y + scan_row_offset_ - scan_height / 2;
```

确认后续是否只是：

```text
固定 row
↓
读取 Depth
↓
计算距离
↓
range_min/range_max 判断
↓
生成 LaserScan
```

需要搜索以下关键词：

```text
scan_row_offset
scan_height
range_min
range_max
ground
height
imu
pitch
roll
tf
transform
RANSAC
plane
```

本轮需要回答：

```text
[ ] 是否存在 ground height filter？
[ ] 是否使用 IMU？
[ ] 是否动态补偿 pitch？
[ ] 是否动态补偿 roll？
[ ] 是否把点转换到重力对齐坐标系？
[ ] 是否只按照 range 判断有效点？
```

当前静态代码审查倾向：

> 没有真正的动态地面过滤。

---

# 7. P1：Costmap

重点文件：

```text
src/allmovebase/config/common_costmap_params.yaml
```

已确认 `/scan` 被作为 obstacle source：

```yaml
observation_sources: laser_scan_sensor

laser_scan_sensor:
  topic: /scan
  data_type: LaserScan
  marking: true
  clearing: true
```

因此：

```text
错误 /scan
    ↓
obstacle layer
    ↓
local/global costmap
```

如果：

```text
/scan 中已经存在地面
```

那么 Costmap 把它标成障碍属于正常行为。

因此：

> **Costmap 当前不是第一修改对象。**

---

# 8. P1：AMCL

重点文件：

```text
src/allmovebase/launch/amcl.launch
src/allmovebase/config/amcl.yaml
```

已确认 AMCL 同样使用：

```text
/scan
```

例如：

```xml
<remap from="scan" to="/scan"/>
```

所以：

```text
错误 /scan
   ├──> Costmap
   └──> AMCL
```

这可以同时解释：

```text
地面假障碍
+
map 坐标位置偏移
```

---

# 9. 为什么 AMCL 可能发生偏移

AMCL 本质上进行：

```text
Map
+
LaserScan
↓
粒子匹配
```

如果 `/scan` 中存在地图上根本不存在的大量地面障碍：

```text
真实地图：
墙、桌腿、固定障碍

实际 Scan：
墙、桌腿、固定障碍
+
大量地面假点
```

则可能造成：

```text
scan-map matching 质量下降
↓
particlecloud 异常
↓
amcl_pose 调整
↓
map -> odom 调整
```

RViz Fixed Frame 使用：

```text
map
```

因此会表现为：

```text
机器人在 RViz 中整体偏移 / 跳动
```

---

# 10. 必须区分两种坐标问题

## Case A

如果：

```text
odom -> base_link 稳定

但

map -> base_link 跳
```

则重点怀疑：

```text
AMCL
↓
map -> odom
```

可能与错误 `/scan` 有关。

---

## Case B

如果：

```text
odom -> base_link
```

自身也发生跳动：

则不能归因于 `/scan`。

继续检查：

```text
leg_odom2
odom_reset
IMU
EKF
robot_localization
```

---

# 11. EKF 当前关系

重点文件：

```text
src/allmovebase/config/ekf_localization.yaml
```

当前静态审查发现 EKF 输入主要为：

```text
/odom_reset
/imu/data_throttled
```

没有直接使用：

```text
/scan
/camera
```

因此：

> 错误 `/scan` 不应该直接修改 `odom -> base_link`。

所以 Debug 时必须同时查看：

```text
map -> odom
odom -> base_link
```

---

# 12. P2：注意遗留配置

发现：

```text
src/allmovebase/config/pointcloud_to_laserscan.yaml
```

其中存在：

```yaml
min_height: 0.1
max_height: 1.5
```

虽然看起来可以过滤地面，但当前主链使用的是：

```text
depthimage_to_laserscan
```

不是：

```text
pointcloud_to_laserscan
```

Codex 需要确认这个 YAML 有没有真正被 launch 加载。

搜索：

```bash
grep -R "pointcloud_to_laserscan.yaml" -n src
grep -R "pointcloud_to_laserscan" -n src
```

如果没有实际引用：

```text
该配置属于遗留/未启用配置
```

不要因为这里存在 `min_height` 就认为当前已经做了 ground filtering。

---

# 13. Debug Step 1：确认 Git 状态

先执行：

```bash
cd <comp2026_ws>

git status
git branch --show-current
git rev-parse HEAD
git log -1 --oneline
```

要求：

```text
不要修改
不要 stash
不要 reset
不要 checkout
不要 commit
不要 push
```

把结果原样回传。

---

# 14. Debug Step 2：确认真实运行链

导航启动后执行：

```bash
rosnode list
```

确认至少以下相关节点：

```text
depthimage_to_laserscan
amcl
move_base
EKF / robot_localization
D435i camera
```

然后：

```bash
rostopic info /scan
rostopic info /amcl_pose
rostopic info /odometry/filtered
```

重点回传：

```text
/scan publisher
/scan subscribers
```

确认 `/scan` 是否确实同时被：

```text
AMCL
move_base / Costmap
```

使用。

---

# 15. Debug Step 3：读取运行时参数

不要只相信源码中的 launch。

执行：

```bash
rosparam list | grep -Ei "scan|depth|laser"
```

找到正确 namespace 后读取：

```bash
rosparam get <depthimage_to_laserscan namespace>
```

确认：

```text
scan_height
scan_row_offset
range_min
range_max
output_frame_id
```

必须以**运行时参数**为准。

---

# 16. Debug Step 4：静止 `/scan` 测试

机器狗正常站立，不运动。

RViz 只重点观察：

```text
/scan
```

同时：

```bash
rostopic hz /scan
rostopic echo -n 1 /scan
```

记录：

```text
静止是否已经扫到地面
假障碍大概距离
左右是否对称
```

---

## 判定

### FAIL

如果：

```text
静止时
/scan 已经出现明显地面
```

说明：

```text
固定 scan_row_offset
在当前静态姿态下就不正确
```

### PASS

如果：

```text
静止 /scan 正常
```

继续测试动态姿态。

---

# 17. Debug Step 5：Pitch 测试

保持机器狗位置尽量不变。

测试：

```text
正常站立
↓
轻微低头
↓
继续低头
↓
恢复
↓
抬头
```

同时观察：

```text
/scan
Local Costmap
Global Costmap
/particlecloud
/amcl_pose
```

同时查看 IMU：

```bash
rostopic echo /imu/data_throttled
```

如 topic 不同，先确认实际 IMU topic。

记录：

```text
大约多少 pitch 时 /scan 开始出现地面
地面假 scan 的距离
```

---

## 强证据

如果出现：

```text
静止 /scan 正常
↓
低头
↓
/scan 出现连续地面
```

则：

```text
[CONFIRMED]
固定 scan_row_offset 无法处理动态 pitch
```

如果进一步：

```text
低头 → scan 地面距离约 2～3 m
```

与前面的几何计算一致，则证据更强。

---

# 18. Debug Step 6：Roll 测试

原地轻微：

```text
左倾
右倾
```

观察 `/scan`。

重点：

```text
左倾是否一侧扫地
右倾是否另一侧扫地
```

如果结果呈镜像：

```text
[CONFIRMED]
固定水平扫描行无法补偿动态 roll
```

---

# 19. Debug Step 7：验证 Costmap

当 `/scan` 出现地面时观察：

```text
local_costmap
global_costmap
```

判断时序：

```text
/scan 出现假地面
↓
Costmap 相同方向 / 距离出现障碍
```

如果成立：

```text
[CONFIRMED]
Costmap 假障碍来自上游错误 /scan
```

不要把 Costmap 当作第一根因。

---

# 20. Debug Step 8：验证 AMCL / TF

同时开三个终端。

### Terminal A

```bash
rosrun tf tf_echo odom base_link
```

### Terminal B

```bash
rosrun tf tf_echo map odom
```

### Terminal C

```bash
rosrun tf tf_echo map base_link
```

同时：

```bash
rostopic echo /amcl_pose
```

RViz：

```text
/particlecloud
```

---

# 21. 最重要判定矩阵

## Case 1

```text
/scan 出现地面
Costmap 出现地面
odom -> base_link 稳定
map -> odom 跳
amcl_pose 跳
```

结论：

```text
[CONFIRMED / HIGH PROBABILITY]

错误 /scan
↓
AMCL 匹配异常
↓
map -> odom 偏移
```

---

## Case 2

```text
/scan 出现地面
Costmap 出现地面
odom -> base_link 稳定
map -> odom 稳定
```

结论：

```text
已确认地面感知问题

但尚未证明 AMCL 位姿异常由它引起
```

---

## Case 3

```text
/scan 正常
odom -> base_link 跳
```

结论：

```text
排查 EKF / IMU / leg odom

不是本轮 /scan 主问题
```

---

## Case 4

```text
/scan 正常
odom -> base_link 稳定
map -> odom 跳
```

结论：

```text
独立 AMCL / 地图 / TF / scan-map matching 问题
```

---

# 22. 建议录制 rosbag

如果存储允许：

```bash
rosbag record \
  /scan \
  /imu/data_throttled \
  /amcl_pose \
  /particlecloud \
  /odometry/filtered \
  /tf \
  /tf_static
```

Topic 不一致时以实际 topic 为准。

建议测试动作：

```text
T0 静止
T1 轻微低头
T2 明显低头
T3 恢复
T4 左倾
T5 右倾
T6 缓慢前进
```

建议文件：

```text
ground_scan_debug_YYYYMMDD_HHMMSS.bag
```

---

# 23. 本轮暂时不要修改

在完成上述验证之前不要优先修改：

```text
inflation_radius
cost_scaling_factor
obstacle_range
raytrace_range

laser_z_hit
laser_z_rand
laser_max_range

AMCL particle 数量

global planner
local planner

range_max
```

原因：

> 如果 `/scan` 本身已经错误，调整下游参数只是在容忍错误传感器数据。

---

# 24. 后续解决方案候选

本轮**不要实现**，Debug 完成后再决定。

---

## A. 继续使用固定扫描行

```text
scan_row_offset
```

优点：

```text
实现简单
算力最低
```

缺点：

```text
只能适应静态姿态
无法处理 pitch
无法处理 roll
上下坡容易失效
四足步态下不稳定
```

用途：

```text
仅适合快速验证
```

---

## B. IMU 动态扫描行

实时获取 pitch，根据内参动态计算：

```text
scan row
```

概念：

```text
v = cy + fy * tan(theta)
```

优点：

```text
算力较小
能够补偿动态 pitch
```

缺点：

```text
roll 仍难正确处理
需要时间同步
仍不是真正三维地面判断
```

用途：

```text
中间方案
```

---

## C. 三维地面过滤

推荐最终评估方向：

```text
Depth Image / PointCloud
        ↓
反投影 XYZ
        ↓
Camera Extrinsic
        ↓
IMU / Gravity
        ↓
机器人重力对齐坐标系
        ↓
Ground Filtering
        ↓
Obstacle Points
        ↓
LaserScan
        ↓
/scan
```

可考虑：

```text
高度过滤
PassThrough
CropBox
RANSAC ground plane
IMU gravity aligned filtering
```

优点：

```text
支持动态 pitch
支持动态 roll
适合四足机器人
```

---

# 25. Codex 最终必须回传

请严格使用：

```text
## Environment

workspace:
branch:
HEAD:
git status:

## Runtime Pipeline

depth topic:
depthimage_to_laserscan node:
scan publisher:
scan subscribers:
AMCL scan input:
Costmap scan input:

## Runtime Parameters

scan_height:
scan_row_offset:
range_min:
range_max:
output_frame_id:

## Static Test

/scan ground:
false obstacle range:
costmap:
amcl_pose:
map->odom:
odom->base_link:

## Pitch Test

pitch:
scan result:
false obstacle range:
costmap result:
particlecloud:
amcl_pose:
map->odom:
odom->base_link:

## Roll Test

left roll:
right roll:
single-side ground:
mirror behavior:

## Root Cause

[CONFIRMED]

...

[HIGH PROBABILITY]

...

[UNCONFIRMED]

...

[EXCLUDED]

...

## Suggested Fix Location

file:
function/config:
reason:

## Recommended Solution

方案：
理由：

## Git Check

modified files:
commit created:
push performed:
```

最后三项理论上必须为：

```text
modified files: none
commit created: no
push performed: no
```

---

# 26. 一句话任务指令

> **不要修改代码。请在机器狗实机上沿 `D435i → depthimage_to_laserscan → /scan → Costmap / AMCL` 完整排查，重点验证固定 `scan_row_offset=-132` 是否会因为动态 pitch/roll 重新扫到地面；同步比较 `/scan`、Costmap、`amcl_pose`、`map->odom` 和 `odom->base_link`，确认地面假扫描是否同时导致 Costmap 污染以及 AMCL 坐标偏移。所有结论必须附实机证据，按本文模板回传，不 commit、不 push。**
::: ​​