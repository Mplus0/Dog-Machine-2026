# 任务 launch map

Top-level task entries are the only launch files in this folder that start with
`task_2026_`.

## Top-level entries

- `task_2026_obstacle_test.launch`
  - Starts `message_transformer`, the shared navigation stack, and the obstacle
    zone test node.
  - Use this for the standalone obstacle-area validation route.

- `task_2026_navigation.launch`
  - Starts `message_transformer`, the shared navigation stack, persistent Docker
    meter inspection, audio/state storage, and the sequential navigation
    inspection node.
  - Use this for the full navigation route. The only recognition interface is
    `/meter_inspect_trigger` -> `/meter_status`.

- `task_2026_hardcoded_motion.launch`
  - Starts `message_transformer`, persistent Docker meter inspection,
    audio/state storage, and the hardcoded cmd_vel sequence.
  - Use this for the fallback hardcoded route.

## Shared/local launch files

- `stack_nav_base.launch`
  - Shared navigation stack used by task launches.
  - Includes camera, camera TF, odometry preprocessing, depth-to-scan, map,
    AMCL, and move_base.

- `node_obstacle_zone_task.launch`
  - Local wrapper for `obstacle_zone_task.py`.

- `node_sequential_nav_inspect.launch`
  - Local wrapper for `sequential_nav_inspect.py`.

## Bottom-level component launch files

These are component launch files and should usually be included by a task or
stack launch instead of started as the competition entry point:

- `amcl.launch`
- `camera.launch`
- `camera2base_tf.launch`
- `depth2laser.launch`
- `ekf_localization.launch`
- `map_server.launch`
- `movebase.launch`
- `odom.launch`
- `odom_add_frame.launch`
- `odom_reset.launch`
- `rviz_nav.launch`
- `throttle_imu.launch`

