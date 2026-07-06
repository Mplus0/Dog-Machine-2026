# dog_motion 识别与语音接口

当前工程只保留 Docker 仪表识别链路，不再维护旧的预选赛兼容识别接口。

## 统一话题

- `/meter_inspect_trigger`：识别触发，`std_msgs/String`
- `/meter_status`：识别结果，`std_msgs/String`，格式如 `rec_pose_1,A,normal`
- `/meter_state_json`：A/B/C/D 状态记忆快照，latched JSON 字符串

## 主要 launch

```bash
roslaunch dog_motion meter_reader_docker_persistent.launch
```

该 launch 会启动：

- `meter_persistent_docker_inspection_node.py`：启动常驻 Docker 容器、等待触发、抓取彩色图、复用已加载模型推理、发布 `/meter_status`
- `meter_audio_node.py`：订阅 `/meter_status` 并播报语音
- `meter_state_store_node.py`：订阅 `/meter_status` 并记忆 A/B/C/D 状态；默认启动时清空上一轮状态，避免抓放阶段误用旧结果

备用链路：

```bash
roslaunch dog_motion meter_reader_docker_on_demand.launch
```

备用链路每次触发都会重新启动 Docker 并加载模型，速度较慢，但适合排查常驻容器问题。

比赛总入口通常不直接启动本 launch，而是由：

```bash
roslaunch allmovebase task_2026_navigation.launch
roslaunch allmovebase task_2026_hardcoded_motion.launch
```

自动包含。

## Docker 约定

默认镜像：

```text
yolo11
```

默认容器工作目录：

```text
/workspace
```

默认模型路径：

```text
/workspace/models/yuyin.engine
```

对应宿主机路径：

```text
comp2026_ws/src/dog_motion/models/yuyin.engine
```

模型文件通常被 ignore，不会随本地工程同步；实机部署时请确认该文件存在。

当前 `meter_batch_infer.py` 不做裁剪，直接对采样的整张彩色图推理。

常驻链路使用 `meter_persistent_infer.py`，同样不做裁剪，直接对采样目录内的整张 JPG 推理。

手动进入容器调试：

```bash
docker run -it --rm \
  --runtime=nvidia \
  --privileged \
  --network host \
  -v ~/comp2026_ws/src/dog_motion:/workspace \
  -v /dev:/dev \
  -w /workspace \
  --entrypoint /bin/bash \
  yolo11
```

## 单独测试

启动识别节点后，确保已有 `/camera/color/image_raw`，再触发：

```bash
rostopic pub -1 /meter_inspect_trigger std_msgs/String "data: 'rec_pose_1'"
rostopic echo /meter_status
rostopic echo /meter_state_json
```

音频设备可用性仍可用系统命令检查：

```bash
aplay -l
pactl list short sinks
speaker-test -c 2
```
