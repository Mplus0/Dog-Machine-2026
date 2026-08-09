# 实机上机 Checklist

本文用于下一次场地测试时按顺序执行和记录。默认命令在感知主机执行。

网络拓扑详见：

```text
README_NETWORK_TOPOLOGY.md
```

## 0. 测试记录

```text
日期：
场地：
测试人员：
开发 WiFi/手机热点：iPhone-hotspot
手机热点网关：172.20.10.1
感知主机 wlan0：172.20.10.2/28，DHCP 当前地址，默认路由
感知主机 wlan0 MAC：6C:1F:F7:88:0C:02
感知主机 wlan1：192.168.2.213/24，连接机器狗 AP，ipv4.never-default=yes
感知主机 wlan1 MAC：6C:1F:F7:F4:09:76
机器狗对外 WiFi/AP 名：YSC-JYML-dt3tfa-5G
感知主机内部 IP：192.168.1.103
运动主机内部 IP：192.168.1.120
运动主机热点网段别名：192.168.137.120
运动主机 p2p0/AP 地址：192.168.2.1
掌机 IP：192.168.2.65
git commit：
主要目标：
```

注意：`172.20.10.2` 是当前 DHCP 租约，不是固定地址。每次现场测试先通过手机热点客户端列表或 `ip -br -4 addr show wlan0` 复核。

## 1. 代码与环境

```bash
cd ~/comp2026_ws
git status
git log -1 --oneline
catkin_make
source devel/setup.bash
```

记录：

```text
git commit：
是否有未提交改动：
catkin_make 结果：
```

## 2. 私有配置

确认本地私有配置存在且不提交：

```bash
cd ~/comp2026_ws/src/tools
cp -n private_robot_access.example.yaml private_robot_access.yaml
nano private_robot_access.yaml
```

至少确认：

```text
robot_hotspot_ip: 172.20.10.2（当前 DHCP 地址）
developer_wifi_ssid: iPhone-hotspot
perception_management_interface: wlan0
perception_wifi_adapter: USB ID 368b:8d85，具体型号未确认，driver=usb
perception_management_mac: 6C:1F:F7:88:0C:02
robot_wifi_ssid: YSC-JYML-dt3tfa-5G
perception_robot_ap_interface: wlan1
perception_robot_ap_ip: 192.168.2.213
perception_robot_ap_adapter: USB ID 368b:8d85，具体型号未确认，driver=usb
perception_robot_ap_mac: 6C:1F:F7:F4:09:76
perception_host: 192.168.1.103
motion_host: 192.168.1.120
motion_host_hotspot_alias: 192.168.137.120
motion_host_p2p: 192.168.2.1
motion_host_ssh_user: ysc
motion_host_ssh_password: 已填写/未填写
sshpass: true/false
sudo: false
sudo_with_password: false
handheld_ip: 192.168.2.65
```

## 3. 上机前预检

未启动 ROS 任务链路前先做基础检查：

```bash
cd ~/comp2026_ws/src/tools
python3 preflight.py --skip-hz
```

记录：

```text
运动主机 ping：
Docker：
模型文件：
~/bags 剩余空间：
异常：
```

如果感知主机没有连上 `iPhone-hotspot`，但电脑能够连接机器狗 AP，可通过运动主机跳板登录感知主机：

```bash
ssh -J ysc@192.168.2.1 ysc@192.168.1.103
```

不要在没有物理控制台或其他备用入口时远程重启 NetworkManager。

记录：

```text
是否能通过 iPhone-hotspot 直接 SSH：
wlan0 当前 DHCP 地址：
是否需要使用运动主机跳板：
```

检查两块 WiFi 是否分别连接正确网络，并确认 `wlan1` 不产生默认路由：

```bash
nmcli dev status
ip -br -4 addr
ip route
nmcli connection show "iPhone-hotspot" | grep -E "autoconnect|interface-name|never-default|route-metric|method"
nmcli connection show "YSC-JYML-dt3tfa-5G" | grep -E "autoconnect|interface-name|never-default|route-metric|method"
ip route get 1.1.1.1
ip route get 192.168.2.1
ip route get 192.168.1.120
ping -c 4 1.1.1.1
getent hosts github.com
ping -c 4 192.168.2.1
ping -c 4 192.168.1.120
```

记录：

```text
wlan0 是否连接 iPhone-hotspot：
wlan0 当前 DHCP 地址：
wlan0 是否承担唯一默认路由：
wlan1 是否连接 YSC-JYML-dt3tfa-5G：
wlan1 IP 是否为 192.168.2.x：
wlan1 ipv4.never-default 是否为 yes：
wlan1 是否未产生默认路由：
Internet/DNS：
192.168.2.1 ping：
192.168.1.120 ping：
```

## 4. 基础链路启动

先只启动底层通讯：

```bash
roslaunch message_transformer message_transformer.launch
```

另开终端检查：

```bash
source ~/comp2026_ws/devel/setup.bash
rostopic echo /lite3/robot_basic_state
rostopic echo /lite3_motion_cmd
```

谨慎测试动作：

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'ensure_stand'"
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'stop'"
```

记录：

```text
起立状态：
底层状态反馈：
异常：
```

## 5. D435i 与扫描

```bash
roslaunch allmovebase camera.launch enable_color:=false depth_fps:=15
roslaunch allmovebase depth2laser.launch
rostopic hz /camera/depth/image_rect_raw
rostopic hz /camera/depth/camera_info
rostopic hz /scan
rostopic echo -n 1 /scan
```

记录：

```text
depth hz：
camera_info hz：
/scan hz：
/scan 是否有有效 ranges：
```

## 6. D435i 倾角标定

机器狗站稳、前方地面尽量干净后运行：

```bash
cd ~/comp2026_ws/src/tools
python3 calibrate_d435i_ground_tilt.py --note field_test_01
```

如果 ROS 相机链路已启动且不想直接打开 RealSense：

```bash
python3 calibrate_d435i_ground_tilt.py --backend ros --note field_test_01_ros
```

记录：

```text
标定文件：src/tools/d435i_ground_tilt_calibration.yaml
pitch_about_optical_x_deg：
roll_about_optical_z_deg：
total_ground_normal_tilt_deg：
是否需要写入 camera2base_tf_tilted.yaml：
```

## 7. 导航可视化

轻量网页看板：

```bash
cd ~/comp2026_ws/src/tools
python3 ros_nav_debug_stream.py --port 8082
```

开发机浏览器打开：

```text
http://172.20.10.2:8082
```

`172.20.10.2` 是当前 DHCP 地址，访问失败时先执行 `ip -br -4 addr show wlan0` 复核。

完整 RViz 诊断：

```bash
roslaunch allmovebase rviz_nav.launch
```

记录：

```text
网页深度图：
地图 + AMCL：
/health 异常：
RViz /scan 是否贴图：
AMCL 粒子是否收敛：
costmap 是否异常：
```

## 8. 启动任务链路后预检

启动目标链路后再跑一次：

```bash
cd ~/comp2026_ws/src/tools
python3 preflight.py --min-free-gb 30
```

记录：

```text
缺失 topic：
异常 hz：
磁盘剩余：
```

## 9. 短录 rosbag

先短录一段估算容量和负载：

```bash
cd ~/comp2026_ws/src/tools
python3 record_rosbag.py --profile full --check-only --hz-check
python3 record_rosbag.py --profile full --split --split-size 4096
```

停止后记录：

```text
bag 文件：
manifest 文件：
录制时长：
文件大小：
CPU/负载感觉：
是否改用 nav/state/no-rgb：
```

回放检查：

```bash
rosparam set use_sim_time true
rosbag play --clock ~/bags/<bag文件>
```

## 10. 掌机/运动主机抓包

先感知主机侧被动抓全端口：

```bash
cd ~/comp2026_ws/src/tools
python3 motion_host_packet_capture.py capture --all-ports --duration 30
```

抓包期间操作掌机：

```text
动作 1：静止俯仰
动作 2：低速运动
动作 3：一边跑一边俯仰
```

如果感知主机抓不到掌机与运动主机通讯，再远程普通权限抓运动主机：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.65 \
  --all-ports \
  --duration 30
```

只有万不得已才临时加 sudo：

```bash
python3 motion_host_packet_capture.py remote-capture --all-ports --sudo --sudo-with-password --duration 30
```

离线检查：

```bash
python3 motion_host_packet_capture.py summarize --pcap ~/packet_captures/<pcap文件> --count 100
python3 motion_host_packet_capture.py decode --pcap ~/packet_captures/<pcap文件> --count 200
```

记录：

```text
pcap 文件：
掌机 IP：192.168.2.65
可疑端口：
是否出现 JoystickChannelFrame：
是否出现 SimpleCMD/ComplexCMD：
是否需要补解码：
```

## 11. 任务测试记录

### 避障单段

```bash
roslaunch allmovebase task_2026_obstacle_test.launch
```

```text
结果：
失败点：
/scan：
AMCL：
move_base status：
bag 文件：
```

### 正式导航识别

```bash
roslaunch allmovebase task_2026_navigation.launch
```

```text
结果：
rec_pose_1：
rec_pose_2：
rec_pose_4：
rec_pose_3：
识别结果：
语音播报：
bag 文件：
```

### 硬编码备用

```bash
roslaunch allmovebase task_2026_hardcoded_motion.launch run_meter_inspection:=false
```

```text
直行距离：
转向角度：
闭环是否需要：
异常：
```

## 12. 收尾

```bash
rostopic pub -1 /lite3_motion_cmd std_msgs/String "data: 'stop'"
```

回收文件：

```text
rosbag：
manifest：
pcap：
preflight json：
标定 yaml：
截图/照片：
```

提交前确认不要提交：

```bash
git status --short
```

不要提交：

```text
src/tools/private_robot_access.yaml
*.bag
*.pcap
runtime 样本图
```
