# 机器狗网络拓扑

本文记录当前实机测试时确认的网络结构。

```text
README_NETWORK_TOPOLOGY.md
```

工具读取的私有配置仍放在：

```text
src/tools/private_robot_access.yaml
```

## 1. 总体结构

当前感知主机配置了两块独立无线网卡，并保留与运动主机连接的内部有线网络。

`wlan0` 当前地址由开发路由器 DHCP 分配，可能随网卡、租约或路由器配置发生变化。本文记录的是当前实测地址，不代表固定静态地址。

```text
开发路由器 / Internet
  SSID: Bad_Puppy
  网关: 192.168.31.1
    |
    | wlan0
    v
感知主机 Jetson / Ubuntu 20.04 / ROS Noetic
  wlan0: 192.168.31.175/24，DHCP
  wlan0: 连接 Bad_Puppy，负责 SSH、Git、Internet 和默认路由
  eth0 : 192.168.1.103/24，机器狗内部有线网
  wlan1: 192.168.2.213/24，DHCP
  wlan1: 连接机器狗对外 WiFi/AP YSC-JYML-dt3tfa-5G
    |
    | eth0 <-> eth1
    v
运动主机 RK3588 / Ubuntu 20.04 / QNX 运动控制侧
  eth1: 192.168.1.120/24
  eth1: 192.168.137.120/24（同一接口的第二地址）
  p2p0: 192.168.2.1/24
  对外 WiFi/AP 名：YSC-JYML-dt3tfa-5G
    |
    | p2p0/AP
    v
掌机及感知主机 wlan1
  感知主机 wlan1: 192.168.2.213
  掌机: 192.168.2.65
```

## 2. 已确认地址

| 设备/接口                | IP 或名称                  | 说明                                                    |
| ------------------------ | -------------------------- | ------------------------------------------------------- |
| 开发路由器网关           | `192.168.31.1`             | 感知主机 `wlan0` 的默认路由网关                         |
| 开发路由器/WiFi          | `Bad_Puppy`                | 开发机和感知主机管理网络                                |
| 感知主机 `wlan0`         | `192.168.31.175/24`        | 当前 DHCP 地址，负责 SSH、Git、Internet 和默认路由      |
| 感知主机 `eth0`          | `192.168.1.103/24`         | 机器狗内部有线控制网                                    |
| 感知主机 `wlan1`         | `192.168.2.213/24`         | 连接机器狗对外 WiFi/AP                                  |
| 运动主机 `eth1`          | `192.168.1.120/24`         | 感知主机访问运动主机的主要地址                          |
| 运动主机 `eth1` 第二地址 | `192.168.137.120/24`       | 当前实测仍存在，与 `192.168.1.120` 位于同一物理接口     |
| 运动主机 `p2p0`          | `192.168.2.1/24`           | 机器狗对外 AP/P2P 网关                                  |
| 运动主机对外 WiFi/AP     | `YSC-JYML-dt3tfa-5G`      | 感知主机 `wlan1` 和厂家掌机连接                         |
| 掌机                     | `192.168.2.65`             | 已确认连接机器狗对外 WiFi/AP                            |

## 3. SSH 入口

感知主机和运动主机当前 SSH 用户名均为：

```text
ysc
```

密码不在仓库文档中记录：

```text
<占位符>
```

当前感知主机和运动主机的 SSH 用户名/密码相同。sudo 密码与 SSH 登录密码相同。

开发机连接 `Bad_Puppy` 后，使用感知主机 `wlan0` 当前 DHCP 地址登录：

```bash
ssh ysc@192.168.31.175
```

`192.168.31.175` 是当前 DHCP 租约。地址变化时，先在路由器客户端列表中查询，或在感知主机执行：

```bash
ip -br -4 addr show wlan0
```

感知主机通过内部有线网登录运动主机：

```bash
ssh ysc@192.168.1.120
```

电脑连接机器狗对外 WiFi `YSC-JYML-dt3tfa-5G` 后，可以登录运动主机 `p2p0` 地址：

```bash
ssh ysc@192.168.2.1
```

如果开发管理网络不可用，可使用运动主机作为跳板登录感知主机内部有线地址：

```bash
ssh -J ysc@192.168.2.1 ysc@192.168.1.103
```

## 4. 感知主机路由

实测感知主机：

```text
eth0 : 192.168.1.103/24
wlan0: 192.168.31.175/24
wlan1: 192.168.2.213/24

default via 192.168.31.1 dev wlan0 proto dhcp metric 600
192.168.1.0/24 dev eth0 proto kernel scope link src 192.168.1.103 metric 100
192.168.2.0/24 dev wlan1 proto kernel scope link src 192.168.2.213 metric 700
192.168.31.0/24 dev wlan0 proto kernel scope link src 192.168.31.175 metric 600
```

含义：

- 访问 Internet 和开发路由器网段走 `wlan0`，当前默认网关为 `192.168.31.1`。
- 访问运动主机 `192.168.1.120` 走 `eth0`。
- 访问运动主机 AP、掌机及其他 `192.168.2.0/24` 设备走 `wlan1`。
- `wlan1` 已配置 `ipv4.never-default yes` 和 `ipv6.never-default yes`，不会成为 IPv4 或 IPv6 默认路由。
- `wlan1` 的 IPv4 路由 metric 为 `700`；`192.168.2.0/24` 直连路由保持可用。
- 掌机与运动主机通过机器狗 AP 直连，其通信流量不一定经过感知主机。

2026-08-07 实测连接配置：

```text
wlan0 -> Bad_Puppy
  IP: 192.168.31.175/24（DHCP 当前租约）
  gateway: 192.168.31.1
  connection.autoconnect: yes
  ipv4.method: auto
  ipv4.never-default: no
  effective default route metric: 600

wlan1 -> YSC-JYML-dt3tfa-5G
  IP: 192.168.2.213/24
  connection.autoconnect: yes
  ipv4.method: auto
  ipv4.never-default: yes
  ipv4.route-metric: 700
  ipv6.method: auto
  ipv6.never-default: yes
```

`wlan1` 内部网络连接配置命令记录：

```bash
sudo nmcli connection modify "YSC-JYML-dt3tfa-5G" \
  connection.autoconnect yes \
  connection.interface-name wlan1 \
  ipv4.never-default yes \
  ipv4.route-metric 700 \
  ipv6.never-default yes
sudo nmcli connection up "YSC-JYML-dt3tfa-5G" ifname wlan1
```

验证命令：

```bash
nmcli dev status
ip -br -4 addr
ip route
nmcli connection show "YSC-JYML-dt3tfa-5G" | grep -E "autoconnect|interface-name|never-default|route-metric|method"
ip route get 1.1.1.1
ip route get 192.168.2.1
ping -c 4 192.168.2.1
ping -c 4 192.168.1.120
```

只有在已经具备运动主机跳板或物理控制台等备用入口时，才谨慎重启 NetworkManager：

```bash
sudo systemctl restart NetworkManager
```

该命令会短暂重置感知主机网络连接，只在 WiFi 自连异常时使用。

## 5. 运动主机路由

实测运动主机：

```text
eth1: 192.168.1.120/24
eth1: 192.168.137.120/24
p2p0: 192.168.2.1/24
default via 192.168.137.120 dev eth1
192.168.1.0/24 dev eth1 proto kernel scope link src 192.168.1.120 metric 100
192.168.2.0/24 dev p2p0 proto kernel scope link src 192.168.2.1 metric 600
192.168.137.0/24 dev eth1 proto kernel scope link src 192.168.137.120 metric 100
```

含义：

- `192.168.1.120` 是感知主机通过内部有线网访问运动主机的主要地址。
- `192.168.137.120` 是同一 `eth1` 接口上仍然存在的第二地址，不是独立网卡。
- `192.168.2.1` 是运动主机对外 AP/P2P 地址。
- `default via 192.168.137.120 dev eth1` 是运动主机当前实测静态配置。该网关写法较特殊，但本项目只记录现状；没有厂家依据和明确需求时，不修改运动主机路由。

运动主机 `ip neigh show dev p2p0` 可用于确认掌机：

```bash
ip neigh show dev p2p0
```

## 6. 端口记录

| 端口          | 方向/用途                                     | 来源                  |
| ------------- | --------------------------------------------- | --------------------- |
| `43893/udp` | 感知主机 -> 运动主机，开发者 UDP 控制目标端口 | 厂家文档 /`ros2qnx` |
| `43894/udp` | `ros2qnx` 本地绑定端口                      | 当前工程              |
| `43897/udp` | 运动主机 -> 感知主机，`qnx2ros` 状态接收    | 当前工程              |
| `43899/udp` | `nx2app` / APP / 掌机相关接收端口           | 当前工程              |
| `8554/tcp`  | 运动主机 RTSP 广角相机流                      | 厂家文档 / 去年工程   |

掌机与运动主机直连通讯端口尚未完全确认，需通过 `p2p0` 抓包分析。

## 7. 抓包建议

感知主机视角，只能看到经过感知主机网卡的流量：

```bash
cd ~/comp2026_ws/src/tools
python3 motion_host_packet_capture.py capture --all-ports --duration 30
```

掌机与运动主机直连流量大概率不经过感知主机，建议远程登录运动主机，在 `p2p0` 上抓。

抓掌机：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.65 \
  --all-ports \
  --duration 60 \
  --output ~/packet_captures/handheld_192_168_2_65.pcap
```

默认远程抓包不使用 sudo。若普通权限无法抓包，优先退回感知主机侧抓包；确实万不得已再临时加：

```bash
--sudo --sudo-with-password
```

## 8. 常用网页入口

工具在感知主机运行时，开发机连接 `Bad_Puppy` 后，使用感知主机 `wlan0` 当前 DHCP 地址访问：

```text
D435i 彩色预览:       http://192.168.31.175:8080
广角相机预览:         http://192.168.31.175:8081
ROS 导航调试看板:     http://192.168.31.175:8082
```

`192.168.31.175` 是当前 DHCP 租约，不是固定地址。无法访问时，先在感知主机执行：

```bash
ip -br -4 addr show wlan0
```

## 9. 备注

- `192.168.2.1`、`192.168.1.120`、`192.168.137.120` 是同一台运动主机的不同地址。
- 掌机当前地址是 `192.168.2.65`。
- 感知主机通过 `wlan1` 连入机器狗 AP 后可访问 `192.168.2.0/24`，用于掌机链路观察和后续机械臂主控通信；但掌机与运动主机的直连流量仍不一定经过感知主机。
- 不建议在运动主机上长期启用 sudo 抓包或修改系统服务。
