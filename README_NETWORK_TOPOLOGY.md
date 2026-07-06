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

```text
开发机
  学校 WiFi / Internet
  开发机 WiFi/热点名：jj6
  Windows 热点网段：192.168.137.0/24
    |
    | wlan0
    v
感知主机 Jetson / Ubuntu 20.04 / ROS Noetic
  wlan0: 192.168.137.144/24
  wlan0: 连接开发机 WiFi/热点 jj6，作为默认路由
  eth0 : 192.168.1.103/24
  wlan1: 192.168.2.213/24
  wlan1: 连接机器狗对外 WiFi/AP YSC-JYML-gb9zfq-5G，仅用于访问 192.168.2.0/24 内部网
    |
    | eth0 <-> eth1
    v
运动主机 RK3588 / Ubuntu 20.04 / QNX 运动控制侧
  eth1: 192.168.1.120/24
  eth1: 192.168.137.120/24
  p2p0: 192.168.2.1/24
  对外 WiFi/AP 名：YSC-JYML-gb9zfq-5G
    |
    | p2p0/AP
    v
掌机/平板
  掌机: 192.168.2.62
  平板: 192.168.2.16
```

## 2. 已确认地址

| 设备/接口              | IP                     | 说明                                               |
| ---------------------- | ---------------------- | -------------------------------------------------- |
| 开发机热点网关         | `192.168.137.1`      | 感知主机默认路由网关                               |
| 感知主机 `wlan0`     | `192.168.137.144`    | 连接开发机 WiFi/热点 `jj6`，保留默认路由          |
| 感知主机 `eth0`      | `192.168.1.103`      | 机器狗内部网                                       |
| 感知主机 `wlan1`     | `192.168.2.213`      | 连接机器狗对外 WiFi/AP，仅访问 `192.168.2.0/24` |
| 运动主机 `eth1`      | `192.168.1.120`      | 感知主机访问运动主机的主要地址                     |
| 运动主机 `eth1` 别名 | `192.168.137.120`    | 运动主机在热点网段上的地址                         |
| 运动主机 `p2p0`      | `192.168.2.1`        | 掌机/平板连接的 AP/P2P 网段                        |
| 运动主机对外 WiFi/AP   | `YSC-JYML-gb9zfq-5G` | 掌机/平板连接                                      |
| 开发机 WiFi/热点       | `jj6`                | 开发机侧网络名                                     |
| 掌机                   | `192.168.2.62`       | 已确认                                             |
| 平板                   | `192.168.2.16`       | 已确认                                             |

## 3. SSH 入口

用户名：

```text
ysc
```

密码：

```text
<占位符>
```

当前感知主机和运动主机的 SSH 用户名/密码相同。sudo 密码与 SSH 登录密码相同。

推荐从感知主机登录运动主机：

```bash
ssh ysc@192.168.1.120
```

也可以通过运动主机 p2p0 地址登录，同一台机器：

```bash
ssh ysc@192.168.2.1
```

开发机访问感知主机时，使用热点网段地址：

```bash
ssh ysc@192.168.137.144
```

如果感知主机没有自动连上开发机 WiFi/热点，可用平板连接机器狗内部网络后登录感知主机内部地址：

```bash
ssh ysc@192.168.1.103
```

## 4. 感知主机路由

实测感知主机：

```text
eth0 : 192.168.1.103/24
wlan0: 192.168.137.144/24
wlan1: 192.168.2.213/24
default via 192.168.137.1 dev wlan0
192.168.1.0/24 dev eth0
192.168.2.0/24 dev wlan1
192.168.137.0/24 dev wlan0
```

含义：

- 访问互联网和开发机热点网段走 `wlan0`。
- 访问运动主机 `192.168.1.120` 走 `eth0`。
- 访问机械臂主控、掌机、平板所在的 `192.168.2.0/24` 内部网走 `wlan1`。
- `wlan1` 的连接配置必须保持 `ipv4.never-default yes`，避免抢默认路由导致开发机/Internet 访问异常。
- 感知主机现在能直接访问 `192.168.2.0/24`，但掌机/平板与运动主机的直连流量仍不一定经过感知主机。

2026-05-31 实测连接配置：

```text
wlan0 -> jj6
  IP: 192.168.137.144/24
  default route: yes, metric 600

wlan1 -> YSC-JYML-gb9zfq-5G
  IP: 192.168.2.213/24
  route: 192.168.2.0/24 dev wlan1 metric 700
  ipv4.never-default: yes
  ipv6.method: ignore
```

现场配置命令记录：

```bash
sudo nmcli dev wifi connect "YSC-JYML-gb9zfq-5G" password "<机器狗AP密码>" ifname wlan1 bssid 2E:C3:E6:E3:5D:E9
sudo nmcli connection modify "YSC-JYML-gb9zfq-5G" connection.autoconnect yes
sudo nmcli connection modify "YSC-JYML-gb9zfq-5G" connection.interface-name wlan1
sudo nmcli connection modify "YSC-JYML-gb9zfq-5G" ipv4.never-default yes
sudo nmcli connection modify "YSC-JYML-gb9zfq-5G" ipv4.route-metric 700
sudo nmcli connection modify "YSC-JYML-gb9zfq-5G" ipv6.method ignore
sudo nmcli connection down "YSC-JYML-gb9zfq-5G"
sudo nmcli connection up "YSC-JYML-gb9zfq-5G"
```

验证命令：

```bash
nmcli dev status
ip addr
ip route
nmcli connection show "YSC-JYML-gb9zfq-5G" | grep -E "autoconnect|interface-name|never-default|route-metric|method"
ping -c 4 192.168.2.1
ping -c 4 192.168.1.120
```

如果感知主机没有自动连上开发机 WiFi/热点，可从平板进入机器狗内部网络后 SSH 到 `192.168.1.103`，谨慎重启 NetworkManager：

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
192.168.1.0/24 dev eth1
192.168.2.0/24 dev p2p0
192.168.137.0/24 dev eth1
```

运动主机 `ip neigh show dev p2p0` 可用于确认掌机/平板：

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

掌机/平板与运动主机大概率不经过感知主机，建议远程登录运动主机，在 `p2p0` 上抓。

抓掌机：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.62 \
  --all-ports \
  --duration 60 \
  --output ~/packet_captures/handheld_192_168_2_62.pcap
```

抓平板：

```bash
python3 motion_host_packet_capture.py remote-capture \
  --interface p2p0 \
  --filter-host 192.168.2.1 \
  --host 192.168.2.16 \
  --all-ports \
  --duration 60 \
  --output ~/packet_captures/tablet_192_168_2_16.pcap
```

默认远程抓包不使用 sudo。若普通权限无法抓包，优先退回感知主机侧抓包；确实万不得已再临时加：

```bash
--sudo --sudo-with-password
```

## 8. 常用网页入口

工具在感知主机运行时，开发机浏览器通常使用热点地址访问：

```text
D435i 彩色预览:       http://192.168.137.144:8080
广角相机预览:         http://192.168.137.144:8081
ROS 导航调试看板:     http://192.168.137.144:8082
```

## 9. 备注

- `192.168.2.1`、`192.168.1.120`、`192.168.137.120` 是同一台运动主机的不同地址。
- 掌机是 `192.168.2.62`，平板是 `192.168.2.16`。
- 感知主机通过 `wlan1` 连入机器狗 AP 后可访问 `192.168.2.0/24`，用于后续和机械臂主控通信；但掌机/平板与运动主机直连流量仍不一定经过感知主机。
- 不建议在运动主机上长期启用 sudo 抓包或修改系统服务。
