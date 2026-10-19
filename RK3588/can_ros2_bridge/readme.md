## 如果3588不带canable的识别，进行如下操作可以识别到，并配置波特率，重启也不会影响
# 一、将usb设备改名，不然容易设置错
sudo nano /etc/udev/rules.d/99-canable.rules

SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", SYMLINK+="canable", TAG+="systemd", ENV{SYSTEMD_ALIAS}+="/dev/canable", ENV{SYSTEMD_WANTS}+="canable.service"

# 二、设置自动检查can设备并修改参数，之后就叫can0了，s8指的是波特率为1000000，s6为500000
sudo nano /etc/systemd/system/canable.service

[Unit]
Description=CANable SLCAN interface
BindsTo=dev-canable.device
After=dev-canable.device

[Service]
Type=simple
ExecStart=/usr/bin/slcand -F -o -c -s8 /dev/canable can0
ExecStartPost=/usr/sbin/ip link set can0 up
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target

sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl enable canable.service

# 三、插拔一次，检查是否更改成功
ls -l /dev/canable

# 四、下载can ros2的包
sudo apt install ros-jazzy-ros2-socketcan ros-jazzy-ros2-socketcan-msgs

## 工程模块

| 文件 | 作用 |
|---|---|
| `bridge_node.cpp` | SocketCAN 主节点，收帧后交给上行处理器。 |
| `f407_uplink_controller.cpp` | 解析 F407 上行帧、检查心跳并发布 ROS 数据。 |
| `can_command_controller.cpp` | 订阅控制话题、组装下行帧并周期发送心跳。 |
| `can_transport.cpp`、`rk3588_protocol.cpp` | CAN 收发及协议帧编码、解码。 |

## CAN 与 ROS 数据

| 方向 | CAN ID | 命令 | 数据 / ROS 话题 |
|---|---:|---:|---|
| F407 → 3588 | `0x010` | `0x00` | F407 心跳，每 100 ms 一帧 `[00]`。 |
| 3588 → F407 | `0x110` | `0x00` | 3588 心跳，每 100 ms 一帧 `[00]`。 |
| F407 → 3588 | `0x010` | `0x01` / `0x02` / `0x03` | IMU gyro / accel / pitch-roll-yaw，三个大端 `int16`。 |
| F407 → 3588 | `0x010` | `0x04` / `0x05` | 底盘速度 `(vx, vy, wz)` / 位置 `(x, y, yaw)`，三个大端 `int16`。 |
| 3588 → F407 | `0x110` | `0x01` | 人工使能控制位；bit 0 底盘，bit 1 发球机。 |
| 3588 → F407 | `0x110` | `0x02` | 底盘目标速度 `(vx, vy, wz)`，三个大端 `int16`。 |

F407 心跳状态发布在 `/f407/connected`（`std_msgs/msg/UInt8`，在线 `1`、离线 `0`），每 100 ms 发布一次；200 ms 未收到心跳即判离线并清除桥接侧缓存控制状态。F407 连续 300 ms 未收到 3588 心跳时会关闭全局使能并门控底盘。状态话题可通过 `f407_status_topic` 参数修改。

ROS 数据话题及单位：

- `/imu/gyro`：`Vector3Stamped`，XYZ，°/s；原始值换算为 `raw × 2000 / 32768`。
- `/imu/accel`：`Vector3Stamped`，XYZ，g；换算为 `raw × 16 / 32768`。
- `/imu/euler`：`Vector3Stamped`，x=pitch、y=roll、z=yaw，°；换算为 `raw × 180 / 32768`。
- `/chassis/velocity`：`Vector3Stamped`，vx/vy/wz，m/s、m/s、rad/s。
- `/chassis/position`：`Vector3Stamped`，x/y/yaw，m、m、rad。

IMU 和底盘话题各自按收到的 CAN 命令独立发布。里程计由 F407 的轮速积分得到，未融合 IMU，误差会随时间累积。话题名及 `frame_id` 可通过对应 ROS 参数修改。

## 常用操作

从仓库根目录启动桥接节点（脚本会增量构建，默认使用 `can0`）：

```bash
./RK3588/can_ros2_bridge/run.sh
```

查看在线状态或传感器数据：

```bash
ros2 topic echo /f407/connected
ros2 topic echo /imu/euler
ros2 topic echo /chassis/velocity
ros2 topic echo /chassis/position
```

底盘速度从 `/cmd_vel`（`geometry_msgs/msg/Twist`）下发，使用 `linear.x`、`linear.y`、`angular.z`，单位为 m/s、m/s、rad/s。控制端应持续发布，例如以 20 Hz 下发 `vx=0.1`、`vy=0`、`wz=0.2`：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.2}}"
```

人工使能话题为 `/manual_enable/chassis` 和 `/manual_enable/launcher`（`std_msgs/msg/UInt8`，`data` 为 `0` 或 `1`）。例如：

```bash
ros2 topic pub --once /manual_enable/chassis std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /manual_enable/launcher std_msgs/msg/UInt8 "{data: 1}"
```

其他 CAN 接口可通过 `CAN_INTERFACE` 指定；默认不打印逐帧日志，设置 `log_can_frames:=1` 可开启：

```bash
CAN_INTERFACE=can1 ./RK3588/can_ros2_bridge/run.sh
./RK3588/can_ros2_bridge/run.sh -p log_can_frames:=1
```
