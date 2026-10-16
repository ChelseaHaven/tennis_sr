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

- `bridge_node.cpp`：主程序，协调 CAN 收发、协议处理和心跳。
- `can_data_controller.cpp`：订阅底盘、发球机使能话题，维护控制位并调用 CAN 发送回调；同时解析 MCU 的 IMU 和底盘里程计 CAN 帧、换算并发布向量话题。
- `can_transport.cpp`：经典 CAN 通用收发。
- `rk3588_protocol.cpp`：3588 与单片机间的 CAN 协议和帧编码。
- `heartbeat.cpp`：`HeartbeatWatchdog` 判断心跳在线状态；`HeartbeatController` 在状态变化时调用回调，主程序通过回调发送全局使能 CAN 命令。超时阈值为 200 ms。

全局使能由心跳监控自动控制，使用 CAN ID `0x110`、命令 `0x00`，数据为 `[0x00, 0x01]`（使能）或 `[0x00, 0x00]`（失能）。心跳识别要求标准帧 CAN ID 为 `0x010`、数据长度恰好为 1，且数据字节为 `0x00`。

底盘目标速度通过 `/cmd_vel`（`geometry_msgs/msg/Twist`）下发，默认取 `linear.x`、`linear.y`、`angular.z`，单位为 m/s、m/s、rad/s。桥接节点分别乘以 1000、四舍五入并饱和到 `int16` 范围，再通过标准 CAN ID `0x110`、命令 `0x02` 和三个大端有符号 `int16` 发送；F407 协议单位为 mm/s、mm/s、mrad/s。控制端应持续发布速度指令。

例如以 20 Hz 下发 `vx=0.1 m/s`、`vy=0`、`wz=0.2 rad/s`：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.2}}"
```

IMU 数据通过标准 CAN ID `0x010` 上发，命令字节后跟三个大端有符号 `int16`：`0x01` 为 gyro XYZ，`0x02` 为 accel XYZ，`0x03` 为 pitch/roll/yaw。桥接节点收到每条命令后独立发布对应的 `geometry_msgs/msg/Vector3Stamped` 话题：`/imu/gyro`（°/s）、`/imu/accel`（g）和 `/imu/euler`（x=pitch、y=roll、z=yaw，°），与 F407 的 `imu` 结构体单位一致，默认 `frame_id=imu_link`。换算比例分别为 `gyro=raw*2000/32768`、`accel=raw*16/32768`、`angle=raw*180/32768`。话题和 frame ID 可用 `gyro_topic`、`accel_topic`、`attitude_topic`、`imu_frame_id` 参数覆盖。

底盘里程计通过相同 CAN ID 上发，使用同一通用三大端有符号 `int16` 格式：`0x04` 为车体速度 `(vx, vy, wz)`，F407 发送值依次为 mm/s、mm/s、mrad/s；`0x05` 为全局位置 `(x, y, yaw)`，发送值依次为 cm、cm、mrad。F407 每个 20 ms 底盘周期反解四个车轮实测速度并发送一组速度和位置，位置和航向由轮速积分得到，暂未融合 IMU，因此会随时间累积里程计误差。桥接节点使用 `/chassis/velocity` 和 `/chassis/position` 两个 `geometry_msgs/msg/Vector3Stamped` 话题，分别换算成 m/s、m/s、rad/s 和 m、m、rad；默认 `frame_id` 分别为 `base_link` 与 `odom`。话题和坐标系可通过 `chassis_velocity_topic`、`chassis_position_topic`、`chassis_frame_id`、`odom_frame_id` 参数覆盖。

查看数据：

```bash
ros2 topic echo /imu/euler
ros2 topic echo /chassis/velocity
ros2 topic echo /chassis/position
```

人工使能通过语义明确的 ROS 话题控制：`/manual_enable/chassis` 控制底盘，`/manual_enable/launcher` 控制发球机。话题类型为 `std_msgs/msg/UInt8`，`data` 只能是 `0` 或 `1`。桥接节点把底盘状态映射到控制字 bit 0、把发球机状态映射到 bit 1，再合并为 4 字节控制位，通过 CAN ID `0x110`、命令 `0x01` 发送。重复收到相同状态时不重复发送，状态变化后才发送新帧；单片机心跳超时时人工控制位清零，心跳恢复时重新发送当前人工状态。

例如，分别发布以下消息会使能底盘和发球机，对应 CAN 数据 `[0x01, 0x03, 0x00, 0x00, 0x00]`：

```bash
ros2 topic pub --once /manual_enable/chassis std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /manual_enable/launcher std_msgs/msg/UInt8 "{data: 1}"
```

构建并启动：

```bash
cd ~/chenzhihan/tennis_project/tennis_sr
source /opt/ros/jazzy/setup.bash
colcon build --packages-select can_ros2_bridge
source install/setup.bash
ros2 run can_ros2_bridge can_receiver_node
```

也可以直接运行脚本；启动前会执行增量构建，默认使用 `can0`：

```bash
./RK3588/can_ros2_bridge/run.sh
```

使用其他 SocketCAN 接口时可指定 `CAN_INTERFACE`，例如：

```bash
CAN_INTERFACE=can1 ./RK3588/can_ros2_bridge/run.sh
```
