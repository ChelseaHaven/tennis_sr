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

- `can_bridge_node.cpp`：主程序，协调 CAN 收发、协议解析和心跳判断。
- `socketcan_transport.cpp`：经典 CAN 通用收发，校验标准/扩展 ID 和 8 字节数据长度。
- `rk3588_can_protocol.cpp`：3588 与单片机约定的 CAN ID、心跳识别和底盘使能帧编码。
- `heartbeat_monitor.cpp`：根据 CAN ID `0x010` 的相邻心跳时间判断在线状态，超时阈值为 200 ms。

底盘使能帧使用 CAN ID `0x110`，数据为 `[0x00, 0x01]`；不使能帧数据为 `[0x00, 0x00]`。心跳识别要求标准帧 CAN ID 为 `0x010`、数据长度恰好为 1，且数据字节为 `0x00`。

构建并启动：

```bash
cd ~/chenzhihan/ros_project/can_ros2_bridge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select can_ros2_bridge
source install/setup.bash
ros2 run can_ros2_bridge can_receiver_node
```
