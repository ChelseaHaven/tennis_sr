# 麦轮规划 · ROS 2 Jazzy

所有命令都从仓库根目录 `tennis_sr` 执行。仿真和实车使用不同启动脚本，**不要同时运行**。

## 编译

首次使用或修改工作区代码后编译：

```bash
./RK3588/mecanum_ros2_ws/build.sh
```

## 仿真

```bash
./RK3588/mecanum_ros2_ws/run_interactive.sh
```

默认打开 RViz，Gazebo 在后台运行。需要 Gazebo 窗口时：

```bash
./RK3588/mecanum_ros2_ws/run_interactive.sh gui:=true
```

在启动终端按 `Ctrl+C` 退出仿真。仿真默认使用 `ROS_DOMAIN_ID=72`。远程显示到电脑时，
从电脑用 `ssh -Y pi@192.168.1.88` 登录后运行上述脚本，并确保电脑端 X Server 已启动。

## 实车

确认 F407 固件和 CAN 已就绪后，一条命令启动桥接、规划和 RViz：

```bash
./RK3588/mecanum_ros2_ws/run_real_robot_all.sh
```

实车默认使用 `ROS_DOMAIN_ID=0`，不会自动使能底盘。先在另一个终端确认 F407 在线、
里程计正常：

```bash
export ROS_DOMAIN_ID=0
ros2 topic echo /f407/connected --once
ros2 topic hz /odom
```

确认安全后手动使能底盘：

```bash
ros2 topic pub --once /manual_enable/chassis std_msgs/msg/UInt8 '{data: 1}'
```

撤销使能：

```bash
ros2 topic pub --once /manual_enable/chassis std_msgs/msg/UInt8 '{data: 0}'
```

一体化启动默认关闭逐帧 CAN 日志；排查 CAN 数据时可打开：

```bash
CAN_BRIDGE_LOG_FRAMES=1 ./RK3588/mecanum_ros2_ws/run_real_robot_all.sh
```

按 `Ctrl+C` 会停止桥接和规划节点。实车速度上限默认 `0.3 m/s`，加速度上限 `0.5 m/s²`；
可通过 `v_max:=... a_max:=...` 调整。

## RViz 点选

Fixed Frame 使用 `odom`。用 **Publish Point** 添加途经点，再用 **2D Goal Pose** 选择终点。
仿真速度上限默认 `4.5 m/s`，可通过 `v_max:=0.6 a_max:=1.0` 降低。

常用话题：`/odom`、`/cmd_vel`、`/track_traj`、`/goal_pose`。实车还可查看
`/f407/connected`、`/chassis/position` 和 `/chassis/velocity`。
