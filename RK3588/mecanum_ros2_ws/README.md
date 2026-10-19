# 麦轮规划工程 · Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic

这是 `../mecanum_ws` 的原生 ROS 2 移植工程，旧 Noetic 工程保留供对照。
运行链路为：RViz2 点选 → CasADi/IPOPT 两阶段时间最优规划 → 最近轨迹节点速度前馈
→ `/cmd_vel` → Harmonic 底盘力控制 → `/odom`。

## 安装与编译

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-plugin-vendor ros-jazzy-rviz2 ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions python3-venv python3-pip libx11-dev libxrandr-dev
./RK3588/mecanum_ros2_ws/build.sh
```

`build.sh` 使用系统 Python 3.12 编译 ROS 消息，并在 `.venv` 中安装 CasADi。
不需要 Docker、Noetic、catkin 或 ROS 1 bridge。启动脚本自动加载 Jazzy、工作空间
和 CasADi，路径相对于脚本解析，可移动工程目录。不要在已加载 Noetic 的终端运行。

## X11 启动

在本机桌面终端执行：

```bash
./RK3588/mecanum_ros2_ws/run_interactive.sh
```

默认只开 RViz2，Gazebo 在后台进行真实物理仿真。需要 Gazebo 窗口时：

```bash
./RK3588/mecanum_ros2_ws/run_interactive.sh gui:=true
```

退出时在启动仿真的同一终端按 `Ctrl+C`，launch 会停止它直接启动的 Gazebo 和 ROS 节点。
若是旧启动遗留的 Gazebo 服务端（`pgrep -af 'gz sim'` 仍能看到），需对对应 PID 单独发送
`kill -INT <PID>`；新启动方式不会再通过 shell 包装 Gazebo 进程。

脚本默认使用 `ROS_DOMAIN_ID=72`，避免与板上默认域的实车节点混用 `/cmd_vel`。
其他终端查看仿真话题时也需设置 `ROS_DOMAIN_ID=72`，可通过环境变量覆盖。

从远端显示到你的电脑时，请在客户端启用 X Server，并重新建立带 X11 转发的连接：

```bash
ssh -Y pi@192.168.1.88
./RK3588/mecanum_ros2_ws/run_interactive.sh
```

`echo "$DISPLAY"` 应有值（SSH 转发通常为 `localhost:10.0`）。保留 SSH 分配的
`DISPLAY` / Xauthority，不要改为 `:0`。`DISPLAY=:0` 表示显示在板子的本地桌面，
不会显示到 SSH 客户端。脚本强制 Qt 使用 `xcb`。遇到 OpenGL 驱动错误可尝试：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./RK3588/mecanum_ros2_ws/run_interactive.sh
```

如果当前是普通 SSH 终端，但希望窗口显示在板子的本机桌面，可使用
`./RK3588/mecanum_ros2_ws/run_interactive.sh --local-display`。脚本会选择 `:0` 并查找当前用户的
XWayland 授权文件；它不会把窗口显示到你的电脑。

无需放开 `xhost +`。只有后台仿真时：`./RK3588/mecanum_ros2_ws/run_interactive.sh rviz:=false`。

工程自带 `rviz_x11` 启动器：对没有任何 XRandR 视频模式的无显示器 XWayland，
仅在 RViz 进程内启用 OGRE 1.12 的屏幕尺寸回退路径，避免启动段错误。
正常显示器/SSH X11 转发有有效模式时行为不变。可设 `MECANUM_X11_GUARD=0` 禁用。
此处理不创建显示器，也不将本地窗口转发到 SSH 客户端；远程查看仍需 `ssh -Y`。

## 点选规划

RViz Fixed Frame 使用 `odom`，顶视图中：

1. 用 **Publish Point** 依次添加路点，红色球表示待规划路点。
2. 用 **2D Goal Pose** 点击终点，触发规划，蓝线表示规划路径。
3. 小车沿路径运动，终端速度约束为零。成功规划后清空待选路点；失败则保留。

保留旧 `/move_base_simple/goal` 输入，同时支持 RViz2 默认 `/goal_pose`。
点击位置需在 ±19.5 m 内；原优化器的位置状态界限为 ±20 m。默认每轴速度上限
4.5 m/s、加速度上限 3.0 m/s²，与旧 interactive launch 的参数一致。
可用 `v_max:=0.6 a_max:=1.0` 降低规划限制。

## 包和接口

| 包 | 功能 |
| --- | --- |
| `fast_fly_interfaces` | ROS 2 `TrackTraj` 消息，`start_time` 改为 `builtin_interfaces/Time` |
| `fast_fly_ros2` | rclpy 规划/跟踪、odom→TF、launch、RViz2、机器人网格与 URDF |
| `mecanum_gz` | Harmonic `System` 插件，速度误差驱动底盘受力，支持横移及命令超时制动 |

主要话题：`/clicked_point`、`/goal_pose`、`/track_traj`、`/planed_path`（沿用旧拼写）、
`/plan_markers`、`/cmd_vel`、`/odom`、`/joint_states`、`/clock`、`/tf`、`/robot_description`。
ROS 与 Gazebo 通过 `ros_gz_bridge` 通信；所有显示/规划节点使用仿真时间。

消息约定：N 个 `dt` 对应 N 段轨迹，position/velocity/orientation/angular 各为 N+1 个节点，
包含当前初始状态与最终静止状态。这修正了旧脚本缺少初始节点、重复最后节点的表示方式。
跟踪仍采用最近节点速度前馈，不包含 MPC；实测终点偏差需结合速度、间距与底盘响应评估。

## 物理模型与迁移范围

复用 nexus 原网格、质量、惯量、轮子关节与碰撞体。沿用旧工程的**底盘施力模型**：
轮子摩擦为零，以底盘速度误差施加平面力，碰撞与重力由 Harmonic 物理引擎计算。
它不是逐滚子麦克纳姆轮接触模型。新插件对正负速度对称限幅，并按质量限制最大平面力；
旧插件只用 `min` 限制正方向的行为已修正。

移植了 README 中的麦轮交互规划主链路。原无人机 PX4/MPC 节点不属于该链路；
旧模型中未被规划使用的超声波/激光 ROS 1 插件未迁移，传感器安装结构保留。
原先依赖外部 `model://target` 的标记板以盒状几何体替代，避免在线模型依赖。
原规划器与 Gates 来自 `../mecanum_ws/src/fast_fly`，模型来自 nexus description；
这些文件沿用各自上游许可条件，新增 Harmonic 插件为 Apache-2.0。

## 接实车

实车使用独立启动文件 `real_robot.launch.py`，不启动 Gazebo。它在本工作区内将桥接节点
提供的 `/chassis/position`、`/chassis/velocity`（`Vector3Stamped`）转换为规划链路需要的
`/odom`（`nav_msgs/Odometry`），并保留已有的 `/cmd_vel` 下发链路。位置按米、航向按弧度；
速度为车体坐标系 m/s 和 rad/s。转换器以最新位置和速度组装 odometry，任一上行数据
超过 0.2 秒未更新就停止发布，tracker 的 odometry watchdog 会随之发零速。

工作区首次使用或代码有改动时，从仓库根目录编译：

```bash
./RK3588/mecanum_ros2_ws/build.sh
```

然后从仓库根目录用一条命令启动 CAN 桥接、实车规划和 RViz：

```bash
./RK3588/mecanum_ros2_ws/run_real_robot_all.sh
```

该脚本默认使用 `ROS_DOMAIN_ID=0`，并且不会发送底盘使能。按 `Ctrl+C` 会同时停止桥接与
实车规划节点。若需覆盖 ROS 域，可在启动前设置 `ROS_DOMAIN_ID`；桥接和规划会使用相同域。
仿真启动脚本 `./RK3588/mecanum_ros2_ws/run_interactive.sh` 默认使用域 72。实车默认规划上限为
0.3 m/s、0.5 m/s²，可以通过 `v_max:=... a_max:=...` 调整。确认底盘周围安全后，需在
另一个终端中显式使能底盘：

```bash
ros2 topic pub --once /manual_enable/chassis std_msgs/msg/UInt8 '{data: 1}'
```

手动撤销使能时发布 `'{data: 0}'`。全局 CAN 心跳和手动底盘使能都必须正常，规划器才会
驱动车体。可用 `/f407/connected` 检查 F407 心跳，用 `/odom` 查看合成里程计；RViz 的
Fixed Frame 为 `odom`。真实底盘的编码器比例、坐标方向、航向符号以及可用速度/加速度
需要上车校准，当前尚未进行硬件闭环验证。仿真与实车不要同时启动，也不要让多个节点
同时发布 `/cmd_vel`。

## 端到端验证

先在终端 A 启动隔离仿真：

```bash
ROS_DOMAIN_ID=72 GZ_PARTITION=tennis_migration_test ./RK3588/mecanum_ros2_ws/run_interactive.sh rviz:=false
```

在终端 B（同一工程目录）执行：

```bash
source /opt/ros/jazzy/setup.bash
source RK3588/mecanum_ros2_ws/install/setup.bash
ROS_DOMAIN_ID=72 ROS_LOG_DIR="$PWD/RK3588/mecanum_ros2_ws/.ros/log" python3 RK3588/mecanum_ros2_ws/tests/verify_sim.py
```

测试检查四轮关节、仿真时钟、负向/侧向运动、指令超时停车、途经点规划、
轨迹消息长度、终端速度约束和终点停车。测试只对新启动的空场景运行一次。

2026-10-15 在当前 Ubuntu 24.04 / ARM64 环境实测：三个包编译成功；
上述端到端测试通过，负向双轴速度约 −0.4 m/s，8 段轨迹总时长 1.79 s，
最终速度为 0，终点误差约 0.28 m。RViz2 在 X11 下显示正常，Global Status 为 OK。
这次验收使用 Gazebo 后台物理仿真与 RViz2；未验收 Gazebo GUI 和真实底盘。
当前无显示器 XWayland 环境中，整组 launch 用 Ctrl+C 退出时观察到 RViz2
清理阶段报 `exit code -11`；单独 RViz 的退出诊断未复现。运行期间显示正常，
此退出问题尚未解决。launch 会停止其余节点。

![当前环境 RViz2 的模型与规划路径](docs/rviz2-x11.png)
