# DUCO 机器人 ROS2 驱动

[简体中文](https://github.com/Rsm080/-GCR30-1100/blob/main/README.md) | English

这是一个完整的 DUCO 协作机器人（GCR5-910 型号）ROS2 驱动包，提供与 MoveIt2 集成的硬件接口，用于运动规划与控制。

## 目录

## 概述

本仓库为 DUCO 机械臂提供一个简洁、独立的 ROS2 驱动。它与 ROS2 control 框架和 MoveIt2 集成，实现以下功能：

- 通过硬件接口进行实时机器人控制
- 使用 MoveIt2 进行运动规划
- 轨迹执行
- 关节状态监控
- RViz 可视化

该驱动使用 DUCO 官方机器人通信协议，通过 TCP/IP 连接机器人控制器。

## 特性

- **ROS2 Control 集成**：完整的 `ros2_control` 硬件接口实现
- **MoveIt2 支持**：完整的 MoveIt2 运动规划配置
- **关节轨迹控制器**：执行复杂轨迹，位置控制（`arm_1_controller`，默认激活）
- **直接位置流控制器**：加载一个 `forward_command_controller` 实例（`forward_position_controller`），与 JTC 并存，默认不激活。用户可按需激活，并在每个控制器周期直接将 `std_msgs/Float64MultiArray` 关节目标流式写入位置命令接口——写入模式与 FZI 的 `cartesian_*_controller` 系列相同，只是在关节空间中暴露。
- **实时状态监控**：监控关节位置、速度和机器人状态
- **网络通信**：通过 TCP/IP 连接机器人控制器
- **RViz 可视化**：交互式可视化与运动规划界面
- **自定义消息**：用于机器人状态和控制的专用消息类型

## 系统要求

### 软件要求

- 操作系统：Ubuntu 22.04（Jammy）
- ROS2 发行版：Humble Hawksbill 或更高版本
- 构建系统：`colcon`
- 依赖项：`moveit2`、`ros2_control`、`ros2_controllers`、`controller_manager`、`xacro`、`joint_state_broadcaster`、`joint_trajectory_controller`

### 硬件要求

- DUCO GCR5-910 机械臂
- 固件版本：V3.8.4 或更高（`ros2_control` 支持所必需）
- 请参阅机械臂手册获取系统更新说明
- 与机器人控制器的网络连接（推荐以太网）
- 具备足够资源进行实时控制的计算机

## 安装

### 1. 克隆仓库

```bash
cd ~/Documents
git clone https://github.com/yizhongzhang1989/duco_ros2_driver.git
cd duco_ros2_driver
```

### 2. 安装依赖

安装 ROS2 Humble（如果尚未安装）：

```bash
# 添加 ROS2 apt 仓库
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

安装 MoveIt2 和所需软件包：

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-xacro \
  ros-humble-warehouse-ros-mongo
```

### 3. 安装额外依赖

```bash
# 安装 DUCO SDK 库（如果你的机器人需要）
# 请遵循 DUCO 官方的 SDK 安装说明
```

### 4. 构建工作空间

```bash
cd ~/Documents/duco_ros2_driver
colcon build
```

### 5. Source 工作空间

```bash
source install/setup.bash
```

为方便起见，可添加到 `~/.bashrc`：

```bash
echo "source ~/Documents/duco_ros2_driver/install/setup.bash" >> ~/.bashrc
```

## 配置

### Launch 参数

演示 launch 文件接受以下参数：

- `server_host_1`：机器人控制器 IP 地址（默认：`127.0.0.1`）
- `arm_num`：机械臂数量（默认：`1`）
- `arm_dof`：每条臂的自由度（默认：`6`）
- `use_rviz`：是否启动 RViz 可视化（默认：`true`）
- `db`：是否启动仓库数据库（默认：`false`）
- `debug`：是否启用调试模式（默认：`false`）

### 修改 Launch 参数

在启动时传入参数：

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py server_host_1:=<IP地址>
```

### 控制器配置

控制器参数定义在：

```text
src/duco_gcr5_910_moveit_config/config/ros2_controllers_hardware.yaml
```

该子模块在启动时启动两个臂级控制器：

| 控制器 | 类型 | 默认状态 |
|---|---|---|
| `arm_1_controller` | `joint_trajectory_controller/JointTrajectoryController` | 激活 |
| `forward_position_controller` | `forward_command_controller/ForwardCommandController` | 未激活 |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 激活 |

两个臂控制器占用同一个位置命令接口，因此同一时间只能有一个处于激活状态。默认情况下轨迹控制器优先；若要切换到 forward 控制器，请使用“直接位置流”一节中给出的服务调用。

关键参数：

- `update_rate`：控制循环频率（默认：250 Hz）
- `joints`：受控关节列表（`arm_1_joint_1` .. `arm_1_joint_6`）
- `command_interfaces`：`position`
- `state_interfaces`：`position`、`velocity`

### URDF 配置

机器人 URDF 的硬件接口参数位于：

```text
src/duco_gcr5_910_moveit_config/config/gcr5_910.ros2_control.xacro
```

硬件接口使用 `mock_components` 进行仿真：

```text
mock_components/GenericSystem
```

## 用法

该驱动提供两种 launch 模式：

- **自定义驱动模式**（`demo.launch.py`）：使用自定义 DUCO 驱动节点
- **ros2_control 模式**（`demo_ros2_control.launch.py`）：使用标准 `ros2_control` 框架与 DUCO 硬件接口

### 基本用法 — 自定义驱动模式

#### 1. 启动机器人系统（自定义驱动）

使用自定义 DUCO 驱动节点启动完整系统：

```bash
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py
```

这将启动：

- 机器人状态发布器
- DUCO 驱动节点（`DucoDriver`、`DucoRobotStatus`、`DucoTrajectoryAction`、`DucoRobotControl`）
- MoveIt `move_group` 节点
- 带 MoveIt 插件的 RViz

参数：

- `server_host_1`：机器人控制器 IP 地址（默认：`127.0.0.1`）
- `arm_num`：机械臂数量（默认：`1`）
- `arm_dof`：自由度（默认：`6`）

### 基本用法 — ros2_control 模式

#### 1. 启动机器人系统（ros2_control）

使用标准 `ros2_control` 框架启动：

```bash
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py robot_ip:=<IP地址>
```

无硬件测试/仿真：

```bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py use_fake_hardware:=true
```

这将启动：

- 机器人状态发布器
- `ros2_control` 硬件接口（`DucoHardwareInterface` 或 `mock_components`）
- 关节状态广播器
- 关节轨迹控制器（`arm_1_controller`）
- MoveIt `move_group` 节点
- 带 MoveIt 插件的 RViz（可选）

参数：

- `robot_ip`：机器人控制器 IP 地址（默认：`192.168.1.10`）
- `robot_port`：机器人控制器端口（默认：`7003`）
- `use_fake_hardware`：使用模拟硬件代替真实硬件（默认：`false`）
- `use_rviz`：是否启动 RViz（默认：`true`）
- `db`：是否启动数据库（默认：`false`）

检查控制器：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

启动后 `list_controllers` 的预期输出：

```text
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster    active
arm_1_controller           joint_trajectory_controller/JointTrajectoryController    active
forward_position_controller forward_command_controller/ForwardCommandController    inactive
```

#### 直接位置流（forward_position_controller）

驱动附带一个 `forward_command_controller` 实例，它绕过 MoveIt 的轨迹管线，在每个控制器周期（250 Hz）将用户提供的关节目标直接写入位置命令接口。它是 FZI `cartesian_motion_controller` 的关节空间对应版本，适用于遥操作、学习策略，或任何已经自行计算关节设定值的应用。它默认加载为未激活状态，以便轨迹控制器默认占用位置接口。

**步骤 1 — 激活它（并在同一个原子调用中停用轨迹控制器）：**

```bash
ros2 control switch_controllers \
  --deactivate arm_1_controller \
  --activate forward_position_controller \
  --strict --activate-asap
```

这是 `ros2 service call /controller_manager/switch_controller ...` 的简短 CLI 形式；两者底层调用的是同一个 `SwitchController` 服务。只有在需要脚本化完整请求负载（例如自定义超时值）时才使用详细服务形式。成功响应为 `ok: True`。用 `ros2 control list_controllers` 确认：

```text
arm_1_controller ... inactive
forward_position_controller ... active
```

**步骤 2 — 流式发送命令。** 向 `/forward_position_controller/commands` 发布一个包含 6 个浮点数（弧度，按 `arm_1_joint_1` .. `arm_1_joint_6` 顺序）的 `std_msgs/Float64MultiArray`。

保持当前位置快速测试：

```bash
# 读取当前关节位置并以 100 Hz 原样回显。
# 将数组替换为你自己的设定值。
ros2 topic pub -r 100 /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [-2.907, -0.040, -1.502, 1.605, 1.563, -1.641]}"
```

对于真正的流式应用，使用一个在每个周期更新数组的节点——控制器会在每次控制器更新时将最后发布的值写入硬件。建议速率：等于控制器的 `update_rate`（250 Hz）。每条消息中必须包含全部 6 个关节值；控制器不做内部插值。

**步骤 3 — 想恢复 MoveIt 时切换回轨迹控制器：**

```bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate arm_1_controller \
  --strict --activate-asap
```

最小 Python 发布者（完整斜坡 + 正弦示例）：

```python
#!/usr/bin/env python3
"""在关节 2 上流式发送 0.1 Hz、0.1 rad 的正弦信号，从捕获的初始位置平滑缓入。
在 forward_position_controller 激活时运行。"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['arm_1_joint_1', 'arm_1_joint_2', 'arm_1_joint_3',
          'arm_1_joint_4', 'arm_1_joint_5', 'arm_1_joint_6']
TARGET_JOINT = 1  # arm_1_joint_2
AMPLITUDE = 0.1   # rad
FREQUENCY = 0.1   # Hz
EASE_IN_SEC = 2.0 # 前 2 秒平滑增大振幅


class Streamer(Node):
    def __init__(self):
        super().__init__('forward_position_streamer')
        self.pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.home = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / 250.0, self.tick)  # 匹配控制器 update_rate

    def on_state(self, msg):
        if self.home is None:
            self.home = [msg.position[msg.name.index(j)] for j in JOINTS]
            self.get_logger().info(f'Captured home: {self.home}')

    def tick(self):
        if self.home is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        ease = min(1.0, 0.5 * (1.0 - math.cos(math.pi * min(t, EASE_IN_SEC) / EASE_IN_SEC)))
        delta = AMPLITUDE * ease * math.sin(2 * math.pi * FREQUENCY * t)
        target = list(self.home)
        target[TARGET_JOINT] += delta
        self.pub.publish(Float64MultiArray(data=target))


def main():
    rclpy.init()
    rclpy.spin(Streamer())


if __name__ == '__main__':
    main()
```

#### 2. 使用 RViz 进行运动规划

在 RViz 窗口中：

- 使用交互标记将末端执行器拖到目标位置
- 在 MotionPlanning 面板中点击“Plan”计算轨迹
- 点击“Execute”将轨迹发送给机器人
- 机器人将移动到目标位姿

### 高级用法

#### 自定义驱动模式 + 自定义 IP

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py \
  server_host_1:=192.168.0.100
```

#### ros2_control 模式 + 自定义 IP

```bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py \
  robot_ip:=192.168.1.100 \
  robot_port:=7003
```

#### 不启动 RViz

```bash
# 自定义驱动模式
ros2 launch duco_gcr5_910_moveit_config demo.launch.py use_rviz:=false

# ros2_control 模式
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py use_rviz:=false
```

#### 使用 MoveIt Python 接口

创建 Python 脚本进行程序化控制：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy


def main():
    rclpy.init()

    # 初始化 MoveIt
    moveit = MoveItPy(node_name="moveit_py_demo")
    arm = moveit.get_planning_component("arm_1")

    # 设置目标位姿
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.w = 1.0

    # 规划并执行
    arm.set_pose_goal(target_pose)
    plan_result = arm.plan()

    if plan_result:
        arm.execute(plan_result.trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 使用 Action Client 进行轨迹控制

直接发送轨迹命令：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_1_controller/follow_joint_trajectory')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # 定义关节名称
        goal_msg.trajectory.joint_names = [
            'arm_1_joint_1', 'arm_1_joint_2', 'arm_1_joint_3',
            'arm_1_joint_4', 'arm_1_joint_5', 'arm_1_joint_6'
        ]

        # 定义路点
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
        point.time_from_start.sec = 3
        goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    client = TrajectoryActionClient()
    future = client.send_goal()
    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 监控机器人状态

查看关节状态：

```bash
ros2 topic echo /joint_states
```

查看控制器状态：

```bash
ros2 control list_controllers
```

检查硬件接口状态：

```bash
ros2 control list_hardware_interfaces
```

### 常用 ROS2 命令

```bash
# 列出所有活动节点
ros2 node list

# 列出所有活动话题
ros2 topic list

# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查控制器管理器
ros2 control list_controllers

# 加载控制器
ros2 control load_controller <控制器名称>

# 启动控制器
ros2 control switch_controllers --activate <控制器名称>
```

## 架构

### 系统架构

```text
┌─────────────────────────────────────────────────────────────┐
│                        MoveIt2                              │
│  ┌─────────────────┐         ┌──────────────────┐          │
│  │  运动规划        │────────▶│    轨迹执行       │          │
│  │  管线            │         │                  │          │
│  └─────────────────┘         └──────────────────┘          │
└───────────────────────────────────┬─────────────────────────┘
                                    │ action goal
                                    ▼
┌─────────────────────────────────────────────────────────────┐
│                     Controller Manager                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            Joint Trajectory Controller                │  │
│  │  ┌────────────────┐    ┌────────────────┐            │  │
│  │  │  轨迹插值器     │───▶│  PID 控制器     │            │  │
│  │  └────────────────┘    └────────────────┘            │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Joint State Broadcaster                  │  │
│  └──────────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────┬───────────────────┘
                    │ cmd                 │ state
                    ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                      ros2_control                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            DUCO 硬件接口                              │  │
│  │  ┌──────────────┐         ┌──────────────┐          │  │
│  │  │  write()     │         │   read()      │          │  │
│  │  │  命令        │         │   状态        │          │  │
│  │  └──────────────┘         └──────────────┘          │  │
│  └──────────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────┬───────────────────┘
                    │ TCP/IP              │
                    ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    DUCO 机器人控制器                         │
└─────────────────────────────────────────────────────────────┘
```

### 数据流

1. **MoveIt2**：计算无碰撞轨迹并发送 action goal
2. **控制器**：通过 action 接口接收轨迹目标
3. **控制器**：对轨迹点进行插值并执行
4. **硬件接口**：通过 TCP/IP 向机器人发送位置命令
5. **机器人**：执行运动并发送状态反馈
6. **状态广播**：关节状态发布到 ROS2 话题
7. **可视化**：RViz 显示当前机器人状态

### ROS2 话题

| 话题 | 类型 | 描述 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 当前关节位置和速度 |
| `/robot_description` | `std_msgs/String` | 机器人 URDF 模型 |
| `/arm_1_controller/follow_joint_trajectory` | Action | 轨迹执行 action 服务器（当 `arm_1_controller` 激活时可用） |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 直接关节位置设定值（当 `forward_position_controller` 激活时可用）。6 个浮点数，按 `arm_1_joint_1` .. `arm_1_joint_6` 顺序，单位弧度 |
| `/controller_manager/list_controllers` | Service | 列出可用控制器 |
| `/controller_manager/switch_controller` | Service | 原子地激活/停用控制器 |

### ROS2 参数

机器人系统的关键参数：

- `robot_ip`：机器人控制器 IP 地址（默认：`"192.168.1.10"`）
- `robot_port`：端口号（默认：`"7003"`）
- `arm_num`：机械臂数量（默认：`1`）
- `arm_dof`：每条臂的自由度（默认：`6`）

## 故障排除

### 常见问题

#### 1. 无法连接机器人

**症状**：启动因连接超时而失败

**解决方案**：

- 确认机器人已上电且控制器正在运行
- 检查网络连通性：`ping <机器人IP>`
- 验证 launch 参数中的 IP 地址和端口是否正确
- 检查防火墙设置
- 确保机器人控制器软件正在运行

```bash
# 测试连接
ping 192.168.1.10

# 使用正确 IP 启动
ros2 launch duco_gcr5_910_moveit_config demo.launch.py robot_ip:=<IP地址>
```

#### 2. 控制器未加载

**症状**：控制器 spawner 失败

**解决方案**：

- 检查 `ros2_controllers.yaml` 或 `ros2_controllers_hardware.yaml` 中的控制器配置
- 验证硬件接口已加载：`ros2 control list_hardware_interfaces`
- 检查控制器管理器状态

```bash
# 列出硬件接口
ros2 control list_hardware_interfaces

# 列出控制器
ros2 control list_controllers

# 手动 spawn 控制器
ros2 control load_controller arm_1_controller
ros2 control switch_controllers --activate arm_1_controller
```

#### 2a. 控制器“已加载”或“无法从激活状态配置”

**症状**：启动失败，出现类似错误：

- `Controller already loaded, skipping load_controller`
- `Controller 'arm_1_controller' can not be configured from 'active' state`
- `Failed to configure controller`

**原因**：上一次启动的控制器仍在运行。

**解决方案**：

```bash
# 选项 1：终止特定的 ros2_control 进程
pkill -f "ros2_control_node|controller_manager|spawner"

# 选项 2：终止所有 ROS2 进程（如果安全）
pkill -f ros2

# 然后以干净状态重新启动
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py
```

**预防**：始终使用 `Ctrl+C` 干净地终止 launch，并等待所有进程退出后再重新启动。

#### 2b. 硬件连接验证

启动成功后，验证系统是否正常运行：

```bash
# 检查控制器是否激活
ros2 control list_controllers
# 预期输出：
# arm_1_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# 检查关节状态是否发布
ros2 topic echo /joint_states --once

# 检查硬件接口状态
ros2 control list_hardware_interfaces
```

#### 3. MoveIt 规划失败

**症状**：未找到有效规划

**解决方案**：

- 检查 `joint_limits.yaml` 中的关节限位
- 验证运动学求解器配置
- 调整规划时间：在 MoveIt 设置中增加超时
- 检查起始或目标状态是否存在碰撞

#### 4. 构建错误

**症状**：编译失败

**解决方案**：

```bash
# 清理构建
rm -rf build/ install/ log/

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 重新构建
colcon build
```

#### 5. 变换错误（TF）

**症状**：TF 查找失败

**解决方案**：

- 确保 `robot_state_publisher` 正在运行
- 检查 URDF 是否正确加载
- 验证所有坐标系都已发布

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 回显特定变换
ros2 run tf2_ros tf2_echo base_link arm_1_link_6
```

### 调试模式

启用调试输出以获取更多信息：

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py debug:=true
```

### 日志

查看日志以获取详细错误消息：

```bash
# 查看最新构建日志
cat log/latest_build/duco_ros_driver/stdout_stderr.log

# 运行时日志存储在：
~/.ros/log/
```

## 贡献

欢迎贡献！请：

- Fork 仓库
- 创建功能分支
- 提交更改
- 推送到分支
- 创建 Pull Request

## 支持

如有问题和疑问：

- 在 GitHub 上提交 issue：https://github.com/yizhongzhang1989/duco_ros2_driver/issues
- 查阅 DUCO 机器人文档
- 查看 ROS2 和 MoveIt2 官方文档

## 参考

## 许可证

本项目采用 MIT 许可证 - 详情请参阅 [LICENSE](https://github.com/Rsm080/-GCR30-1100/blob/main/LICENSE) 文件。

## 致谢

- DUCO 机器人团队提供机械臂和 SDK
- MoveIt2 社区提供运动规划框架
- ROS2 社区提供机器人中间件
- 本项目的贡献者

版本：1.1.0
最后更新：2026 年 5 月
维护者：yizhongzhang1989