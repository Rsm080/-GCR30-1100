# DUCO Robot ROS2 Driver

A complete ROS2 driver package for DUCO collaborative robot arms (GCR5-910 model), providing hardware interface integration with MoveIt2 for motion planning and control.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

This repository provides a clean, standalone ROS2 driver for DUCO robotic arms. It integrates with the ROS2 control framework and MoveIt2 to enable:

- Real-time robot control via hardware interface
- Motion planning with MoveIt2
- Trajectory execution
- Joint state monitoring
- RViz visualization

The driver uses the official DUCO robot communication protocol to connect to the robot controller over TCP/IP.

## Features

- **ROS2 Control Integration**: Full ros2_control hardware interface implementation
- **MoveIt2 Support**: Complete MoveIt2 configuration for motion planning
- **Joint Trajectory Controller**: Execute complex trajectories with position control (`arm_1_controller`, active by default)
- **Direct Position Streaming Controller**: A `forward_command_controller` instance (`forward_position_controller`) is loaded alongside the JTC and stays **inactive by default**. Users activate it on demand and stream `std_msgs/Float64MultiArray` joint targets straight to the position command interface every controller tick — same write pattern as the FZI `cartesian_*_controller` family, just exposed in joint space. See [Direct Position Streaming](#direct-position-streaming-forward_position_controller) below.
- **Real-time State Monitoring**: Monitor joint positions, velocities, and robot status
- **Network Communication**: TCP/IP connection to robot controller
- **RViz Visualization**: Interactive visualization and motion planning interface
- **Custom Messages**: Specialized message types for robot state and control

## System Requirements

### Software Requirements

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS2 Distribution**: Humble Hawksbill or later
- **Build System**: colcon
- **Dependencies**:
  - moveit2
  - ros2_control
  - ros2_controllers
  - controller_manager
  - xacro
  - joint_state_broadcaster
  - joint_trajectory_controller

### Hardware Requirements

- DUCO GCR5-910 Robot Arm
  - **Firmware Version**: V3.8.4 or later (required for ros2_control support)
  - Refer to robot arm manual for system update instructions
- Network connection to robot controller (Ethernet recommended)
- Computer with sufficient resources for real-time control

## Installation
### 1. Clone the Repository

```bash
cd ~/Documents
git clone https://github.com/yizhongzhang1989/duco_ros2_driver.git
cd duco_ros2_driver
```

### 2. Install Dependencies

Install ROS2 Humble (if not already installed):

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

Install MoveIt2 and required packages:

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

### 3. Install Additional Dependencies

```bash
# Install DUCO SDK libraries (if required by your robot)
# Follow DUCO's official installation instructions for their SDK
```

### 4. Build the Workspace

```bash
cd ~/Documents/duco_ros2_driver
colcon build
```

### 5. Source the Workspace

```bash
source install/setup.bash
```

Add to your `~/.bashrc` for convenience:

```bash
echo "source ~/Documents/duco_ros2_driver/install/setup.bash" >> ~/.bashrc
```

## Configuration

### Launch Arguments

The demo launch file accepts the following arguments:

- **server_host_1**: Robot controller IP address (default: `127.0.0.1`)
- **arm_num**: Number of robot arms (default: `1`)
- **arm_dof**: Degrees of freedom per arm (default: `6`)
- **use_rviz**: Launch RViz visualization (default: `true`)
- **db**: Start warehouse database (default: `false`)
- **debug**: Enable debug mode (default: `false`)

### Modifying Launch Arguments

Pass arguments during launch:

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py server_host_1:=<YOUR_ROBOT_IP>
```

### Controller Configuration

Controller parameters are defined in:

```
src/duco_gcr5_910_moveit_config/config/ros2_controllers_hardware.yaml
```

The submodule launches two arm-level controllers at startup:

| Controller | Type | Default state |
|---|---|---|
| `arm_1_controller` | `joint_trajectory_controller/JointTrajectoryController` | **active** |
| `forward_position_controller` | `forward_command_controller/ForwardCommandController` | **inactive** |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |

Both arm controllers claim the **same** position command interface, so only one can be active at a time. The trajectory controller wins by default; switch to the forward controller with the service call shown in [Direct Position Streaming](#direct-position-streaming-forward_position_controller).

Key parameters:

- **update_rate**: Control loop frequency (default: 250 Hz)
- **joints**: List of controlled joints (`arm_1_joint_1` .. `arm_1_joint_6`)
- **command_interfaces**: `position`
- **state_interfaces**: `position`, `velocity`

### URDF Configuration

The robot URDF includes hardware interface parameters in:

```
src/duco_gcr5_910_moveit_config/config/gcr5_910.ros2_control.xacro
```

The hardware interface uses mock_components for simulation:

```xml
<plugin>mock_components/GenericSystem</plugin>
```

## Usage

The driver provides two launch modes:

1. **Custom Driver Mode** (`demo.launch.py`): Uses custom DUCO driver nodes
2. **ros2_control Mode** (`demo_ros2_control.launch.py`): Uses standard ros2_control framework with DUCO hardware interface

### Basic Usage - Custom Driver Mode

#### 1. Start the Robot System (Custom Driver)

Launch the complete system with custom DUCO driver nodes:

```bash
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py
```

This will start:

- Robot state publisher
- DUCO driver nodes (DucoDriver, DucoRobotStatus, DucoTrajectoryAction, DucoRobotControl)
- MoveIt move_group node
- RViz with MoveIt plugin

**Arguments:**
- `server_host_1`: Robot controller IP address (default: `127.0.0.1`)
- `arm_num`: Number of arms (default: `1`)
- `arm_dof`: Degrees of freedom (default: `6`)

### Basic Usage - ros2_control Mode

#### 1. Start the Robot System (ros2_control)

**Hardware Requirements:** Ensure the DUCO robot controller is powered on and accessible at the configured IP address.

Launch with standard ros2_control framework:

```bash
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

For testing/simulation without hardware:

```bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py use_fake_hardware:=true
```

This will start:

- Robot state publisher
- ros2_control hardware interface (DucoHardwareInterface or mock_components)
- Joint state broadcaster
- Joint trajectory controller (arm_1_controller)
- MoveIt move_group node
- RViz with MoveIt plugin (optional)

**Arguments:**
- `robot_ip`: Robot controller IP address (default: `192.168.1.10`)
- `robot_port`: Robot controller port (default: `7003`)
- `use_fake_hardware`: Use mock hardware instead of real hardware (default: `false`)
- `use_rviz`: Launch RViz (default: `true`)
- `db`: Start database (default: `false`)

**Check controllers:**
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Expected output of `list_controllers` right after launch:

```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster        active
arm_1_controller             joint_trajectory_controller/JointTrajectoryController  active
forward_position_controller  forward_command_controller/ForwardCommandController  inactive
```

#### Direct Position Streaming (`forward_position_controller`)

The driver ships a `forward_command_controller` instance that bypasses MoveIt's trajectory pipeline and writes user-supplied joint targets directly to the position command interface every controller tick (250 Hz). It is the joint-space analogue of FZI's `cartesian_motion_controller` and is intended for teleoperation, learning policies, or any application that already computes its own joint setpoints. It is loaded **inactive** so the trajectory controller can own the position interface by default.

**Step 1 — Activate it (and deactivate the trajectory controller in the same atomic call):**

```bash
ros2 control switch_controllers \
    --deactivate arm_1_controller \
    --activate   forward_position_controller \
    --strict --activate-asap
```

> This is the short CLI form of `ros2 service call /controller_manager/switch_controller ...`; both call the same `SwitchController` service under the hood. Use the verbose service form only when you need to script the full request payload (e.g. for custom `timeout` values).

A successful response is `ok: True`. Confirm with `ros2 control list_controllers`:

```
arm_1_controller             ... inactive
forward_position_controller  ... active
```

**Step 2 — Stream commands.** Publish a `std_msgs/Float64MultiArray` of six floats (radians, ordered `arm_1_joint_1` .. `arm_1_joint_6`) to `/forward_position_controller/commands`. Hold-current-position quick test:

```bash
# Read the current joint positions and echo them right back at 100 Hz.
# Replace the array with your own setpoints.
ros2 topic pub -r 100 /forward_position_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "{data: [-2.907, -0.040, -1.502, 1.605, 1.563, -1.641]}"
```

For a real streaming application use a node that updates the array each tick — the controller writes whatever was last published to the hardware on every controller update. Suggested rate: equal to the controller's `update_rate` (250 Hz). All six joint values **must** be present in every message; the controller does no internal interpolation.

⚠️ **Safety**: There is no trajectory smoothing on this path. A step from the current position to a far target will be executed at maximum joint velocity. Always start a session by publishing the **current** joint positions for the first ~2 seconds and then slowly ramp toward the target. Stop publishing → the controller holds the last received value indefinitely.

**Step 3 — Switch back to the trajectory controller** when you want MoveIt back:

```bash
ros2 control switch_controllers \
    --deactivate forward_position_controller \
    --activate   arm_1_controller \
    --strict --activate-asap
```

**Minimal Python publisher (full ramp + sine example):**

```python
#!/usr/bin/env python3
"""Stream a 0.1 Hz, 0.1 rad sine on joint 2, smoothly eased in from the
captured home position. Run with the forward_position_controller active."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['arm_1_joint_1', 'arm_1_joint_2', 'arm_1_joint_3',
          'arm_1_joint_4', 'arm_1_joint_5', 'arm_1_joint_6']
TARGET_JOINT = 1     # arm_1_joint_2
AMPLITUDE   = 0.1    # rad
FREQUENCY   = 0.1    # Hz
EASE_IN_SEC = 2.0    # smoothly grow amplitude over the first 2 s

class Streamer(Node):
    def __init__(self):
        super().__init__('forward_position_streamer')
        self.pub  = self.create_publisher(Float64MultiArray,
                                          '/forward_position_controller/commands', 10)
        self.home = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / 250.0, self.tick)  # match controller update_rate

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

#### 2. Using RViz for Motion Planning

In the RViz window:

1. Use the interactive markers to drag the end-effector to desired position
2. Click "Plan" in the MotionPlanning panel to compute a trajectory
3. Click "Execute" to send the trajectory to the robot
4. The robot will move to the target pose

### Advanced Usage

#### Custom Driver Mode with Custom IP

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py \
    server_host_1:=192.168.0.100
```

#### ros2_control Mode with Custom IP

```bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py \
    robot_ip:=192.168.1.100 \
    robot_port:=7003
```

#### Launch Without RViz

```bash
# Custom driver mode
ros2 launch duco_gcr5_910_moveit_config demo.launch.py use_rviz:=false

# ros2_control mode
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py use_rviz:=false
```

#### Using MoveIt Python Interface

Create a Python script for programmatic control:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy

def main():
    rclpy.init()
    
    # Initialize MoveIt
    moveit = MoveItPy(node_name="moveit_py_demo")
    arm = moveit.get_planning_component("arm_1")
    
    # Set target pose
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.w = 1.0
    
    # Plan and execute
    arm.set_pose_goal(target_pose)
    plan_result = arm.plan()
    
    if plan_result:
        arm.execute(plan_result.trajectory)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Using Action Client for Trajectory Control

Send trajectory commands directly:

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
            self,
            FollowJointTrajectory,
            '/arm_1_controller/follow_joint_trajectory'
        )
    
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        # Define joint names
        goal_msg.trajectory.joint_names = [
            'arm_1_joint_1', 'arm_1_joint_2', 'arm_1_joint_3',
            'arm_1_joint_4', 'arm_1_joint_5', 'arm_1_joint_6'
        ]
        
        # Define waypoint
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

### Monitoring Robot State

#### View Joint States

```bash
ros2 topic echo /joint_states
```

#### View Controller Status

```bash
ros2 control list_controllers
```

#### Check Hardware Interface Status

```bash
ros2 control list_hardware_interfaces
```

### Useful ROS2 Commands

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# View TF tree
ros2 run tf2_tools view_frames

# Check controller manager
ros2 control list_controllers

# Load a controller
ros2 control load_controller <controller_name>

# Start a controller
ros2 control switch_controllers --activate <controller_name>
```

## Architecture

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        MoveIt2                              │
│  ┌─────────────────┐        ┌──────────────────┐          │
│  │  Motion Planning│───────▶│  Trajectory      │          │
│  │  Pipeline       │        │  Execution       │          │
│  └─────────────────┘        └──────────────────┘          │
└───────────────────────────────────┬─────────────────────────┘
                                    │ action goal
                                    ▼
┌─────────────────────────────────────────────────────────────┐
│               Controller Manager                            │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     Joint Trajectory Controller                      │  │
│  │  ┌────────────────┐      ┌────────────────┐         │  │
│  │  │ Trajectory     │─────▶│ PID Controller │         │  │
│  │  │ Interpolator   │      │                │         │  │
│  │  └────────────────┘      └────────────────┘         │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     Joint State Broadcaster                          │  │
│  └──────────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────┬───────────────────┘
                    │ cmd                 │ state
                    ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                  ros2_control                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     DUCO Hardware Interface                          │  │
│  │  ┌──────────────┐          ┌──────────────┐         │  │
│  │  │ write()      │          │ read()       │         │  │
│  │  │ commands     │          │ states       │         │  │
│  │  └──────────────┘          └──────────────┘         │  │
│  └──────────────────────────────────────────────────────┘  │
└───────────────────┬─────────────────────┬───────────────────┘
                    │ TCP/IP              │
                    ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│              DUCO Robot Controller                          │
│                 (192.168.1.10:7003)                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              DUCO GCR5-910 Robot Arm                        │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Planning**: User interacts with RViz or Python API to request motion
2. **MoveIt2**: Computes collision-free trajectory
3. **Action Goal**: Trajectory sent to controller via action interface
4. **Controller**: Interpolates and executes trajectory points
5. **Hardware Interface**: Sends position commands via TCP/IP to robot
6. **Robot**: Executes motion and sends state feedback
7. **State Broadcasting**: Joint states published to ROS2 topics
8. **Visualization**: RViz displays current robot state

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions and velocities |
| `/robot_description` | `std_msgs/String` | Robot URDF model |
| `/arm_1_controller/follow_joint_trajectory` | Action | Trajectory execution action server (active when `arm_1_controller` is) |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Direct joint-position setpoints (active when `forward_position_controller` is). 6 floats in `arm_1_joint_1` .. `arm_1_joint_6` order, radians. |
| `/controller_manager/list_controllers` | Service | List available controllers |
| `/controller_manager/switch_controller` | Service | Activate / deactivate controllers atomically |

### ROS2 Parameters

Key parameters for the robot system:

- `robot_ip`: IP address of robot controller (default: "192.168.1.10")
- `robot_port`: Port number (default: "7003")
- `arm_num`: Number of robot arms (default: 1)
- `arm_dof`: Degrees of freedom per arm (default: 6)

## Troubleshooting

### Common Issues

#### 1. Cannot Connect to Robot

**Symptom**: Launch fails with connection timeout

**Solutions**:
- Verify robot is powered on and controller is running
- Check network connectivity: `ping <robot_ip>`
- Verify correct IP address and port in launch arguments
- Check firewall settings
- Ensure robot controller software is running

```bash
# Test connection
ping 192.168.1.10

# Launch with correct IP
ros2 launch duco_gcr5_910_moveit_config demo.launch.py robot_ip:=<CORRECT_IP>
```

#### 2. Controllers Not Loading

**Symptom**: Controller spawner fails

**Solutions**:
- Check controller configuration in `ros2_controllers.yaml` or `ros2_controllers_hardware.yaml`
- Verify hardware interface is loaded: `ros2 control list_hardware_interfaces`
- Check controller manager status

```bash
# List hardware interfaces
ros2 control list_hardware_interfaces

# List controllers
ros2 control list_controllers

# Manually spawn controller
ros2 control load_controller arm_1_controller
ros2 control switch_controllers --activate arm_1_controller
```

#### 2a. Controller "Already Loaded" or "Cannot Configure from Active State"

**Symptom**: Launch fails with errors like:
- `Controller already loaded, skipping load_controller`
- `Controller 'arm_1_controller' can not be configured from 'active' state`
- `Failed to configure controller`

**Cause**: Controllers from a previous launch are still running

**Solutions**:

```bash
# Option 1: Kill specific ros2_control processes
pkill -f "ros2_control_node|controller_manager|spawner"

# Option 2: Kill all ROS2 processes (if safe)
pkill -f ros2

# Then relaunch with a clean state
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo_ros2_control.launch.py
```

**Prevention**: Always cleanly terminate launches with `Ctrl+C` and wait for all processes to exit before relaunching.

#### 2b. Hardware Connection Verification

After successful launch, verify the system is operational:

```bash
# Check controllers are active
ros2 control list_controllers
# Expected output:
# arm_1_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# Check joint states are published
ros2 topic echo /joint_states --once

# Check hardware interface status
ros2 control list_hardware_interfaces
```

#### 3. MoveIt Planning Fails

**Symptom**: No valid plan found

**Solutions**:
- Check joint limits in `joint_limits.yaml`
- Verify kinematics solver configuration
- Adjust planning time: increase timeout in MoveIt settings
- Check for collisions in start or goal state

#### 4. Build Errors

**Symptom**: Compilation fails

**Solutions**:

```bash
# Clean build
rm -rf build/ install/ log/

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

#### 5. Transform Errors (TF)

**Symptom**: TF lookup failures

**Solutions**:
- Ensure robot_state_publisher is running
- Check URDF is correctly loaded
- Verify all frames are published

```bash
# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link arm_1_link_6
```

### Debug Mode

Enable debug output for more information:

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py debug:=true
```

### Logging

Check logs for detailed error messages:

```bash
# View latest build log
cat log/latest_build/duco_ros_driver/stdout_stderr.log

# Runtime logs stored in:
~/.ros/log/
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Support

For issues and questions:

- Open an issue on GitHub: https://github.com/yizhongzhang1989/duco_ros2_driver/issues
- Consult DUCO robot documentation
- Check ROS2 and MoveIt2 official documentation

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/)
- [ros2_control Documentation](https://control.ros.org/humble/)
- [DUCO Robot Official Website](https://www.siasun-duco.com/)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- DUCO robotics team for the robot arm and SDK
- MoveIt2 community for motion planning framework
- ROS2 community for the robotic middleware
- Contributors to this project

---

**Version**: 1.1.0  
**Last Updated**: May 2026  
**Maintainer**: yizhongzhang1989
