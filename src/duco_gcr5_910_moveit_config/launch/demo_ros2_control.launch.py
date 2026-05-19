from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_demo_launch(moveit_config, launch_package_path=None, declared_arguments=None):
    """
    Launches a self contained demo with ros2_control

    launch_package_path is optional to use different launch and config packages
    declared_arguments is optional list of declared launch arguments

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    
    # Add declared arguments if provided
    if declared_arguments:
        for arg in declared_arguments:
            ld.add_action(arg)
    
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Official ROS2 Control Node with DUCO hardware interface
    # Generate robot_description with network parameters
    robot_description_content = Command([
        "xacro ", str(moveit_config.package_path / "config/gcr5_910.urdf.xacro"),
        " robot_ip:=", LaunchConfiguration("robot_ip"),
        " robot_port:=", LaunchConfiguration("robot_port"),
        " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware")
    ])
    
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher with custom robot_description
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            robot_description_param,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            'arm_num', 
            default_value='1',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'arm_dof', 
            default_value='6',
        )
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            str(moveit_config.package_path / "config/ros2_controllers_hardware.yaml"),
        ],
        output="both",
    )
    ld.add_action(ros2_control_node)

    # Joint state broadcaster - wait for controller_manager
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessStart
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Delay joint_state_broadcaster spawner until ros2_control_node starts
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    ld.add_action(delayed_joint_state_broadcaster)
    
    # Joint trajectory controller - wait for joint_state_broadcaster
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_1_controller", "--controller-manager", "/controller_manager"],
    )
    
    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )
    ld.add_action(delayed_arm_controller)

    # FZI-style direct position controller - loaded inactive so it shows up
    # in `ros2 control list_controllers` but does NOT claim the position
    # command interface (which is owned by arm_1_controller at startup).
    # Users activate it manually via /controller_manager/switch_controller
    # and then stream Float64MultiArray to
    # /forward_position_controller/commands.
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
    )

    delayed_forward_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[forward_position_controller_spawner],
        )
    )
    ld.add_action(delayed_forward_position_controller)

    return ld


def generate_launch_description():
    # Declare launch arguments for network configuration
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="Robot IP address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port", 
            default_value="7003",
            description="Robot port number",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock hardware instead of real hardware interface",
        )
    )
    
    # Build robot description with network parameters  
    moveit_config = MoveItConfigsBuilder("gcr5_910", package_name="duco_gcr5_910_moveit_config").to_moveit_configs()
    launch_description = generate_demo_launch(moveit_config, declared_arguments=declared_arguments)
    
    return launch_description
