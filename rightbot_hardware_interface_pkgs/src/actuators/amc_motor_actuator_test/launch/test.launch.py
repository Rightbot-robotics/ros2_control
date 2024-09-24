import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    
    urdf_path = get_package_share_directory('amc_motor_actuator_test') + '/urdf/test.urdf'
    urdf = open(urdf_path).read()
    
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': urdf}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("amc_motor_actuator_test"),
        "config",
        "controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': urdf}, ros2_controllers_path],
        output="both",
        arguments=['--ros-args', '--log-level', "info", '--log-level', 'rcl:=INFO']
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout=360"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager", "--controller-manager-timeout=360"],
    )
    
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager", "--controller-manager-timeout=360"],
    )

    conveyor_belt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["conveyor_belt_controller", "-c", "/controller_manager", "--controller-manager-timeout=360"],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            velocity_controller_spawner,
            # position_controller_spawner,
            # conveyor_belt_controller_spawner,
        ]
    )
