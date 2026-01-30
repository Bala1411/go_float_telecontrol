from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # -----------------------------
    # Launch argument
    # -----------------------------
    use_teleop = LaunchConfiguration('use_teleop')

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Enable task-space teleoperation'
    )

    # -----------------------------
    # Robot description
    # -----------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('dsr_a0509_description'),
                'urdf',
                'a0509.urdf.xacro'
            ])
        ]),
        value_type=str
    )

    # -----------------------------
    # robot_state_publisher (ALWAYS ON)
    # -----------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # -----------------------------
    # joint_state_publisher (ONLY when teleop = false)
    # -----------------------------
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_teleop),
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # -----------------------------
    # Teleop node (ONLY when teleop = true)
    # -----------------------------
    teleop_node = Node(
        package='cartesian_teleop_controller',
        executable='task_space_teleop',
        condition=IfCondition(use_teleop),
        output='screen'
    )

    # -----------------------------
    # RViz
    # -----------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('dsr_a0509_description'),
            'rviz',
            'display.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        declare_use_teleop,
        rsp_node,
        jsp_node,
        teleop_node,
        rviz_node
    ])
