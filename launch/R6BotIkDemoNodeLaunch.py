from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("multi_end_effector_kinematics"),
                    "models",
                    "r6bot",
                    "r6bot.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("multi_end_effector_kinematics"),
            "models",
            "r6bot",
            "rviz",
            "view_robot.rviz",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ik_demo_node = Node(
        package="multi_end_effector_kinematics",
        executable="R6BotIkDemoNode",
        name="R6BotIkDemoNode",
        output="screen",
        parameters=[
            {
                "solver_name": "NewtonRaphson",
                "step_coefficient": 0.8,
                "initial_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                "target_position": [-0.091895, -0.627343, 1.622792,],
                "target_orientation_rpy": [1.044101, -0.709588, -1.391157,],
                "iteration_period_ms": 500,
                "max_demo_iterations": 200,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            ik_demo_node,
            rviz_node,
        ]
    )