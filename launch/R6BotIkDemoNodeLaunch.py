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
                "initial_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                "goal_joint_positions": [0.30, -0.45, 0.55, 0.25, -0.35, 0.15,],
                "iteration_period_ms": 1500,
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