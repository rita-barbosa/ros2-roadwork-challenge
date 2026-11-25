import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "scenario": "1",
        "seed": "1",
        "work_shift": "0.0",
        "traffic_speed": "3.0",
        "noise": "0.02",
        "rviz": "true",
        "visualize": "true",
        "result_path": "roadwork_result.json",
        # Ablation switches (plan_node); the defaults are the final method.
        "use_traffic_gap": "true",
        "use_obstacle_check": "true",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    sim_launch = os.path.join(get_package_share_directory("roadwork_sim"), "launch", "challenge.launch.py")
    student_share = get_package_share_directory("roadwork_student")
    params_file = os.path.join(student_share, "config", "solution_params.yaml")
    rviz_config = os.path.join(student_share, "rviz", "solution.rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "scenario": LaunchConfiguration("scenario"),
            "seed": LaunchConfiguration("seed"),
            "work_shift": LaunchConfiguration("work_shift"),
            "traffic_speed": LaunchConfiguration("traffic_speed"),
            "noise": LaunchConfiguration("noise"),
            "rviz": "false",
            "student": "false",
            "visualize": LaunchConfiguration("visualize"),
            "result_path": LaunchConfiguration("result_path"),
        }.items(),
    )
    nodes = [
        Node(package="roadwork_student", executable="sense_node", output="screen", parameters=[params_file]),
        Node(
            package="roadwork_student",
            executable="plan_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_traffic_gap": ParameterValue(LaunchConfiguration("use_traffic_gap"), value_type=bool),
                    "use_obstacle_check": ParameterValue(LaunchConfiguration("use_obstacle_check"), value_type=bool),
                },
            ],
        ),
        Node(package="roadwork_student", executable="act_node", output="screen", parameters=[params_file]),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{"use_sim_time": True}],
            arguments=["-d", rviz_config],
        ),
    ]
    return LaunchDescription(arguments + [simulation] + nodes)
