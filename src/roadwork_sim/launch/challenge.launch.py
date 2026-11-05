# Launch configuration only; all simulation and solution logic is C++.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    defaults = dict(scenario='1', seed='1', work_shift='0.0', traffic_speed='3.0',
                    noise='0.02', rviz='true', student='true', visualize='true',
                    result_path='roadwork_result.json')
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, t in
              [('scenario', int), ('seed', int), ('work_shift', float),
               ('traffic_speed', float), ('noise', float), ('visualize', bool),
               ('result_path', str)]}
    actions += [Node(package='roadwork_sim', executable='simulator_node', output='screen', parameters=[params]),
                Node(package='roadwork_student', executable='sense_node', output='screen', parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('student'))),
Node(package='roadwork_student', executable='plan_node', output='screen', parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('student'))),
Node(package='roadwork_student', executable='act_node', output='screen', parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('student'))),
                Node(package='rviz2', executable='rviz2', output='screen',
                     condition=IfCondition(LaunchConfiguration('rviz')),
                     parameters=[{'use_sim_time': True}],
                     arguments=['-d', os.path.join(get_package_share_directory('roadwork_sim'), 'rviz', 'challenge.rviz')])]
    return LaunchDescription(actions)
