import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('my_robot_description')
    #tb3_pkg     = get_package_share_directory('turtlebot3_description')
    # Using Turtlebot3 Waffle model temporarily
    xacro_file  = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds',  'cylinder3.world')
    rviz_config = os.path.join(pkg, 'rviz',    'robot.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([

        # 1 — Gazebo with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            on_exit=Shutdown()
        ),

        # 2 — Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]
        ),

        # 3 — Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ]
        ),

        # 4 — RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
