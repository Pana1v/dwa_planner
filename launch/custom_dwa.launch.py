import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_description_pkg = get_package_share_directory('turtlebot3_description')
    rviz_config_file = os.path.join(
        get_package_share_directory('dwa_planner'),
        'rviz',
        'default.rviz'
    )

    # Launch TurtleBot3 Gazebo world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Launch RViz2 with local config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Launch DWA Planner node in a new terminal
    # dwa_node_terminal = ExecuteProcess(
    #     cmd=[
    #         'gnome-terminal', '--',
    #         'ros2', 'run', 'dwa_planner', 'dwa_node'
    #     ],
    #     output='screen'
    # )


    return LaunchDescription([
        gazebo_launch,
        rviz2_node,
    ])