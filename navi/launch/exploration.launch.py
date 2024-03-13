from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    max_step_par = LaunchConfiguration('max_step', default=5)

    robot_id_arg = LaunchConfiguration(
        'robotID',
        default='0',
    )
    number_of_cluster_arg = LaunchConfiguration(
        'NUMBER_OF_CLUSTER',
        default='5',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
                                            '/turtlebot3_world.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch'),
                                            '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                                            '/navigation_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='navi',
            executable='odom2map',
            output='screen'
        ),
        Node(
            package='navi',
            executable='action_nav',
            output='screen',
            parameters=[{'max_step': max_step_par}],
        ),
        Node(
            package='frontier_detector_pkg',
            executable='fd',
            output='screen',
            parameters=[{'NUMBER_OF_CLUSTER': number_of_cluster_arg}]
        ),
        Node(
            package='vis_point',
            executable='vis',
            output='screen'
        ),
        Node(
            package='navi',
            executable='testnode',
            output='screen',
            parameters=[{'robotID': robot_id_arg}]
        )

    ])