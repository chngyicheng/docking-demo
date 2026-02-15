from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_pose_noise = LaunchConfiguration('enable_pose_noise', default='false')
    enable_scan_dropout = LaunchConfiguration('enable_scan_dropout', default='false')
    
    # Docking node
    docking_node = Node(
        package='docking_demo',
        executable='docking_node',
        name='docking_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('docking_demo'),
                'config',
                'params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Disturbance node
    # disturbance_node = Node(
    #     package='docking_demo',
    #     executable='disturbance_node',
    #     name='disturbance_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'enable_pose_noise': enable_pose_noise,
    #         'enable_scan_dropout': enable_scan_dropout
    #     }]
    # )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # DeclareLaunchArgument('enable_pose_noise', default_value='false'),
        # DeclareLaunchArgument('enable_scan_dropout', default_value='false'),
        docking_node
        # disturbance_node
    ])
