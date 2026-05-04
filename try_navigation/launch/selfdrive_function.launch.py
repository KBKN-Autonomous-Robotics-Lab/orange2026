import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('try_navigation'),
        'config', 'config.rviz')
    assert os.path.exists(rviz_config_dir)
    #get livox data    
    livox_to_pointcloud2_launch_file = os.path.join(
        get_package_share_directory('livox_to_pointcloud2'),
        'launch',
        'livox_to_pointcloud2.launch.py'
    )
    ##################################################################################################################################
    # define config
    odom = LaunchConfiguration('odom')
    declare_odom_arg = DeclareLaunchArgument(
        'odom',
        default_value='/odom/wheel_imu',
        description='Odometry topic name'
    )
    # IGVC SelfDrive Quolification line stop test
    sd_quolification_line_stop = LaunchConfiguration('sd_quolification_line_stop')
    sd_quolification_line_stop_arg = DeclareLaunchArgument(
        'sd_quolification_line_stop',
        default_value='0',
        description='sd_quolification_line_stop name'
    )
    # IGVC SelfDrive III.1 function test
    sd_line_stop_set = LaunchConfiguration('sd_line_stop_set')
    sd_line_stop_set_arg = DeclareLaunchArgument(
        'sd_line_stop_set',
        default_value='0',
        description='sd_line_stop_set name'
    )
    # IGVC SelfDrive V.2 function test
    sd_human_stop = LaunchConfiguration('sd_human_stop')
    sd_human_stop_arg = DeclareLaunchArgument(
        'sd_human_stop',
        default_value='0',
        description='sd_human_stop name'
    )
    # IGVC SelfDrive Full
    sd_full_flag = LaunchConfiguration('sd_full_flag')
    sd_full_flag_arg = DeclareLaunchArgument(
        'sd_full_flag',
        default_value='0',
        description='sd_full_flag name'
    )
    
    # IGVC SD back flag
    sd_back_flag = LaunchConfiguration('sd_back_flag')
    sd_back_flag_arg = DeclareLaunchArgument(
        'sd_back_flag',
        default_value='0',
        description='sd_back_flag name'
    )
    
    
    #base speed 0.55 or 1.10
    speed_set = LaunchConfiguration('speed_set')
    speed_set_arg = DeclareLaunchArgument(
        'speed_set',
        default_value='0.55',
        description='speed_set name'
    )
    #DRIVE MODE #autonav:1 selfdrive:0
    functions_test = LaunchConfiguration('functions_test')
    functions_test_arg = DeclareLaunchArgument(
        'functions_test',
        default_value='0',
        description='functions_test name'
    )
    # IGVC SelfDrive Quolification line stop test shougaubutu shoukyo
    sd_line_stop_test = LaunchConfiguration('sd_line_stop_test')
    sd_line_stop_test_arg = DeclareLaunchArgument(
        'sd_line_stop_test',
        default_value='0',
        description='sd_line_stop_test name'
    )
    ##################################################################################################################################
    return LaunchDescription([
    
        declare_odom_arg, sd_quolification_line_stop_arg, sd_line_stop_set_arg, sd_human_stop_arg, sd_full_flag_arg, sd_back_flag_arg, speed_set_arg,  functions_test_arg, sd_line_stop_test_arg,
    
        #rviz2
        Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
        #get livox data
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_to_pointcloud2_launch_file)
        ),
        
        #pcd rotation
        Node(package='pcd_convert',
            executable='pcd_rotation',
            name='pcd_rotation_node',
            output='screen',
            arguments=[]
        ),
   
        #gps ekf edit
        Node(package='try_navigation',
            executable='ekf_myself_gps',
            name='sensor_fusion',
            output='screen',
            arguments=[]
        ),
        
        #pcd segmentation
        Node(package='pcd_convert',
            executable='pcd_height_segmentation',
            name='pcd_heigth_segmentation_node',
            output='screen',
            arguments=[]
        ),
        #odom wheel
        #Node(package='try_navigation',
        #    executable='odom_wheel',
        #    name='odom_wheel_node',
        #    output='screen',
        #    arguments=[]
        #),
        
        #odom combination
        Node(package='orange_gnss',
            executable='odom_combination',
            name='odom_combination',
            output='screen',
            arguments=[],
        ),
        
        #waypoint manager
        # waypoint gps command
        Node(package='navigation_control',
            executable='gps_waypoint',
            name='gps_waypoint',
            output='screen',
            parameters=[{'odom': odom}],
            arguments=[],
        ),
        # $ ros2 run navigation_control gps_waypoint
        # file path /ros2_ws/src/Use_action/navigation_control/navigation_control/gps_waypoint
        
        #reflection intensity map
        #Node(package='try_navigation',
        #    executable='reflection_intensity_map',
        #    name='reflection_intensity_map_node',
        #    output='screen',
        #    arguments=[],
        #),
        #path planning
        Node(package='try_navigation',
            executable='potential_astar',
            name='potential_astar_node',
            output='screen',
            parameters=[{'odom': odom},
                        {'functions_test': functions_test},
                        {'sd_line_stop_test': sd_line_stop_test}],
            arguments=[],
        ),
        #robot ctrl
        Node(package='try_navigation',
            executable='path_follower',
            name='path_follower_node',
            output='screen',
            parameters=[{'odom': odom},
                        {'sd_quolification_line_stop': sd_quolification_line_stop},
                        {'sd_line_stop_set': sd_line_stop_set},
                        {'sd_human_stop': sd_human_stop},
                        {'sd_full_flag': sd_full_flag},
                        {'sd_back_flag': sd_back_flag},
                        {'speed_set': speed_set}],
            arguments=[],
        ),
        
        #navigation start
        Node(package='navigation_control',
            executable='button',
            name='button',
            output='screen',
            arguments=[],
        ),
        #takamori Autonav
        Node(package='try_navigation',
            executable='reflection_to_pcd',
            name='reflection_to_pcd',
            output='screen',
            arguments=[],
        ),
        #takamori Selfdrive
        #Node(package='try_navigation',
        #    executable='self_drive_line',
        #    name='self_drive_line',
        #    output='screen',
        #    arguments=[],
        #),
    ])
