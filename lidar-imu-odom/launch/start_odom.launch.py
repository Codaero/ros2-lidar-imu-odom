import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time')

    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom_matcher',
                'laser_frame': 'lidar',
                'publish_odom': '/odom_matcher',
                'publish_tf': True,
                'scan_sub_topic' : '/scan'
            }],
        executable='laser_scan_matcher',
        name='laser_scan_odom_publisher',
    )
    
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
    )
    
    imu_compfilter_node = Node(
        package='imu_complementary_filter', executable='complementary_filter_node',
        remappings=[('imu/data_raw', '/os1_cloud_node/imu')],
        parameters=[{
                'publish_debug_topics': False,
                'fixed_frame': "odom",
                'publish_tf': False,
            }],
        name='imu_complementary_filter'
    ),
    
    ekf_node = Node(
        package="robot_localization", executable="ekf_node",
        parameters=[os.path.join(get_package_share_directory("lidar-imu-odom"), 'config', 'ekf.yaml')],
        output='screen',
        name='ekf_filter_localization_node'
    )

    # Create and return the launch description
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                        description='Flag to enable use_sim_time'),
        odometry_node,
        pointcloud_to_laserscan_node,
        imu_compfilter_node
    ])

