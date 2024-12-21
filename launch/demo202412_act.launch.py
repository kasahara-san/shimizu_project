import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ic120_bringup_dir = get_package_share_directory('ic120_bringup')
    zx200_bringup_dir = get_package_share_directory('zx200_bringup')
    tms_ts_launch_dir = get_package_share_directory('tms_ts_launch')
    shimizu_project_dir = get_package_share_directory('shimizu_project')
    tms_if_for_opera_dir = get_package_share_directory('tms_if_for_opera')


    ic120_remote_launch_file = os.path.join(ic120_bringup_dir, 'launch', 'ic120_remote.launch.py')
    ic120_vehicle_launch_file = os.path.join(ic120_bringup_dir, 'launch', 'ic120_vehicle.launch.py')



    zx200_vehicle_launch_file = os.path.join(zx200_bringup_dir, 'launch', 'vehicle.launch.py')
    tms_ts_demo202412_launch_file = os.path.join(tms_ts_launch_dir, 'launch', 'tms_ts_demo202412.launch.py')
    tms_if_for_opera_launch_file = os.path.join(tms_if_for_opera_dir, 'launch', 'tms_if_for_opera.launch.py')
    sample_rviz_file = os.path.join(shimizu_project_dir, 'rviz2', 'sample.rviz')
    return LaunchDescription([

        DeclareLaunchArgument('prefix_ic120_1', default_value='ic120'),
        DeclareLaunchArgument('use_rviz_ic120', default_value='false'),
        DeclareLaunchArgument('use_rviz_zx200', default_value='true'),
        DeclareLaunchArgument('robot_name_zx200', default_value='zx200'),
        DeclareLaunchArgument('task_id1', default_value='10'), # 8: ic120 (for the simulation), 10: ic120 (for the actualk machinery)
        DeclareLaunchArgument('task_id2', default_value='9'),
        DeclareLaunchArgument('command_interface_name_zx200', default_value='velocity'),
        DeclareLaunchArgument('collision_object_record_name', default_value='collision_objects'),

        
        # ic120_remote起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_remote_launch_file),
        ),

        # ic120_vehicle起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ic120_vehicle_launch_file),
        ),

        # zx200_vehicle起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zx200_vehicle_launch_file),
            launch_arguments={'robot_name': launch.substitutions.LaunchConfiguration('robot_name_zx200'),
                              'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz_zx200'),
                              'command_interface_name': launch.substitutions.LaunchConfiguration('command_interface_name_zx200')}.items()
        ),

        # タスク管理機構(ROS2-TMS for Construction)起動用
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tms_ts_demo202412_launch_file),
            launch_arguments={
                'task_id1': launch.substitutions.LaunchConfiguration('task_id1'),
                'task_id2': launch.substitutions.LaunchConfiguration('task_id2')
                }.items(),
        ),

        # tms_if_for_opera起動用
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tms_if_for_opera_launch_file),
            launch_arguments={'collision_object_record_name': launch.substitutions.LaunchConfiguration('collision_object_record_name')}.items()
        ),


        # # Node(
        # #     package='shimizu_project',
        # #     executable='operasim_physx_default_map_publisher',
        # #     name='operasim_physx_default_map_publisher',
        # #     namespace = 'map_coodinater',
        # #     parameters=[{'image_file': 'shimizu_project'},
        # #                 {'origin_x': -49.0},
        # #                 {'origin_y': -47.0}]),
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{'use_sim_time': True}],
            arguments=["--display-config", sample_rviz_file]),
    ])