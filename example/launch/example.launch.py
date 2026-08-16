import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. YDLidar 런치 파일 포함
    ydlidar_launch_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch'
    )
    ydlidar_launch_file = os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')
    ydlidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file)
    )

    # 2. iAHRS IMU 설정 파일 경로
    iahrs_pkg_share = get_package_share_directory("iahrs_ros2_driver")
    iahrs_driver_config = os.path.join(iahrs_pkg_share, "param", "config.yaml")

    return LaunchDescription([
        # YDLidar 실행
        ydlidar_include,

        # iAHRS IMU 실행
        Node(
            package="iahrs_ros2_driver",
            executable="iahrs_ros2_driver_node",
            name="iahrs_ros2_driver",
            output="screen",
            parameters=[iahrs_driver_config],
            emulate_tty=True,
        ),

        # LiDAR + IMU 각도 서버 전송 노드 실행
        Node(
            package='example',
            executable='lidar_sub',
            name='lidar_subscriber',
            output='screen'
        ),

        # GPS 웹 시각화 노드 실행
        Node(
            package='example',
            executable='gps_sub',
            name='gps_visualizer_node',
            output='screen'
        ),
    ])