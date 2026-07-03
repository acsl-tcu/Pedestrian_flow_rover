from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. カメラ準備
    sudo_action = ExecuteProcess(
        cmd=['sudo', 'modprobe', 'v4l2loopback'],
        output='screen'
    )

    # 2. カメラ配信 (sudo完了後に実行)
    gst_action = ExecuteProcess(
        cmd=['/home/student/libuvc-theta-sample/gst/gst_loopback', '--format', '2K'],
        cwd='/home/student/libuvc-theta/build',
        output='screen'
    )

    # 3. Lidar起動
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar_s1_launch.py'
            )
        ])
    )

    return LaunchDescription([
        lidar_launch,
        sudo_action,
        RegisterEventHandler(
            OnProcessExit(target_action=sudo_action, on_exit=[gst_action])
        )
    ])
