from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name = 'robot1'

    # 1. カメラ準備 (これが終わるのを待つ)
    sudo_action = ExecuteProcess(
        cmd=['sudo', 'modprobe', 'v4l2loopback'],
        output='screen'
    )

    # 2. カメラ配信
    gst_action = ExecuteProcess(
        cmd=['/home/student/libuvc-theta-sample/gst/gst_loopback', '--format', '2K'],
        cwd='/home/student/libuvc-theta/build',
        output='screen'
    )

    # 3. YOLO起動
    yolo_node = Node(
        package='ms_yolo',
        executable='ms_yolo',
        namespace=robot_name,
        output='screen'
    )

    # 4. ロボット制御起動
    robot_node = Node(
        package='ms_robot',
        executable='ms_robot',
        namespace=robot_name,
        output='screen'
    )

    # 5. Lidar起動 (これは最初から動いてOK)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar_s1_launch.py'
            )
        ])
    )

    # 6. イベントハンドラー (sudoが終わったら、配信・YOLO・制御をまとめて起動！)
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=sudo_action,
            on_exit=[gst_action, yolo_node, robot_node]
        )
    )

    return LaunchDescription([
        lidar_launch, # すぐ起動
        sudo_action,  # すぐ起動（パスワード入力待ち）
        event_handler # sudoが終わるのを監視して残りを起動
    ])