from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    #自分で名前を決める(1号機、robot1)
    robot_name = 'robot1'
    # 1. sudo modprobe(カメラの準備)
    sudo_action = ExecuteProcess(
        cmd=['sudo', 'modprobe', 'v4l2loopback'],
        output='screen'
    )

    # 2. gst_loopback(カメラ映像の配信)
    gst_action = ExecuteProcess(
        cmd=['/home/student/libuvc-theta-sample/gst/gst_loopback', '--format', '2K'],
        cwd='/home/student/libuvc-theta/build',
        output='screen'
    )

    # 3.YOLO（検知）の起動
    yolo_node =  Node(
    package='ms_robot',
    executable='ms_robot',
    namespace=robot_name,#これが名札
    output='screen'
    )
    #4.ロボット本体の制御の起動
    robot_node = Node(
    package='ms_robot',
    executable='ms_robot',
    namespace=robot_name,
    output='screen'
    )
    #5.イベントハンドラー（sudoが終わったらカメラ配信を開始）
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=sudo_action,
            on_exit=[gst_action]
        )
    )
    #全部まとめて実行
    
    return LaunchDescription([sudo_action, event_handler, yolo_node, robot_node])
