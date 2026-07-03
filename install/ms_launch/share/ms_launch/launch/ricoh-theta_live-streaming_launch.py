from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # 1. sudo modprobe
    sudo_action = ExecuteProcess(
        cmd=['sudo', 'modprobe', 'v4l2loopback'],
        output='screen'
    )

    # 2. gst_loopback
    gst_action = ExecuteProcess(
        cmd=['/home/student/libuvc-theta-sample/gst/gst_loopback', '--format', '2K'],
        cwd='/home/student/libuvc-theta/build',
        output='screen'
    )

    # 3. sudo が完了したら gst_loopback を実行
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=sudo_action,
            on_exit=[gst_action]
        )
    )

    return LaunchDescription([sudo_action, event_handler])
