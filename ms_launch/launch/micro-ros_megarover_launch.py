from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # 1. USB2 の権限変更（sudo）
    chmod_action = ExecuteProcess(
        cmd=['sudo', 'chmod', '666', '/dev/ttyUSB2'],
        output='screen'
    )

    # 2. micro-ROS agent の起動
    micro_ros_action = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--baudrate', '115200', '--dev', '/dev/ttyUSB2'
        ],
        output='screen'
    )

    # 3. chmod が完了したら micro-ROS agent を起動
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=chmod_action,
            on_exit=[micro_ros_action]
        )
    )

    return LaunchDescription([chmod_action, event_handler])
