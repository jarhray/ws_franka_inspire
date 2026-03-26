from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # mode=1：仅启动 hand_modbus_control_node（service 服务端）
    # mode=2：同时启动 control_node + 触觉 topic 发布节点
    mode = LaunchConfiguration('mode', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='1', description='1=仅 control 节点, 2=control+触觉发布'),
        LogInfo(msg=['Current mode is: ', mode]),

        # 启动 hand_modbus_control_node
        Node(
            package='inspire_hand_modbus_ros2',
            executable='hand_modbus_control_node',
            name='hand_modbus_control_node',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'inspire_hand_modbus_ros2', 'inspire_hand_modbus_topic.py'],
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == '2'"])),
        ),
    ])

