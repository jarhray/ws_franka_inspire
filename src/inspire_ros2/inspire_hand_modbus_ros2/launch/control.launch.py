from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # mode=1：仅启动 hand_modbus_control_node（service 服务端）
    # mode=2：同时启动 control_node + 触觉 topic 发布节点
    mode = LaunchConfiguration('mode', default='2')
    publish_rate = LaunchConfiguration('publish_rate', default='10.0')
    spacename = LaunchConfiguration('spacename', default='inspire')
    name = LaunchConfiguration('name', default='hand1')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='2', description='1=仅 control 节点, 2=control+触觉发布'),
        DeclareLaunchArgument('publish_rate', default_value='10.0', description='topic 发布频率(Hz)'),
        DeclareLaunchArgument('spacename', default_value='inspire', description='topic 空间名前缀'),
        DeclareLaunchArgument('name', default_value='hand1', description='手名称前缀'),
        LogInfo(msg=['Current mode is: ', mode]),

        # 启动 hand_modbus_control_node
        Node(
            package='inspire_hand_modbus_ros2',
            executable='hand_modbus_control_node',
            name='hand_modbus_control_node',
            output='screen',
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'inspire_hand_modbus_ros2', 'inspire_hand_modbus_topic.py',
                '--ros-args',
                '-p', ['publish_rate:=', publish_rate],
                '-p', ['spacename:=', spacename],
                '-p', ['name:=', name],
            ],
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == '2'"])),
        ),
    ])
