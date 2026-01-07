#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动遥控驱动全流程：RemoteCore、FlexCore、driver_control"""

    # 节点名称可配置，便于复用/多实例
    driver_node_name_arg = DeclareLaunchArgument(
        "driver_node_name",
        default_value="driver_control",
        description="Python 驱动节点名称",
    )
    flex_node_name_arg = DeclareLaunchArgument(
        "flex_node_name",
        default_value="flex_control_node",
        description="FlexCore 主控节点名称",
    )
    remote_node_name_arg = DeclareLaunchArgument(
        "remote_node_name",
        default_value="remote_control_node",
        description="遥控解析节点名称",
    )

    # 遥控数据解析节点（负责发布 remote_ctrl_data）
    remote_core_node = Node(
        package="flex_core",
        executable="remote_control_core_node",
        name=LaunchConfiguration("remote_node_name"),
        output="screen",
        emulate_tty=True,
    )

    # 主控制节点（订阅遥控数据，调用电机服务）
    flex_core_node = Node(
        package="flex_core",
        executable="flex_control_core_node",
        name=LaunchConfiguration("flex_node_name"),
        output="screen",
        emulate_tty=True,
    )

    # 电机驱动服务节点
    driver_control_node = Node(
        package="flex_core",
        executable="driver_control.py",
        name=LaunchConfiguration("driver_node_name"),
        output="screen",
        emulate_tty=True,
    )

    # 当遥控器节点启动后，启动电机驱动节点
    start_driver_control_event = RegisterEventHandler(
        OnProcessStart(
            target_action=remote_core_node,
            on_start=[driver_control_node],
        )
    )

    # 当电机驱动节点启动后，启动主控制节点
    start_flex_core_event = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_control_node,
            on_start=[flex_core_node],
        )
    )

    return LaunchDescription(
        [
            driver_node_name_arg,
            flex_node_name_arg,
            remote_node_name_arg,
            remote_core_node,  # 首先启动遥控器节点
            start_driver_control_event,  # 遥控器节点启动后启动电机驱动节点
            start_flex_core_event,  # 电机驱动节点启动后启动主控制节点
        ]
    )

