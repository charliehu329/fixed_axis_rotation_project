#!/usr/bin/env python3
"""
fixed_axis_rotation.launch.py

功能：
    启动 Franka FR3 固定空间轴旋转主控制节点。

启动内容：
    fixed_axis_rotation_node:
        1. 接收 Franka 当前关节状态。
        2. 接收固定轴旋转控制命令。
        3. 计算固定轴旋转末端速度。
        4. 将末端速度转换为七维关节速度。
        5. 根据 dry_run 参数决定是否向底层控制器发布。

接口：
    ros2 launch fixed_axis_rotation fixed_axis_rotation.launch.py

可选参数：
    params_file:
        YAML 参数文件路径。

    dry_run:
        true:
            只计算，不向机器人发布关节速度。

        false:
            向底层速度控制器发布关节速度。

说明：
    fixed_axis_keyboard_node 需要读取交互式终端键盘，
    因此不在本 launch 文件中启动。

    键盘节点需要在另一个终端中使用 ros2 run 单独启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    创建固定轴旋转控制系统启动描述。
    """

    # =========================================================
    # 默认参数文件
    # =========================================================

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "fixed_axis_rotation"
            ),
            "config",
            "fixed_axis_rotation.yaml",
        ]
    )

    # =========================================================
    # Launch 参数
    # =========================================================

    params_file_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "Path to the fixed-axis rotation "
            "YAML parameter file."
        )
    )

    dry_run_argument = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        choices=[
            "true",
            "false",
        ],
        description=(
            "If true, calculate commands without "
            "publishing joint velocity to the robot."
        )
    )

    # =========================================================
    # 主控制节点
    # =========================================================

    fixed_axis_rotation_node = Node(
        package="fixed_axis_rotation",
        executable="fixed_axis_rotation_node",
        name="fixed_axis_rotation_node",
        output="screen",

        parameters=[
            LaunchConfiguration(
                "params_file"
            ),

            # launch 命令中的 dry_run
            # 覆盖 YAML 文件中的 dry_run。
            {
                "dry_run":
                    LaunchConfiguration(
                        "dry_run"
                    )
            },
        ],
    )

    # =========================================================
    # 返回启动描述
    # =========================================================

    return LaunchDescription(
        [
            params_file_argument,
            dry_run_argument,
            fixed_axis_rotation_node,
        ]
    )