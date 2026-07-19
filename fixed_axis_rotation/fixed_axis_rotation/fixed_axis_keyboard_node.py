#!/usr/bin/env python3
"""
fixed_axis_keyboard_node.py


单独终端启动，不能直接由 launch 文件启动。

ros2 run fixed_axis_rotation fixed_axis_keyboard_node \
  --ros-args \
  --params-file \
  ~/franka_ros2_ws/src/fixed_axis_rotation/config/fixed_axis_rotation.yaml

功能：
    创建固定轴旋转键盘控制节点。

    本节点从独立终端读取键盘输入，
    将按键转换为统一的固定轴旋转控制命令，
    并发布到 fixed_axis_rotation_node。

接口：
    keyboard_timer_callback()
    process_key(key)
    publish_current_command()
    publish_command(command, target_angular_speed)
    restore_terminal()

输入：
    键盘按键：

        W:
            开始或继续旋转。
            当前目标角速度为 0 时，
            使用默认正向角速度启动。

        + / D:
            增大目标角速度绝对值。

        - / A:
            减小目标角速度绝对值。

        R:
            反转旋转方向。

        Space:
            正常停止。
            主控制节点按照角加速度限制平滑减速。

        E:
            紧急停止。
            主控制节点立即发送零关节速度。

        C:
            根据当前 fr3_link8 位姿重新记录固定轴。
            重新记录时保持停止。

        Q:
            紧急停止并退出键盘节点。

输出：
    command_topic:
        fixed_axis_rotation_interfaces/msg/FixedAxisCommand

        command:
            控制指令类型。

        target_angular_speed:
            目标角速度，单位 rad/s。
            正负号表示旋转方向。

方法：
    1. 将终端设置为非阻塞按键读取模式。
    2. 使用 ROS 2 定时器周期检查键盘输入。
    3. 在本节点内部维护目标角速度和运行状态。
    4. 持续发布控制命令，作为主控制节点的命令心跳。
    5. 节点退出时恢复终端设置。

说明：
    本节点只负责将键盘输入转换为控制命令，
    不计算末端速度、雅可比矩阵或关节速度。

    本节点必须在独立的交互式终端中运行，
    不建议直接由 launch 文件启动。
"""

import select
import sys
import termios
import tty

import numpy as np
import rclpy
from rclpy.node import Node

from fixed_axis_rotation_interfaces.msg import (
    FixedAxisCommand,
)


class FixedAxisKeyboardNode(Node):
    """
    固定轴旋转键盘控制节点。
    """

    def __init__(self):
        super().__init__(
            "fixed_axis_keyboard_node"
        )

        # =====================================================
        # 声明参数
        # =====================================================

        self.declare_parameter(
            "command_topic"
        )

        self.declare_parameter(
            "publish_rate_hz"
        )

        self.declare_parameter(
            "default_angular_speed"
        )

        self.declare_parameter(
            "angular_speed_step"
        )

        self.declare_parameter(
            "max_angular_speed"
        )

        # =====================================================
        # 读取参数
        # =====================================================

        self.command_topic = str(
            self.get_required_parameter(
                "command_topic"
            )
        )

        self.publish_rate_hz = float(
            self.get_required_parameter(
                "publish_rate_hz"
            )
        )

        self.default_angular_speed = float(
            self.get_required_parameter(
                "default_angular_speed"
            )
        )

        self.angular_speed_step = float(
            self.get_required_parameter(
                "angular_speed_step"
            )
        )

        self.max_angular_speed = float(
            self.get_required_parameter(
                "max_angular_speed"
            )
        )

        self.validate_parameters()

        # =====================================================
        # 检查终端
        # =====================================================

        if not sys.stdin.isatty():
            raise RuntimeError(
                "Keyboard node must run in an "
                "interactive terminal."
            )

        # 保存终端原始设置，
        # 节点退出时必须恢复。
        self.original_terminal_settings = (
            termios.tcgetattr(
                sys.stdin
            )
        )

        self.terminal_restored = False

        # 使用 cbreak 模式读取单个按键，
        # 不需要按 Enter。
        tty.setcbreak(
            sys.stdin.fileno()
        )

        # =====================================================
        # 当前键盘控制状态
        # =====================================================

        # 当前持续发布的控制指令。
        self.current_command = (
            FixedAxisCommand.COMMAND_STOP
        )

        # 当前目标角速度。
        self.target_angular_speed = 0.0

        # 一次性指令。
        # 例如重新记录固定轴，只发送一次。
        self.pending_one_shot_command = None

        # 节点是否正在退出。
        self.exit_requested = False

        # =====================================================
        # ROS 通信
        # =====================================================

        self.command_publisher = (
            self.create_publisher(
                FixedAxisCommand,
                self.command_topic,
                10
            )
        )

        timer_period = (
            1.0 /
            self.publish_rate_hz
        )

        self.keyboard_timer = (
            self.create_timer(
                timer_period,
                self.keyboard_timer_callback
            )
        )

        # =====================================================
        # 启动信息
        # =====================================================

        self.get_logger().info(
            "Fixed-axis keyboard node started."
        )

        self.get_logger().info(
            "Command topic: "
            f"{self.command_topic}"
        )

        self.get_logger().info(
            "Publish rate: "
            f"{self.publish_rate_hz} Hz"
        )

        self.get_logger().info(
            "Default angular speed: "
            f"{self.default_angular_speed:.4f} rad/s"
        )

        self.get_logger().info(
            "Angular speed step: "
            f"{self.angular_speed_step:.4f} rad/s"
        )

        self.get_logger().info(
            "Maximum angular speed: "
            f"{self.max_angular_speed:.4f} rad/s"
        )

        self.print_keyboard_help()
        self.print_status()

    def get_required_parameter(
        self,
        name
    ):
        """
        读取必须由 YAML 或命令行提供的参数。

        参数未设置时直接报错。
        """

        parameter = self.get_parameter(
            name
        )

        if (
            parameter.type_
            == rclpy.Parameter.Type.NOT_SET
        ):
            raise ValueError(
                f"Required parameter '{name}' "
                "is not set."
            )

        return parameter.value

    def validate_parameters(self):
        """
        检查键盘控制参数是否合法。
        """

        if not self.command_topic:
            raise ValueError(
                "command_topic must not be empty."
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError(
                "publish_rate_hz must be positive."
            )

        if self.default_angular_speed <= 0.0:
            raise ValueError(
                "default_angular_speed "
                "must be positive."
            )

        if self.angular_speed_step <= 0.0:
            raise ValueError(
                "angular_speed_step "
                "must be positive."
            )

        if self.max_angular_speed <= 0.0:
            raise ValueError(
                "max_angular_speed "
                "must be positive."
            )

        if (
            self.default_angular_speed
            > self.max_angular_speed
        ):
            raise ValueError(
                "default_angular_speed must not "
                "exceed max_angular_speed."
            )

        if (
            self.angular_speed_step
            > self.max_angular_speed
        ):
            raise ValueError(
                "angular_speed_step must not "
                "exceed max_angular_speed."
            )

    def print_keyboard_help(self):
        """
        打印键盘操作说明。
        """

        print()
        print("==========================================")
        print(" Fixed Axis Rotation Keyboard Controller")
        print("==========================================")
        print(" W       : start / continue rotation")
        print(" D or +  : increase speed magnitude")
        print(" A or -  : decrease speed magnitude")
        print(" R       : reverse direction")
        print(" Space   : normal stop")
        print(" E       : emergency stop")
        print(" C       : capture current fixed axis")
        print(" Q       : emergency stop and quit")
        print(" H       : show this help")
        print("==========================================")
        print()

    def print_status(self):
        """
        打印当前控制状态。
        """

        command_name = {
            FixedAxisCommand.COMMAND_STOP:
                "STOP",

            FixedAxisCommand.COMMAND_RUN:
                "RUN",

            FixedAxisCommand.COMMAND_EMERGENCY_STOP:
                "EMERGENCY_STOP",

            FixedAxisCommand.COMMAND_CAPTURE_AXIS:
                "CAPTURE_AXIS",
        }.get(
            self.current_command,
            "UNKNOWN"
        )

        print(
            "Command: "
            f"{command_name} | "
            "Target angular speed: "
            f"{self.target_angular_speed:+.4f} rad/s"
        )

    def keyboard_timer_callback(self):
        """
        周期读取键盘并发布控制命令。
        """

        try:
            # 一次定时器周期读取当前已经到达的所有按键。
            while select.select(
                [sys.stdin],
                [],
                [],
                0.0
            )[0]:
                key = sys.stdin.read(1)

                self.process_key(
                    key
                )

                if self.exit_requested:
                    return

            self.publish_current_command()

        except Exception as error:
            self.get_logger().error(
                "Keyboard processing failed: "
                f"{error}"
            )

            self.publish_command(
                FixedAxisCommand.COMMAND_EMERGENCY_STOP,
                0.0
            )

            self.exit_requested = True

            if rclpy.ok():
                rclpy.shutdown()

    def process_key(
        self,
        key
    ):
        """
        将单个键盘按键转换为控制状态。
        """

        if not key:
            return

        key_lower = key.lower()

        # =====================================================
        # 开始或继续运行
        # =====================================================

        if key_lower == "w":

            if np.isclose(
                self.target_angular_speed,
                0.0
            ):
                self.target_angular_speed = (
                    self.default_angular_speed
                )

            self.current_command = (
                FixedAxisCommand.COMMAND_RUN
            )

            self.print_status()
            return

        # =====================================================
        # 增大角速度绝对值
        # =====================================================

        if (
            key_lower == "d"
            or key == "+"
            or key == "="
        ):
            direction = np.sign(
                self.target_angular_speed
            )

            if direction == 0.0:
                direction = 1.0

            speed_magnitude = abs(
                self.target_angular_speed
            )

            speed_magnitude += (
                self.angular_speed_step
            )

            speed_magnitude = min(
                speed_magnitude,
                self.max_angular_speed
            )

            self.target_angular_speed = (
                direction *
                speed_magnitude
            )

            self.current_command = (
                FixedAxisCommand.COMMAND_RUN
            )

            self.print_status()
            return

        # =====================================================
        # 减小角速度绝对值
        # =====================================================

        if (
            key_lower == "a"
            or key == "-"
            or key == "_"
        ):
            direction = np.sign(
                self.target_angular_speed
            )

            if direction == 0.0:
                direction = 1.0

            speed_magnitude = abs(
                self.target_angular_speed
            )

            speed_magnitude -= (
                self.angular_speed_step
            )

            speed_magnitude = max(
                0.0,
                speed_magnitude
            )

            self.target_angular_speed = (
                direction *
                speed_magnitude
            )

            if np.isclose(
                speed_magnitude,
                0.0
            ):
                self.target_angular_speed = 0.0

                self.current_command = (
                    FixedAxisCommand.COMMAND_STOP
                )
            else:
                self.current_command = (
                    FixedAxisCommand.COMMAND_RUN
                )

            self.print_status()
            return

        # =====================================================
        # 反转方向
        # =====================================================

        if key_lower == "r":

            if np.isclose(
                self.target_angular_speed,
                0.0
            ):
                self.target_angular_speed = (
                    -self.default_angular_speed
                )
            else:
                self.target_angular_speed = (
                    -self.target_angular_speed
                )

            self.current_command = (
                FixedAxisCommand.COMMAND_RUN
            )

            self.print_status()
            return

        # =====================================================
        # 正常停止
        # =====================================================

        if key == " ":
            self.target_angular_speed = 0.0

            self.current_command = (
                FixedAxisCommand.COMMAND_STOP
            )

            self.print_status()
            return

        # =====================================================
        # 紧急停止
        # =====================================================

        if key_lower == "e":
            self.target_angular_speed = 0.0

            self.current_command = (
                FixedAxisCommand.COMMAND_EMERGENCY_STOP
            )

            self.print_status()
            return

        # =====================================================
        # 重新记录固定轴
        # =====================================================

        if key_lower == "c":
            self.target_angular_speed = 0.0

            self.current_command = (
                FixedAxisCommand.COMMAND_STOP
            )

            self.pending_one_shot_command = (
                FixedAxisCommand.COMMAND_CAPTURE_AXIS
            )

            print(
                "Capture-axis command requested."
            )
            return

        # =====================================================
        # 打印帮助
        # =====================================================

        if key_lower == "h":
            self.print_keyboard_help()
            self.print_status()
            return

        # =====================================================
        # 停止并退出
        # =====================================================

        if key_lower == "q":
            self.target_angular_speed = 0.0

            self.current_command = (
                FixedAxisCommand.COMMAND_EMERGENCY_STOP
            )

            self.publish_command(
                FixedAxisCommand.COMMAND_EMERGENCY_STOP,
                0.0
            )

            self.exit_requested = True

            self.get_logger().info(
                "Quit requested. "
                "Emergency-stop command sent."
            )

            if rclpy.ok():
                rclpy.shutdown()

    def publish_current_command(self):
        """
        发布当前控制状态。

        正常状态会周期发布，
        作为主控制节点的命令心跳。
        """

        if self.pending_one_shot_command is not None:

            self.publish_command(
                self.pending_one_shot_command,
                0.0
            )

            self.pending_one_shot_command = None
            return

        self.publish_command(
            self.current_command,
            self.target_angular_speed
        )

    def publish_command(
        self,
        command,
        target_angular_speed
    ):
        """
        发布一条固定轴控制命令。

        输入：
            command:
                控制指令常量。

            target_angular_speed:
                带符号目标角速度，单位 rad/s。
        """

        target_angular_speed = float(
            target_angular_speed
        )

        if not np.isfinite(
            target_angular_speed
        ):
            raise ValueError(
                "target_angular_speed contains "
                "nan or inf."
            )

        target_angular_speed = float(
            np.clip(
                target_angular_speed,
                -self.max_angular_speed,
                self.max_angular_speed
            )
        )

        message = FixedAxisCommand()

        message.command = int(
            command
        )

        message.target_angular_speed = (
            target_angular_speed
        )

        self.command_publisher.publish(
            message
        )

    def restore_terminal(self):
        """
        恢复终端原始设置。
        """

        if self.terminal_restored:
            return

        if hasattr(
            self,
            "original_terminal_settings"
        ):
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.original_terminal_settings
            )

        self.terminal_restored = True

    def destroy_node(self):
        """
        销毁节点前恢复终端。
        """

        self.restore_terminal()

        return super().destroy_node()


def main(args=None):
    """
    ROS 2 节点入口。
    """

    rclpy.init(
        args=args
    )

    node = None

    try:
        node = FixedAxisKeyboardNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Keyboard interrupt. "
                "Emergency-stop command sent."
            )

            node.publish_command(
                FixedAxisCommand.COMMAND_EMERGENCY_STOP,
                0.0
            )

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Unexpected error: {error}"
            )

            node.publish_command(
                FixedAxisCommand.COMMAND_EMERGENCY_STOP,
                0.0
            )

        raise

    finally:
        if node is not None:
            node.restore_terminal()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()