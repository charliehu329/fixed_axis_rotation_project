#!/usr/bin/env python3
"""
fixed_axis_rotation_node.py

功能：
    创建 Franka FR3 固定空间轴旋转控制节点。

    节点首先根据初始 fr3_link8 位姿，
    将 link8 坐标系中定义的轴点和轴方向转换到世界坐标系，
    然后将该空间轴固定。

    收到旋转命令后，节点生成围绕固定空间轴旋转的末端期望位姿，
    计算末端前馈速度和位姿误差反馈速度，
    再通过雅可比矩阵转换为七维关节速度。

功能流程：
    1. 订阅当前七维关节状态。
    2. 订阅键盘控制命令。
    3. 根据初始末端位姿记录固定空间轴。
    4. 根据目标角速度更新旋转角度。
    5. 计算固定轴旋转对应的期望末端位姿。
    6. 计算末端前馈线速度和角速度。
    7. 根据位置和姿态误差增加反馈修正。
    8. 将六维末端速度转换为七维关节速度。
    9. 检查关节限位、雅可比奇异性和消息超时。
    10. 对末端速度和关节速度进行限幅。
    11. 发布七维关节速度给 Franka 底层控制器。
    12. 发生故障、急停或节点退出时发送零关节速度。

接口：
    joint_state_callback(msg)
    command_callback(msg)
    timer_callback()
    capture_fixed_axis(p_world_end, R_world_end)
    compute_desired_pose()
    compute_cartesian_velocity(...)
    enter_safe_stop(reason)
    publish_zero_velocity()

输入：
    joint_state_topic:
        sensor_msgs/msg/JointState

        当前七个关节角：

        [
            q1,
            q2,
            q3,
            q4,
            q5,
            q6,
            q7
        ]

    command_topic:
        fixed_axis_rotation_interfaces/msg/FixedAxisCommand

        包含：
            command
            target_angular_speed

    axis_point_link8:
        固定轴上一点在初始 fr3_link8 坐标系中的位置。
        单位为 m。

    axis_direction_link8:
        固定轴方向在初始 fr3_link8 坐标系中的表达。

输出：
    joint_velocity_command_topic:
        std_msgs/msg/Float64MultiArray

        七维关节速度：

        [
            dq1,
            dq2,
            dq3,
            dq4,
            dq5,
            dq6,
            dq7
        ]

        单位为 rad/s。

方法：
    固定轴在世界坐标系中的位置和方向：

        p_axis_world =
            p_world_end_initial
            + R_world_end_initial @ p_axis_link8

        axis_world =
            R_world_end_initial @ axis_direction_link8

    旋转角度为 theta 时的期望末端位姿：

        p_desired =
            p_axis_world
            + R_axis(theta)
            @ (p_initial - p_axis_world)

        R_desired =
            R_axis(theta)
            @ R_initial

    固定轴旋转前馈速度：

        omega =
            angular_speed * axis_world

        v =
            omega
            x (p_desired - p_axis_world)

    最终末端速度：

        V_command =
            V_feedforward
            + V_feedback

说明：
    1. 本节点控制的是整个 fr3_link8 刚体绕固定空间轴旋转。
    2. 固定轴记录后不会随机器人末端坐标系继续移动。
    3. 普通停止按照最大角加速度平滑减速。
    4. 紧急停止和安全故障立即输出零关节速度。
    5. 初次真机测试必须使用 dry_run=true。
"""

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from fixed_axis_rotation.robot_kinematics import (
    FrankaKinematics,
)
from fixed_axis_rotation.safety import (
    check_finite_vector,
    check_jacobian_condition,
    check_joint_position_limits,
    limit_angular_acceleration,
    limit_cartesian_velocity,
    limit_joint_acceleration,
    limit_joint_velocity,
)
from fixed_axis_rotation.velocity_mapper import (
    cartesian_velocity_to_joint_velocity,
)
from fixed_axis_rotation_interfaces.msg import (
    FixedAxisCommand,
)


class FixedAxisRotationNode(Node):
    """
    Franka FR3 固定空间轴旋转控制节点。
    """

    def __init__(self):
        super().__init__(
            "fixed_axis_rotation_node"
        )

        # =====================================================
        # 声明参数
        # =====================================================

        # 运动学模型
        self.declare_parameter(
            "urdf_path",
            Parameter.Type.STRING
        )

        self.declare_parameter(
            "end_effector_frame",
            Parameter.Type.STRING
        )

        # ROS 2 Topic
        self.declare_parameter(
            "joint_state_topic",
            Parameter.Type.STRING
        )

        self.declare_parameter(
            "command_topic",
            Parameter.Type.STRING
        )

        self.declare_parameter(
            "joint_velocity_command_topic",
            Parameter.Type.STRING
        )

        # 固定轴定义
        self.declare_parameter(
            "axis_point_link8",
            Parameter.Type.DOUBLE_ARRAY
        )

        self.declare_parameter(
            "axis_direction_link8",
            Parameter.Type.DOUBLE_ARRAY
        )

        self.declare_parameter(
            "capture_axis_on_start",
            Parameter.Type.BOOL
        )

        # 控制参数
        self.declare_parameter(
            "publish_rate_hz",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "damping",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_angular_acceleration",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "position_gain",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "angular_gain",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "duration_sec",
            Parameter.Type.DOUBLE
        )

        # 速度限制
        self.declare_parameter(
            "max_joint_velocity",
            Parameter.Type.DOUBLE
        )

        # 加速度限制
        self.declare_parameter(
            "max_joint_acceleration",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_linear_velocity",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_angular_velocity",
            Parameter.Type.DOUBLE
        )

        # 安全参数
        self.declare_parameter(
            "joint_position_margin",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "min_jacobian_singular_value",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_jacobian_condition_number",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_position_error",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "max_orientation_error",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "joint_state_timeout_sec",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "command_timeout_sec",
            Parameter.Type.DOUBLE
        )

        self.declare_parameter(
            "angular_speed_zero_threshold",
            Parameter.Type.DOUBLE
        )

        # 运行模式
        self.declare_parameter(
            "dry_run",
            Parameter.Type.BOOL
        )

        self.declare_parameter(
            "log_period_sec",
            Parameter.Type.DOUBLE
        )

        # =====================================================
        # 读取参数
        # =====================================================

        self.urdf_path = str(
            self.get_required_parameter(
                "urdf_path"
            )
        )

        self.end_effector_frame = str(
            self.get_required_parameter(
                "end_effector_frame"
            )
        )

        self.joint_state_topic = str(
            self.get_required_parameter(
                "joint_state_topic"
            )
        )

        self.command_topic = str(
            self.get_required_parameter(
                "command_topic"
            )
        )

        self.joint_velocity_command_topic = str(
            self.get_required_parameter(
                "joint_velocity_command_topic"
            )
        )

        self.axis_point_link8 = np.asarray(
            self.get_required_parameter(
                "axis_point_link8"
            ),
            dtype=float
        ).reshape(-1)

        self.axis_direction_link8 = np.asarray(
            self.get_required_parameter(
                "axis_direction_link8"
            ),
            dtype=float
        ).reshape(-1)

        self.capture_axis_on_start = bool(
            self.get_required_parameter(
                "capture_axis_on_start"
            )
        )

        self.publish_rate_hz = float(
            self.get_required_parameter(
                "publish_rate_hz"
            )
        )

        self.damping = float(
            self.get_required_parameter(
                "damping"
            )
        )

        self.max_angular_acceleration = float(
            self.get_required_parameter(
                "max_angular_acceleration"
            )
        )

        self.position_gain = float(
            self.get_required_parameter(
                "position_gain"
            )
        )

        self.angular_gain = float(
            self.get_required_parameter(
                "angular_gain"
            )
        )

        self.duration_sec = float(
            self.get_required_parameter(
                "duration_sec"
            )
        )

        self.max_joint_velocity = float(
            self.get_required_parameter(
                "max_joint_velocity"
            )
        )

        self.max_joint_acceleration = float(
            self.get_required_parameter(
                "max_joint_acceleration"
            )
        )

        self.max_linear_velocity = float(
            self.get_required_parameter(
                "max_linear_velocity"
            )
        )

        self.max_angular_velocity = float(
            self.get_required_parameter(
                "max_angular_velocity"
            )
        )

        self.joint_position_margin = float(
            self.get_required_parameter(
                "joint_position_margin"
            )
        )

        self.min_jacobian_singular_value = float(
            self.get_required_parameter(
                "min_jacobian_singular_value"
            )
        )

        self.max_jacobian_condition_number = float(
            self.get_required_parameter(
                "max_jacobian_condition_number"
            )
        )

        self.max_position_error = float(
            self.get_required_parameter(
                "max_position_error"
            )
        )

        self.max_orientation_error = float(
            self.get_required_parameter(
                "max_orientation_error"
            )
        )

        self.joint_state_timeout_sec = float(
            self.get_required_parameter(
                "joint_state_timeout_sec"
            )
        )

        self.command_timeout_sec = float(
            self.get_required_parameter(
                "command_timeout_sec"
            )
        )

        self.angular_speed_zero_threshold = float(
            self.get_required_parameter(
                "angular_speed_zero_threshold"
            )
        )

        self.dry_run = bool(
            self.get_required_parameter(
                "dry_run"
            )
        )

        self.log_period_sec = float(
            self.get_required_parameter(
                "log_period_sec"
            )
        )

        self.validate_parameters()

        # 固定控制周期。
        self.control_period_sec = (
            1.0 /
            self.publish_rate_hz
        )

        # =====================================================
        # 初始化运动学模型
        # =====================================================

        self.kinematics = FrankaKinematics(
            urdf_path=self.urdf_path,
            end_effector_frame=(
                self.end_effector_frame
            )
        )

        (
            self.lower_joint_limits,
            self.upper_joint_limits
        ) = (
            self.kinematics.get_joint_position_limits()
        )

        # =====================================================
        # 当前机器人状态
        # =====================================================

        self.current_q = None

        # 上一个控制周期实际输出的关节速度。
        # 用于进行关节加速度限制。
        self.previous_q_dot = np.zeros(
            7,
            dtype=float
        )

        self.last_joint_state_time = None

        # =====================================================
        # 当前控制命令
        # =====================================================

        self.current_command = (
            FixedAxisCommand.COMMAND_STOP
        )

        self.target_angular_speed = 0.0

        self.current_angular_speed = 0.0

        self.last_command_time = None

        # 收到重新记录固定轴命令后，
        # 先平滑停止，再执行记录。
        self.capture_axis_requested = False

        # =====================================================
        # 固定轴状态
        # =====================================================

        self.axis_is_captured = False

        self.axis_point_world = None

        self.axis_direction_world = None

        self.initial_position_world = None

        self.initial_rotation_world = None

        # 当前期望旋转角度。
        self.rotation_angle = 0.0

        # =====================================================
        # 安全和日志状态
        # =====================================================

        self.safe_stop_reason = None

        self.start_time = (
            self.get_clock().now()
        )

        self.last_log_time = None

        # =====================================================
        # ROS 2 通信
        # =====================================================

        self.joint_state_subscriber = (
            self.create_subscription(
                JointState,
                self.joint_state_topic,
                self.joint_state_callback,
                10
            )
        )

        self.command_subscriber = (
            self.create_subscription(
                FixedAxisCommand,
                self.command_topic,
                self.command_callback,
                10
            )
        )

        self.joint_velocity_publisher = (
            self.create_publisher(
                Float64MultiArray,
                self.joint_velocity_command_topic,
                10
            )
        )

        self.control_timer = self.create_timer(
            self.control_period_sec,
            self.timer_callback
        )

        # =====================================================
        # 启动信息
        # =====================================================

        self.get_logger().info(
            "Fixed-axis rotation node started."
        )

        self.get_logger().info(
            f"URDF: {self.urdf_path}"
        )

        self.get_logger().info(
            "End-effector frame: "
            f"{self.end_effector_frame}"
        )

        self.get_logger().info(
            "Joint state topic: "
            f"{self.joint_state_topic}"
        )

        self.get_logger().info(
            "Command topic: "
            f"{self.command_topic}"
        )

        self.get_logger().info(
            "Joint velocity command topic: "
            f"{self.joint_velocity_command_topic}"
        )

        self.get_logger().info(
            "Axis point in link8: "
            f"{self.axis_point_link8.tolist()}"
        )

        self.get_logger().info(
            "Axis direction in link8: "
            f"{self.axis_direction_link8.tolist()}"
        )

        self.get_logger().info(
            "Control rate: "
            f"{self.publish_rate_hz:.1f} Hz"
        )

        self.get_logger().info(
            "Maximum joint velocity: "
            f"{self.max_joint_velocity:.4f} rad/s"
        )

        self.get_logger().info(
            "Maximum angular speed: "
            f"{self.max_angular_velocity:.4f} rad/s"
        )

        self.get_logger().info(
            f"Dry run: {self.dry_run}"
        )

        self.get_logger().info(
            "Waiting for joint state."
        )

    def get_required_parameter(
        self,
        name
    ):
        """
        读取必须由 YAML 或命令行提供的参数。

        参数没有设置时直接报错。
        """

        parameter = self.get_parameter(
            name
        )

        if (
            parameter.type_
            == Parameter.Type.NOT_SET
        ):
            raise ValueError(
                f"Required parameter '{name}' "
                "is not set."
            )

        return parameter.value

    def validate_parameters(self):
        """
        检查全部参数是否合法。
        """

        if not self.urdf_path:
            raise ValueError(
                "urdf_path must not be empty."
            )

        if not self.end_effector_frame:
            raise ValueError(
                "end_effector_frame must not be empty."
            )

        if not self.joint_state_topic:
            raise ValueError(
                "joint_state_topic must not be empty."
            )

        if not self.command_topic:
            raise ValueError(
                "command_topic must not be empty."
            )

        if not self.joint_velocity_command_topic:
            raise ValueError(
                "joint_velocity_command_topic "
                "must not be empty."
            )

        if self.axis_point_link8.shape != (3,):
            raise ValueError(
                "axis_point_link8 must contain "
                "exactly 3 elements."
            )

        if self.axis_direction_link8.shape != (3,):
            raise ValueError(
                "axis_direction_link8 must contain "
                "exactly 3 elements."
            )

        check_finite_vector(
            self.axis_point_link8,
            "axis_point_link8"
        )

        check_finite_vector(
            self.axis_direction_link8,
            "axis_direction_link8"
        )

        axis_direction_norm = float(
            np.linalg.norm(
                self.axis_direction_link8
            )
        )

        if axis_direction_norm <= 1e-12:
            raise ValueError(
                "axis_direction_link8 must "
                "not be a zero vector."
            )

        # 自动归一化固定轴方向。
        self.axis_direction_link8 = (
            self.axis_direction_link8
            / axis_direction_norm
        )

        positive_parameters = {
            "publish_rate_hz":
                self.publish_rate_hz,

            "max_angular_acceleration":
                self.max_angular_acceleration,

            "max_joint_velocity":
                self.max_joint_velocity,

            "max_joint_acceleration":
                self.max_joint_acceleration,

            "max_linear_velocity":
                self.max_linear_velocity,

            "max_angular_velocity":
                self.max_angular_velocity,

            "min_jacobian_singular_value":
                self.min_jacobian_singular_value,

            "max_jacobian_condition_number":
                self.max_jacobian_condition_number,

            "max_position_error":
                self.max_position_error,

            "max_orientation_error":
                self.max_orientation_error,

            "joint_state_timeout_sec":
                self.joint_state_timeout_sec,

            "command_timeout_sec":
                self.command_timeout_sec,

            "angular_speed_zero_threshold":
                self.angular_speed_zero_threshold,

            "log_period_sec":
                self.log_period_sec,
        }

        for (
            parameter_name,
            parameter_value
        ) in positive_parameters.items():

            if (
                not np.isfinite(parameter_value)
                or parameter_value <= 0.0
            ):
                raise ValueError(
                    f"{parameter_name} must be "
                    "finite and positive."
                )

        if (
            not np.isfinite(self.damping)
            or self.damping < 0.0
        ):
            raise ValueError(
                "damping must be finite "
                "and non-negative."
            )

        if (
            not np.isfinite(self.position_gain)
            or self.position_gain < 0.0
        ):
            raise ValueError(
                "position_gain must be finite "
                "and non-negative."
            )

        if (
            not np.isfinite(self.angular_gain)
            or self.angular_gain < 0.0
        ):
            raise ValueError(
                "angular_gain must be finite "
                "and non-negative."
            )

        if (
            not np.isfinite(self.duration_sec)
            or self.duration_sec < 0.0
        ):
            raise ValueError(
                "duration_sec must be finite "
                "and non-negative."
            )

        if (
            not np.isfinite(
                self.joint_position_margin
            )
            or self.joint_position_margin < 0.0
        ):
            raise ValueError(
                "joint_position_margin must be "
                "finite and non-negative."
            )

        if (
            self.max_jacobian_condition_number
            <= 1.0
        ):
            raise ValueError(
                "max_jacobian_condition_number "
                "must be greater than 1."
            )

    def joint_state_callback(
        self,
        msg
    ):
        """
        接收 Franka 当前七维关节角。
        """

        joint_map = dict(
            zip(
                msg.name,
                msg.position
            )
        )

        try:
            current_q = np.asarray(
                [
                    joint_map["fr3_joint1"],
                    joint_map["fr3_joint2"],
                    joint_map["fr3_joint3"],
                    joint_map["fr3_joint4"],
                    joint_map["fr3_joint5"],
                    joint_map["fr3_joint6"],
                    joint_map["fr3_joint7"],
                ],
                dtype=float
            )

            current_q = check_finite_vector(
                current_q,
                "current_q"
            ).reshape(7)

        except KeyError as error:
            self.current_q = None

            self.enter_safe_stop(
                "Joint state is missing "
                f"a required joint: {error}"
            )
            return

        except ValueError as error:
            self.current_q = None

            self.enter_safe_stop(
                f"Invalid joint state: {error}"
            )
            return

        self.current_q = current_q

        self.last_joint_state_time = (
            self.get_clock().now()
        )

    def command_callback(
        self,
        msg
    ):
        """
        接收固定轴旋转控制命令。
        """

        now = self.get_clock().now()

        command = int(
            msg.command
        )

        requested_speed = float(
            msg.target_angular_speed
        )

        if not np.isfinite(
            requested_speed
        ):
            self.enter_safe_stop(
                "Command angular speed contains "
                "nan or inf."
            )
            return

        valid_commands = {
            FixedAxisCommand.COMMAND_STOP,
            FixedAxisCommand.COMMAND_RUN,
            FixedAxisCommand.COMMAND_EMERGENCY_STOP,
            FixedAxisCommand.COMMAND_CAPTURE_AXIS,
        }

        if command not in valid_commands:
            self.enter_safe_stop(
                "Unknown fixed-axis command: "
                f"{command}"
            )
            return

        self.last_command_time = now

        # =====================================================
        # 正常运行
        # =====================================================

        if command == FixedAxisCommand.COMMAND_RUN:

            requested_speed = float(
                np.clip(
                    requested_speed,
                    -self.max_angular_velocity,
                    self.max_angular_velocity
                )
            )

            if (
                abs(requested_speed)
                <= self.angular_speed_zero_threshold
            ):
                self.current_command = (
                    FixedAxisCommand.COMMAND_STOP
                )

                self.target_angular_speed = 0.0
            else:
                self.current_command = (
                    FixedAxisCommand.COMMAND_RUN
                )

                self.target_angular_speed = (
                    requested_speed
                )

            return

        # =====================================================
        # 正常停止
        # =====================================================

        if command == FixedAxisCommand.COMMAND_STOP:

            self.current_command = (
                FixedAxisCommand.COMMAND_STOP
            )

            self.target_angular_speed = 0.0
            return

        # =====================================================
        # 紧急停止
        # =====================================================

        if (
            command
            == FixedAxisCommand.COMMAND_EMERGENCY_STOP
        ):
            self.current_command = (
                FixedAxisCommand.COMMAND_EMERGENCY_STOP
            )

            self.target_angular_speed = 0.0

            self.current_angular_speed = 0.0

            self.publish_zero_velocity()

            if (
                self.safe_stop_reason
                != "Emergency stop command received."
            ):
                self.get_logger().warning(
                    "Emergency stop command received."
                )

            self.safe_stop_reason = (
                "Emergency stop command received."
            )

            return

        # =====================================================
        # 重新记录固定轴
        # =====================================================

        if (
            command
            == FixedAxisCommand.COMMAND_CAPTURE_AXIS
        ):
            self.current_command = (
                FixedAxisCommand.COMMAND_STOP
            )

            self.target_angular_speed = 0.0

            self.capture_axis_requested = True

            self.get_logger().info(
                "Capture-axis request received. "
                "The controller will stop before "
                "capturing the new axis."
            )

    def timer_callback(self):
        """
        周期计算并发送关节速度。
        """

        now = self.get_clock().now()

        # =====================================================
        # 控制持续时间检查
        # =====================================================

        elapsed_sec = (
            now -
            self.start_time
        ).nanoseconds * 1e-9

        if (
            self.duration_sec > 0.0
            and elapsed_sec >= self.duration_sec
        ):
            self.enter_safe_stop(
                "Control duration reached."
            )

            self.get_logger().info(
                "Control duration reached. "
                "Shutting down."
            )

            rclpy.shutdown()
            return

        # =====================================================
        # 关节状态看门狗
        # =====================================================

        (
            joint_state_is_fresh,
            joint_state_age
        ) = self.message_is_fresh(
            self.last_joint_state_time,
            self.joint_state_timeout_sec,
            now
        )

        if (
            self.current_q is None
            or not joint_state_is_fresh
        ):
            if joint_state_age is None:
                reason = (
                    "Waiting for the first "
                    "joint state message."
                )
            else:
                reason = (
                    "Joint state timeout: "
                    f"{joint_state_age:.3f} s."
                )

            self.enter_safe_stop(
                reason
            )
            return

        # =====================================================
        # 运动学和基础安全检查
        # =====================================================

        try:
            check_joint_position_limits(
                q=self.current_q,
                lower_limits=(
                    self.lower_joint_limits
                ),
                upper_limits=(
                    self.upper_joint_limits
                ),
                margin=(
                    self.joint_position_margin
                )
            )

            (
                p_world_end,
                R_world_end,
                J_world
            ) = (
                self.kinematics.compute_pose_and_jacobian(
                    self.current_q
                )
            )

            (
                minimum_singular_value,
                jacobian_condition_number
            ) = check_jacobian_condition(
                J=J_world,
                min_singular_value=(
                    self.min_jacobian_singular_value
                ),
                max_condition_number=(
                    self.max_jacobian_condition_number
                )
            )

        except Exception as error:
            self.enter_safe_stop(
                "Kinematics safety check failed: "
                f"{error}"
            )
            return

        # =====================================================
        # 第一次自动记录固定轴
        # =====================================================

        if (
            not self.axis_is_captured
            and self.capture_axis_on_start
        ):
            try:
                self.capture_fixed_axis(
                    p_world_end,
                    R_world_end
                )

            except Exception as error:
                self.enter_safe_stop(
                    "Failed to capture initial axis: "
                    f"{error}"
                )
                return

        # =====================================================
        # 命令看门狗
        # =====================================================

        (
            command_is_fresh,
            command_age
        ) = self.message_is_fresh(
            self.last_command_time,
            self.command_timeout_sec,
            now
        )

        if not command_is_fresh:
            if command_age is None:
                reason = (
                    "Waiting for the first "
                    "control command."
                )
            else:
                reason = (
                    "Control command timeout: "
                    f"{command_age:.3f} s."
                )

            self.enter_safe_stop(
                reason
            )
            return

        # =====================================================
        # 紧急停止保持
        # =====================================================

        if (
            self.current_command
            == FixedAxisCommand.COMMAND_EMERGENCY_STOP
        ):
            self.enter_safe_stop(
                "Emergency stop command received."
            )
            return

        # =====================================================
        # 正常角速度变化
        # =====================================================

        try:
            self.current_angular_speed = (
                limit_angular_acceleration(
                    current_angular_speed=(
                        self.current_angular_speed
                    ),
                    target_angular_speed=(
                        self.target_angular_speed
                    ),
                    max_angular_acceleration=(
                        self.max_angular_acceleration
                    ),
                    dt=self.control_period_sec
                )
            )

        except ValueError as error:
            self.enter_safe_stop(
                "Angular speed limiting failed: "
                f"{error}"
            )
            return

        # =====================================================
        # 停止后重新记录固定轴
        # =====================================================

        if self.capture_axis_requested:

            if (
                abs(self.current_angular_speed)
                <= self.angular_speed_zero_threshold
            ):
                try:
                    self.current_angular_speed = 0.0

                    self.capture_fixed_axis(
                        p_world_end,
                        R_world_end
                    )

                    self.capture_axis_requested = False

                except Exception as error:
                    self.enter_safe_stop(
                        "Failed to recapture axis: "
                        f"{error}"
                    )
                    return

            else:
                self.publish_zero_or_ramped_stop(
                    J_world=J_world,
                    p_world_end=p_world_end,
                    R_world_end=R_world_end,
                    minimum_singular_value=(
                        minimum_singular_value
                    ),
                    jacobian_condition_number=(
                        jacobian_condition_number
                    ),
                    now=now
                )
                return

        if not self.axis_is_captured:
            self.enter_safe_stop(
                "Fixed axis has not been captured."
            )
            return

        # =====================================================
        # 更新期望旋转角
        # =====================================================

        self.rotation_angle += (
            self.current_angular_speed
            * self.control_period_sec
        )

        # 将角度限制到 [-pi, pi]，
        # 避免长时间运行后数值不断增大。
        self.rotation_angle = float(
            np.arctan2(
                np.sin(
                    self.rotation_angle
                ),
                np.cos(
                    self.rotation_angle
                )
            )
        )

        # =====================================================
        # 计算期望位姿和末端速度
        # =====================================================

        try:
            (
                p_desired,
                R_desired
            ) = self.compute_desired_pose()

            (
                V_command,
                position_error_norm,
                orientation_error_norm
            ) = self.compute_cartesian_velocity(
                p_world_end=p_world_end,
                R_world_end=R_world_end,
                p_desired=p_desired,
                R_desired=R_desired
            )

            if (
                position_error_norm
                > self.max_position_error
            ):
                raise ValueError(
                    "Position tracking error "
                    f"{position_error_norm:.4f} m "
                    "exceeds limit "
                    f"{self.max_position_error:.4f} m."
                )

            if (
                orientation_error_norm
                > self.max_orientation_error
            ):
                raise ValueError(
                    "Orientation tracking error "
                    f"{orientation_error_norm:.4f} rad "
                    "exceeds limit "
                    f"{self.max_orientation_error:.4f} rad."
                )

            V_command = limit_cartesian_velocity(
                V_command,
                max_linear=(
                    self.max_linear_velocity
                ),
                max_angular=(
                    self.max_angular_velocity
                )
            )

            q_dot = (
                cartesian_velocity_to_joint_velocity(
                    V_e=V_command,
                    J=J_world,
                    damping=self.damping
                )
            )

            q_dot = self.apply_joint_velocity_limits(
                q_dot
            )

        except Exception as error:
            self.enter_safe_stop(
                "Control calculation failed: "
                f"{error}"
            )
            return

        # =====================================================
        # 数据恢复
        # =====================================================

        if self.safe_stop_reason is not None:
            self.get_logger().info(
                "Control inputs are valid again."
            )

            self.safe_stop_reason = None

        # =====================================================
        # 发布和日志
        # =====================================================

        self.publish_joint_velocity(
            q_dot
        )

        self.log_control_status(
            now=now,
            q_dot=q_dot,
            position_error_norm=(
                position_error_norm
            ),
            orientation_error_norm=(
                orientation_error_norm
            ),
            minimum_singular_value=(
                minimum_singular_value
            ),
            jacobian_condition_number=(
                jacobian_condition_number
            )
        )

    def capture_fixed_axis(
        self,
        p_world_end,
        R_world_end
    ):
        """
        根据当前末端位姿记录固定空间轴。

        输入：
            p_world_end:
                当前末端原点在世界坐标系中的位置。

            R_world_end:
                当前末端姿态。
        """

        p_world_end = np.asarray(
            p_world_end,
            dtype=float
        ).reshape(3)

        R_world_end = np.asarray(
            R_world_end,
            dtype=float
        ).reshape(3, 3)

        check_finite_vector(
            p_world_end,
            "p_world_end"
        )

        if not np.all(
            np.isfinite(R_world_end)
        ):
            raise ValueError(
                "R_world_end contains nan or inf."
            )

        axis_point_world = (
            p_world_end
            + R_world_end
            @ self.axis_point_link8
        )

        axis_direction_world = (
            R_world_end
            @ self.axis_direction_link8
        )

        axis_direction_norm = float(
            np.linalg.norm(
                axis_direction_world
            )
        )

        if axis_direction_norm <= 1e-12:
            raise ValueError(
                "Transformed axis direction "
                "is a zero vector."
            )

        axis_direction_world = (
            axis_direction_world
            / axis_direction_norm
        )

        self.axis_point_world = (
            axis_point_world.copy()
        )

        self.axis_direction_world = (
            axis_direction_world.copy()
        )

        self.initial_position_world = (
            p_world_end.copy()
        )

        self.initial_rotation_world = (
            R_world_end.copy()
        )

        self.rotation_angle = 0.0

        self.axis_is_captured = True

        self.get_logger().info(
            "Fixed axis captured."
        )

        self.get_logger().info(
            "Axis point in world: "
            f"{np.round(self.axis_point_world, 6).tolist()}"
        )

        self.get_logger().info(
            "Axis direction in world: "
            f"{np.round(self.axis_direction_world, 6).tolist()}"
        )

        radius_vector = (
            self.initial_position_world
            - self.axis_point_world
        )

        perpendicular_radius = (
            radius_vector
            - np.dot(
                radius_vector,
                self.axis_direction_world
            )
            * self.axis_direction_world
        )

        rotation_radius = float(
            np.linalg.norm(
                perpendicular_radius
            )
        )

        self.get_logger().info(
            "End-effector rotation radius: "
            f"{rotation_radius:.6f} m"
        )

    def compute_desired_pose(self):
        """
        根据当前旋转角度计算期望末端位姿。

        输出：
            p_desired:
                期望末端位置。

            R_desired:
                期望末端姿态。
        """

        if not self.axis_is_captured:
            raise RuntimeError(
                "Fixed axis has not been captured."
            )

        rotation_vector = (
            self.axis_direction_world
            * self.rotation_angle
        )

        R_axis = pin.exp3(
            rotation_vector
        )

        initial_radius_vector = (
            self.initial_position_world
            - self.axis_point_world
        )

        p_desired = (
            self.axis_point_world
            + R_axis
            @ initial_radius_vector
        )

        R_desired = (
            R_axis
            @ self.initial_rotation_world
        )

        return (
            np.asarray(
                p_desired,
                dtype=float
            ).reshape(3),
            np.asarray(
                R_desired,
                dtype=float
            ).reshape(3, 3)
        )

    def compute_cartesian_velocity(
        self,
        p_world_end,
        R_world_end,
        p_desired,
        R_desired
    ):
        """
        计算固定轴旋转末端速度。

        末端速度包括：
            1. 固定轴旋转前馈速度。
            2. 位置误差反馈。
            3. 姿态误差反馈。

        输出速度顺序：
            [
                vx,
                vy,
                vz,
                wx,
                wy,
                wz
            ]
        """

        p_world_end = np.asarray(
            p_world_end,
            dtype=float
        ).reshape(3)

        R_world_end = np.asarray(
            R_world_end,
            dtype=float
        ).reshape(3, 3)

        p_desired = np.asarray(
            p_desired,
            dtype=float
        ).reshape(3)

        R_desired = np.asarray(
            R_desired,
            dtype=float
        ).reshape(3, 3)

        # =====================================================
        # 固定轴旋转前馈速度
        # =====================================================

        angular_velocity_feedforward = (
            self.current_angular_speed
            * self.axis_direction_world
        )

        linear_velocity_feedforward = (
            np.cross(
                angular_velocity_feedforward,
                p_desired
                - self.axis_point_world
            )
        )

        # =====================================================
        # 位置误差反馈
        # =====================================================

        position_error = (
            p_desired
            - p_world_end
        )

        linear_velocity_feedback = (
            self.position_gain
            * position_error
        )

        # =====================================================
        # 姿态误差反馈
        # =====================================================

        # R_desired @ R_current.T 得到在世界方向表达的
        # 当前姿态到期望姿态的旋转误差。
        rotation_error_matrix = (
            R_desired
            @ R_world_end.T
        )

        orientation_error = np.asarray(
            pin.log3(
                rotation_error_matrix
            ),
            dtype=float
        ).reshape(3)

        angular_velocity_feedback = (
            self.angular_gain
            * orientation_error
        )

        # =====================================================
        # 合成末端速度
        # =====================================================

        linear_velocity_command = (
            linear_velocity_feedforward
            + linear_velocity_feedback
        )

        angular_velocity_command = (
            angular_velocity_feedforward
            + angular_velocity_feedback
        )

        V_command = np.concatenate(
            [
                linear_velocity_command,
                angular_velocity_command,
            ]
        )

        V_command = check_finite_vector(
            V_command,
            "V_command"
        ).reshape(6)

        position_error_norm = float(
            np.linalg.norm(
                position_error
            )
        )

        orientation_error_norm = float(
            np.linalg.norm(
                orientation_error
            )
        )

        return (
            V_command,
            position_error_norm,
            orientation_error_norm
        )

    def publish_zero_or_ramped_stop(
        self,
        J_world,
        p_world_end,
        R_world_end,
        minimum_singular_value,
        jacobian_condition_number,
        now
    ):
        """
        等待角速度平滑下降到零时，
        继续计算当前固定轴轨迹并发布速度。
        """

        if not self.axis_is_captured:
            self.publish_zero_velocity()
            return

        self.rotation_angle += (
            self.current_angular_speed
            * self.control_period_sec
        )

        self.rotation_angle = float(
            np.arctan2(
                np.sin(self.rotation_angle),
                np.cos(self.rotation_angle)
            )
        )

        try:
            (
                p_desired,
                R_desired
            ) = self.compute_desired_pose()

            (
                V_command,
                position_error_norm,
                orientation_error_norm
            ) = self.compute_cartesian_velocity(
                p_world_end=p_world_end,
                R_world_end=R_world_end,
                p_desired=p_desired,
                R_desired=R_desired
            )

            V_command = limit_cartesian_velocity(
                V_command,
                max_linear=(
                    self.max_linear_velocity
                ),
                max_angular=(
                    self.max_angular_velocity
                )
            )

            q_dot = (
                cartesian_velocity_to_joint_velocity(
                    V_e=V_command,
                    J=J_world,
                    damping=self.damping
                )
            )

            q_dot = self.apply_joint_velocity_limits(
                q_dot
            )

        except Exception as error:
            self.enter_safe_stop(
                "Failed while stopping before "
                f"axis capture: {error}"
            )
            return

        self.publish_joint_velocity(
            q_dot
        )

        self.log_control_status(
            now=now,
            q_dot=q_dot,
            position_error_norm=(
                position_error_norm
            ),
            orientation_error_norm=(
                orientation_error_norm
            ),
            minimum_singular_value=(
                minimum_singular_value
            ),
            jacobian_condition_number=(
                jacobian_condition_number
            )
        )

    def message_is_fresh(
        self,
        last_message_time,
        timeout_sec,
        now
    ):
        """
        判断消息是否仍在允许的超时时间内。

        输出：
            is_fresh:
                消息是否有效。

            age_sec:
                消息时间间隔。
                从未收到消息时返回 None。
        """

        if last_message_time is None:
            return False, None

        age_sec = (
            now -
            last_message_time
        ).nanoseconds * 1e-9

        age_sec = max(
            0.0,
            float(age_sec)
        )

        return (
            age_sec <= timeout_sec,
            age_sec
        )

    def enter_safe_stop(
        self,
        reason
    ):
        """
        进入立即安全停止状态。

        操作：
            1. 当前角速度清零。
            2. 目标角速度清零。
            3. 当前命令切换为停止。
            4. 持续发布七维零关节速度。
        """

        self.current_angular_speed = 0.0

        self.target_angular_speed = 0.0

        self.previous_q_dot = np.zeros(
            7,
            dtype=float
        )

        if (
            self.current_command
            != FixedAxisCommand.COMMAND_EMERGENCY_STOP
        ):
            self.current_command = (
                FixedAxisCommand.COMMAND_STOP
            )

        if reason != self.safe_stop_reason:
            self.get_logger().warning(
                f"Safety stop: {reason}"
            )

            self.safe_stop_reason = reason

        self.publish_zero_velocity()

    def apply_joint_velocity_limits(
        self,
        q_dot
    ):
        """
        对关节速度执行速度和加速度限制。

        处理顺序：
            1. 检查七维关节速度。
            2. 限制关节速度绝对值。
            3. 限制相邻周期的关节速度变化量。
            4. 保存本周期实际输出速度。
        """

        q_dot = check_finite_vector(
            q_dot,
            "q_dot"
        ).reshape(7)

        q_dot = limit_joint_velocity(
            q_dot,
            max_abs=(
                self.max_joint_velocity
            )
        )

        q_dot = limit_joint_acceleration(
            previous_q_dot=(
                self.previous_q_dot
            ),
            target_q_dot=q_dot,
            max_abs=(
                self.max_joint_acceleration
            ),
            dt=self.control_period_sec
        )

        q_dot = check_finite_vector(
            q_dot,
            "q_dot_safe"
        ).reshape(7)

        self.previous_q_dot = (
            q_dot.copy()
        )

        return q_dot


    def publish_joint_velocity(
        self,
        q_dot
    ):
        """
        发布七维关节速度。

        dry_run=true 时只计算，不向底层控制器发布。
        """

        q_dot = np.asarray(
            q_dot,
            dtype=float
        ).reshape(7)

        check_finite_vector(
            q_dot,
            "q_dot"
        )

        if self.dry_run:
            return

        message = Float64MultiArray()

        message.data = (
            q_dot.tolist()
        )

        self.joint_velocity_publisher.publish(
            message
        )

    def publish_zero_velocity(
        self,
        repeat_count=1
    ):
        """
        发布七维零关节速度。

        输入：
            repeat_count:
                连续发布次数。

        dry_run=true 时不向实际控制器发布。
        """

        if self.dry_run:
            return

        repeat_count = max(
            1,
            int(repeat_count)
        )

        message = Float64MultiArray()

        message.data = [0.0] * 7

        for _ in range(
            repeat_count
        ):
            self.joint_velocity_publisher.publish(
                message
            )

    def log_control_status(
        self,
        now,
        q_dot,
        position_error_norm,
        orientation_error_norm,
        minimum_singular_value,
        jacobian_condition_number
    ):
        """
        按固定时间间隔打印控制状态。
        """

        if self.last_log_time is not None:

            log_age_sec = (
                now -
                self.last_log_time
            ).nanoseconds * 1e-9

            if log_age_sec < self.log_period_sec:
                return

        self.last_log_time = now

        self.get_logger().info(
            "theta="
            f"{self.rotation_angle:+.4f} rad | "
            "omega="
            f"{self.current_angular_speed:+.4f} rad/s | "
            "target="
            f"{self.target_angular_speed:+.4f} rad/s | "
            "position_error="
            f"{position_error_norm:.5f} m | "
            "orientation_error="
            f"{orientation_error_norm:.5f} rad | "
            "sigma_min="
            f"{minimum_singular_value:.5f} | "
            "condition="
            f"{jacobian_condition_number:.2f} | "
            "q_dot="
            f"{np.round(q_dot, 5).tolist()} | "
            "dry_run="
            f"{self.dry_run}"
        )


def main(args=None):
    """
    ROS 2 节点入口。
    """

    rclpy.init(
        args=args
    )

    node = None

    try:
        node = FixedAxisRotationNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Keyboard interrupt. "
                "Sending zero joint velocity."
            )

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Unexpected error: {error}"
            )

        raise

    finally:
        if node is not None:

            if rclpy.ok():
                node.publish_zero_velocity(
                    repeat_count=5
                )

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()