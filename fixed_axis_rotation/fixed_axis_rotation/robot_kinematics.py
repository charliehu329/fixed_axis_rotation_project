#!/usr/bin/env python3
"""
robot_kinematics.py

功能：
    根据当前关节角 q，计算 Franka FR3 的末端位姿和雅可比矩阵。

接口：
    FrankaKinematics(
        urdf_path,
        end_effector_frame="fr3_link8"
    )

    compute_pose_and_jacobian(q)
    compute_jacobian(q)
    get_joint_position_limits()

输入：
    urdf_path:
        FR3 的 URDF 文件路径。

    end_effector_frame:
        需要计算运动学的末端坐标系名称。
        默认使用 "fr3_link8"。

    q:
        7维关节角：

        [
            q1,
            q2,
            q3,
            q4,
            q5,
            q6,
            q7
        ]

        单位为 rad。
        shape = (7,)

输出：
    p_world_end:
        末端坐标系原点在世界坐标系中的位置。

        [
            x,
            y,
            z
        ]

        单位为 m。
        shape = (3,)

    R_world_end:
        末端坐标系相对于世界坐标系的旋转矩阵。

        shape = (3, 3)

    J_world:
        世界坐标系方向表达的末端雅可比矩阵。

        速度顺序为：

        [
            vx,
            vy,
            vz,
            wx,
            wy,
            wz
        ]

        shape = (6, 7)

方法：
    使用 Pinocchio 读取 URDF，并通过正运动学计算末端位姿。

    雅可比矩阵使用：

        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

    该参考系的原点位于末端坐标系原点，
    但线速度和角速度方向使用世界坐标系方向表达。

说明：
    本文件只负责运动学计算，不负责 ROS 通信、
    速度控制、限幅和安全停止。
"""

import os

import numpy as np
import pinocchio as pin


class FrankaKinematics:
    """
    Franka FR3 运动学计算类。
    """

    def __init__(
        self,
        urdf_path,
        end_effector_frame="fr3_link8"
    ):
        """
        初始化 Franka FR3 运动学模型。
        """

        self.urdf_path = str(
            urdf_path
        )

        self.end_effector_frame = str(
            end_effector_frame
        )

        if not os.path.isfile(
            self.urdf_path
        ):
            raise FileNotFoundError(
                "URDF file does not exist: "
                f"{self.urdf_path}"
            )

        # =====================================================
        # 读取 URDF 模型
        # =====================================================

        self.model = pin.buildModelFromUrdf(
            self.urdf_path
        )

        self.data = (
            self.model.createData()
        )

        if not self.model.existFrame(
            self.end_effector_frame
        ):
            raise ValueError(
                "End-effector frame does not exist "
                "in the URDF: "
                f"'{self.end_effector_frame}'."
            )

        self.frame_id = (
            self.model.getFrameId(
                self.end_effector_frame
            )
        )

        # =====================================================
        # Franka 七个机械臂关节
        # =====================================================

        self.arm_joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self.arm_q_indices = []
        self.arm_v_indices = []

        for joint_name in self.arm_joint_names:

            if not self.model.existJointName(
                joint_name
            ):
                raise ValueError(
                    "Joint does not exist in "
                    f"the URDF: '{joint_name}'."
                )

            joint_id = (
                self.model.getJointId(
                    joint_name
                )
            )

            joint_model = (
                self.model.joints[
                    joint_id
                ]
            )

            if (
                joint_model.nq != 1
                or joint_model.nv != 1
            ):
                raise ValueError(
                    f"Joint '{joint_name}' must "
                    "have nq=1 and nv=1."
                )

            self.arm_q_indices.append(
                joint_model.idx_q
            )

            self.arm_v_indices.append(
                joint_model.idx_v
            )

    def build_full_configuration(
        self,
        q
    ):
        """
        将七维机械臂关节角写入完整模型配置。

        输入：
            q:
                7维机械臂关节角。

        输出：
            q_full:
                Pinocchio 完整模型配置。
        """

        q = np.asarray(
            q,
            dtype=float
        ).reshape(-1)

        if q.shape != (7,):
            raise ValueError(
                "q must contain exactly "
                f"7 elements, but got {q.size}."
            )

        if not np.all(
            np.isfinite(q)
        ):
            raise ValueError(
                "q contains nan or inf."
            )

        q_full = pin.neutral(
            self.model
        )

        for arm_index, model_index in enumerate(
            self.arm_q_indices
        ):
            q_full[model_index] = (
                q[arm_index]
            )

        return q_full

    def compute_pose_and_jacobian(
        self,
        q
    ):
        """
        计算末端位姿和世界方向表达的雅可比矩阵。

        输入：
            q:
                7维机械臂关节角。

        输出：
            p_world_end:
                末端原点在世界坐标系中的位置。
                shape = (3,)

            R_world_end:
                末端坐标系相对于世界坐标系的旋转矩阵。
                shape = (3, 3)

            J_world:
                世界坐标系方向表达的末端雅可比矩阵。
                shape = (6, 7)
        """

        q_full = (
            self.build_full_configuration(
                q
            )
        )

        # =====================================================
        # 正运动学
        # =====================================================

        pin.forwardKinematics(
            self.model,
            self.data,
            q_full
        )

        pin.updateFramePlacements(
            self.model,
            self.data
        )

        frame_placement = (
            self.data.oMf[
                self.frame_id
            ]
        )

        p_world_end = np.asarray(
            frame_placement.translation,
            dtype=float
        ).reshape(3).copy()

        R_world_end = np.asarray(
            frame_placement.rotation,
            dtype=float
        ).reshape(3, 3).copy()

        # =====================================================
        # 世界方向表达的末端雅可比
        # =====================================================

        J_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        J_world = np.asarray(
            J_full[
                :,
                self.arm_v_indices
            ],
            dtype=float
        ).copy()

        # =====================================================
        # 输出检查
        # =====================================================

        if p_world_end.shape != (3,):
            raise ValueError(
                "End-effector position must "
                "have shape (3,)."
            )

        if R_world_end.shape != (3, 3):
            raise ValueError(
                "End-effector rotation matrix "
                "must have shape (3, 3)."
            )

        if J_world.shape != (6, 7):
            raise ValueError(
                "Jacobian must have shape "
                f"(6, 7), but got {J_world.shape}."
            )

        if not np.all(
            np.isfinite(p_world_end)
        ):
            raise ValueError(
                "End-effector position contains "
                "nan or inf."
            )

        if not np.all(
            np.isfinite(R_world_end)
        ):
            raise ValueError(
                "End-effector rotation matrix "
                "contains nan or inf."
            )

        if not np.all(
            np.isfinite(J_world)
        ):
            raise ValueError(
                "Jacobian contains nan or inf."
            )

        return (
            p_world_end,
            R_world_end,
            J_world
        )

    def compute_jacobian(
        self,
        q
    ):
        """
        只返回世界方向表达的末端雅可比矩阵。

        输入：
            q:
                7维机械臂关节角。

        输出：
            J_world:
                shape = (6, 7)
        """

        (
            _,
            _,
            J_world
        ) = self.compute_pose_and_jacobian(
            q
        )

        return J_world

    def get_joint_position_limits(
        self
    ):
        """
        获取七个机械臂关节的位置限制。

        输出：
            lower_limits:
                七个关节的位置下限。
                单位为 rad。

            upper_limits:
                七个关节的位置上限。
                单位为 rad。
        """

        lower_limits = np.asarray(
            [
                self.model.lowerPositionLimit[
                    index
                ]
                for index in self.arm_q_indices
            ],
            dtype=float
        )

        upper_limits = np.asarray(
            [
                self.model.upperPositionLimit[
                    index
                ]
                for index in self.arm_q_indices
            ],
            dtype=float
        )

        if (
            lower_limits.shape != (7,)
            or upper_limits.shape != (7,)
        ):
            raise ValueError(
                "Joint position limits must "
                "contain 7 elements."
            )

        if (
            not np.all(
                np.isfinite(lower_limits)
            )
            or not np.all(
                np.isfinite(upper_limits)
            )
        ):
            raise ValueError(
                "Joint position limits contain "
                "nan or inf."
            )

        return (
            lower_limits.copy(),
            upper_limits.copy()
        )