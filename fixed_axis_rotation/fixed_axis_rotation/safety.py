#!/usr/bin/env python3
"""
safety.py

功能：
    提供 Franka 控制中可复用的安全处理函数。

    主要用于：
    1. 检查向量中是否存在 nan 或 inf。
    2. 对关节速度进行限幅。
    3. 对末端线速度和角速度进行限幅。
    4. 检查当前关节位置是否接近关节限位。
    5. 检查雅可比矩阵是否接近奇异。
    6. 对目标角速度变化率进行限制。

接口：
    check_finite_vector(vector, name)

    limit_joint_velocity(
        q_dot,
        max_abs
    )

    limit_cartesian_velocity(
        V_e,
        max_linear,
        max_angular
    )

    check_joint_position_limits(
        q,
        lower_limits,
        upper_limits,
        margin
    )

    check_jacobian_condition(
        J,
        min_singular_value,
        max_condition_number
    )

    limit_angular_acceleration(
        current_angular_speed,
        target_angular_speed,
        max_angular_acceleration,
        dt
    )

输入：
    q_dot:
        7维关节速度：

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

    V_e:
        6维末端速度：

        [
            vx,
            vy,
            vz,
            wx,
            wy,
            wz
        ]

        线速度单位为 m/s。
        角速度单位为 rad/s。

    q:
        7维当前关节位置，单位为 rad。

    lower_limits:
        7维关节位置下限，单位为 rad。

    upper_limits:
        7维关节位置上限，单位为 rad。

    margin:
        距离关节位置上下限的安全余量，单位为 rad。

    J:
        6x7 末端雅可比矩阵。

    min_singular_value:
        允许的最小奇异值。

    max_condition_number:
        允许的最大条件数。

    current_angular_speed:
        当前控制角速度，单位为 rad/s。

    target_angular_speed:
        目标控制角速度，单位为 rad/s。

    max_angular_acceleration:
        最大角加速度，单位为 rad/s^2。

    dt:
        控制周期，单位为 s。

输出：
    limit_joint_velocity:
        返回限幅后的 q_dot_safe。

    limit_cartesian_velocity:
        返回限幅后的 V_e_safe。

    check_joint_position_limits:
        检查通过时返回当前关节角。

        接近或超过关节限位时抛出 ValueError。

    check_jacobian_condition:
        返回：

        min_value:
            雅可比矩阵最小奇异值。

        condition_number:
            雅可比矩阵条件数。

        接近奇异时抛出 ValueError。

    limit_angular_acceleration:
        返回经过角加速度限制后的下一周期角速度。
"""

import numpy as np


def check_finite_vector(
    vector,
    name
):
    """
    检查向量中是否存在 nan 或 inf。
    """

    vector = np.asarray(
        vector,
        dtype=float
    )

    if not np.all(
        np.isfinite(vector)
    ):
        raise ValueError(
            f"{name} contains nan or inf."
        )

    return vector


def limit_joint_velocity(
    q_dot,
    max_abs
):
    """
    对 7 维关节速度进行逐关节限幅。
    """

    q_dot = np.asarray(
        q_dot,
        dtype=float
    ).reshape(-1)

    if q_dot.shape != (7,):
        raise ValueError(
            "q_dot must contain exactly "
            f"7 elements, but got {q_dot.size}."
        )

    check_finite_vector(
        q_dot,
        "q_dot"
    )

    max_abs = float(
        max_abs
    )

    if (
        not np.isfinite(max_abs)
        or max_abs <= 0.0
    ):
        raise ValueError(
            "max_abs must be finite "
            "and positive."
        )

    q_dot_safe = np.clip(
        q_dot,
        -max_abs,
        max_abs
    )

    return q_dot_safe


def limit_cartesian_velocity(
    V_e,
    max_linear,
    max_angular
):
    """
    对 6 维末端速度进行限幅。

    线速度和角速度分别按照向量模长整体缩放，
    从而尽量保持原始运动方向不变。
    """

    V_e = np.asarray(
        V_e,
        dtype=float
    ).reshape(-1)

    if V_e.shape != (6,):
        raise ValueError(
            "V_e must contain exactly "
            f"6 elements, but got {V_e.size}."
        )

    check_finite_vector(
        V_e,
        "V_e"
    )

    max_linear = float(
        max_linear
    )

    max_angular = float(
        max_angular
    )

    if (
        not np.isfinite(max_linear)
        or max_linear <= 0.0
    ):
        raise ValueError(
            "max_linear must be finite "
            "and positive."
        )

    if (
        not np.isfinite(max_angular)
        or max_angular <= 0.0
    ):
        raise ValueError(
            "max_angular must be finite "
            "and positive."
        )

    V_e_safe = V_e.copy()

    linear_velocity = (
        V_e_safe[0:3]
    )

    angular_velocity = (
        V_e_safe[3:6]
    )

    linear_speed = float(
        np.linalg.norm(
            linear_velocity
        )
    )

    angular_speed = float(
        np.linalg.norm(
            angular_velocity
        )
    )

    # 保持线速度方向不变，
    # 只限制线速度向量的模长。
    if linear_speed > max_linear:
        V_e_safe[0:3] = (
            linear_velocity
            * max_linear
            / linear_speed
        )

    # 保持角速度方向不变，
    # 只限制角速度向量的模长。
    if angular_speed > max_angular:
        V_e_safe[3:6] = (
            angular_velocity
            * max_angular
            / angular_speed
        )

    return V_e_safe


def check_joint_position_limits(
    q,
    lower_limits,
    upper_limits,
    margin
):
    """
    检查七个关节是否接近位置限位。

    当任意关节进入安全余量范围时，
    抛出 ValueError，由主控制节点进入安全停止。

    输入：
        q:
            当前七维关节位置。

        lower_limits:
            七维关节位置下限。

        upper_limits:
            七维关节位置上限。

        margin:
            关节位置安全余量，单位为 rad。
    """

    q = np.asarray(
        q,
        dtype=float
    ).reshape(-1)

    lower_limits = np.asarray(
        lower_limits,
        dtype=float
    ).reshape(-1)

    upper_limits = np.asarray(
        upper_limits,
        dtype=float
    ).reshape(-1)

    if q.shape != (7,):
        raise ValueError(
            "q must contain exactly "
            f"7 elements, but got {q.size}."
        )

    if lower_limits.shape != (7,):
        raise ValueError(
            "lower_limits must contain "
            "exactly 7 elements."
        )

    if upper_limits.shape != (7,):
        raise ValueError(
            "upper_limits must contain "
            "exactly 7 elements."
        )

    check_finite_vector(
        q,
        "q"
    )

    check_finite_vector(
        lower_limits,
        "lower_limits"
    )

    check_finite_vector(
        upper_limits,
        "upper_limits"
    )

    margin = float(
        margin
    )

    if (
        not np.isfinite(margin)
        or margin < 0.0
    ):
        raise ValueError(
            "margin must be finite "
            "and non-negative."
        )

    if np.any(
        lower_limits >= upper_limits
    ):
        raise ValueError(
            "Each lower joint limit must "
            "be smaller than its upper limit."
        )

    safe_lower_limits = (
        lower_limits + margin
    )

    safe_upper_limits = (
        upper_limits - margin
    )

    if np.any(
        safe_lower_limits
        >= safe_upper_limits
    ):
        raise ValueError(
            "Joint position margin is too large."
        )

    unsafe_lower = (
        q <= safe_lower_limits
    )

    unsafe_upper = (
        q >= safe_upper_limits
    )

    unsafe_joint_indices = np.where(
        unsafe_lower | unsafe_upper
    )[0]

    if unsafe_joint_indices.size > 0:

        joint_descriptions = []

        for joint_index in unsafe_joint_indices:

            joint_number = (
                int(joint_index) + 1
            )

            if unsafe_lower[joint_index]:
                side = "lower"
            else:
                side = "upper"

            joint_descriptions.append(
                "joint"
                f"{joint_number} "
                f"near {side} limit"
            )

        raise ValueError(
            "Joint position safety limit "
            "reached: "
            + ", ".join(
                joint_descriptions
            )
        )

    return q.copy()


def check_jacobian_condition(
    J,
    min_singular_value,
    max_condition_number
):
    """
    检查雅可比矩阵是否接近奇异。

    使用奇异值分解获得：
    1. 最小奇异值。
    2. 最大奇异值与最小奇异值之比。

    任意指标超过限制时抛出 ValueError。
    """

    J = np.asarray(
        J,
        dtype=float
    )

    if J.shape != (6, 7):
        raise ValueError(
            "J must have shape (6, 7), "
            f"but got {J.shape}."
        )

    if not np.all(
        np.isfinite(J)
    ):
        raise ValueError(
            "J contains nan or inf."
        )

    min_singular_value = float(
        min_singular_value
    )

    max_condition_number = float(
        max_condition_number
    )

    if (
        not np.isfinite(
            min_singular_value
        )
        or min_singular_value <= 0.0
    ):
        raise ValueError(
            "min_singular_value must be "
            "finite and positive."
        )

    if (
        not np.isfinite(
            max_condition_number
        )
        or max_condition_number <= 1.0
    ):
        raise ValueError(
            "max_condition_number must be "
            "finite and greater than 1."
        )

    singular_values = np.linalg.svd(
        J,
        compute_uv=False
    )

    if singular_values.shape != (6,):
        raise ValueError(
            "Jacobian SVD must return "
            "6 singular values."
        )

    maximum_value = float(
        singular_values[0]
    )

    minimum_value = float(
        singular_values[-1]
    )

    if minimum_value <= 0.0:
        condition_number = float(
            "inf"
        )
    else:
        condition_number = (
            maximum_value
            / minimum_value
        )

    if minimum_value < min_singular_value:
        raise ValueError(
            "Jacobian is near singular: "
            "minimum singular value "
            f"{minimum_value:.6f} is below "
            f"{min_singular_value:.6f}."
        )

    if condition_number > max_condition_number:
        raise ValueError(
            "Jacobian is ill-conditioned: "
            "condition number "
            f"{condition_number:.3f} exceeds "
            f"{max_condition_number:.3f}."
        )

    return (
        minimum_value,
        condition_number
    )


def limit_angular_acceleration(
    current_angular_speed,
    target_angular_speed,
    max_angular_acceleration,
    dt
):
    """
    限制角速度在一个控制周期内的变化量。

    正常启动、停止和反向时，
    当前角速度按照最大角加速度逐渐逼近目标角速度。

    紧急停止不使用本函数，
    主控制节点应直接将角速度和关节速度清零。
    """

    current_angular_speed = float(
        current_angular_speed
    )

    target_angular_speed = float(
        target_angular_speed
    )

    max_angular_acceleration = float(
        max_angular_acceleration
    )

    dt = float(
        dt
    )

    values = np.asarray(
        [
            current_angular_speed,
            target_angular_speed,
            max_angular_acceleration,
            dt,
        ],
        dtype=float
    )

    if not np.all(
        np.isfinite(values)
    ):
        raise ValueError(
            "Angular acceleration inputs "
            "contain nan or inf."
        )

    if max_angular_acceleration <= 0.0:
        raise ValueError(
            "max_angular_acceleration "
            "must be positive."
        )

    if dt <= 0.0:
        raise ValueError(
            "dt must be positive."
        )

    maximum_change = (
        max_angular_acceleration
        * dt
    )

    requested_change = (
        target_angular_speed
        - current_angular_speed
    )

    limited_change = float(
        np.clip(
            requested_change,
            -maximum_change,
            maximum_change
        )
    )

    next_angular_speed = (
        current_angular_speed
        + limited_change
    )

    # 防止浮点误差导致速度在目标值附近反复跳动。
    if np.isclose(
        next_angular_speed,
        target_angular_speed,
        atol=1e-12,
        rtol=0.0
    ):
        next_angular_speed = (
            target_angular_speed
        )

    return float(
        next_angular_speed
    )