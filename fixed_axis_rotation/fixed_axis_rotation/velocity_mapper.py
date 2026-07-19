#!/usr/bin/env python3
"""
velocity_mapper.py

功能：
    将末端速度 V_e 转换为关节速度 q_dot。

接口：
    cartesian_velocity_to_joint_velocity(V_e, J, damping=0.01)

输入：
    V_e: 6维末端速度 [vx, vy, vz, wx, wy, wz]
         shape = (6,)
    J:   当前雅可比矩阵
         shape = (6, 7)
    damping : float
        求最小二乘伪逆时的阻尼因子，默认为 0.01

    

输出：
    q_dot: 7维关节速度 [dq1, dq2, dq3, dq4, dq5, dq6, dq7]
           shape = (7,)

方法：
    使用阻尼最小二乘伪逆：
    q_dot = J.T @ inv(J @ J.T + damping^2 * I) @ V_e
"""

import numpy as np


def cartesian_velocity_to_joint_velocity(V_e, J, damping=0.01):
    """
    Convert Cartesian end-effector velocity to joint velocity.

    Parameters
    ----------
    V_e : array-like, shape (6,)
        End-effector velocity [vx, vy, vz, wx, wy, wz].

    J : array-like, shape (6, 7)
        Robot Jacobian matrix.

    damping : float
        Damping factor for damped least-squares inverse.

    Returns
    -------
    q_dot : np.ndarray, shape (7,)
        Joint velocity command.
    """

    V_e = np.asarray(V_e, dtype=float).reshape(6)
    J = np.asarray(J, dtype=float)

    if J.shape != (6, 7):
        raise ValueError(f"Expected J shape (6, 7), but got {J.shape}")

    if damping < 0:
        raise ValueError("damping must be non-negative")

    identity_6 = np.eye(6)

    # Damped pseudo-inverse: J^T (J J^T + λ² I)^-1
    J_damped_pinv = J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * identity_6)

    q_dot = J_damped_pinv @ V_e

    return q_dot