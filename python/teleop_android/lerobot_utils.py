import math

import numpy as np
import transforms3d as t3d

#:

TF_RUB2FLU = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
TF_XYZW_TO_WXYZ = np.array([[0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
TF_WXYZ_TO_XYZW = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0]])

#:


def are_close(a, b=None, lin_tol=1e-9, ang_tol=1e-9):
    """
    Check if two transformation matrices are close to each other within specified tolerances.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py

    Parameters:
        a (numpy.ndarray): The first transformation matrix.
        b (numpy.ndarray, optional): The second transformation matrix. If not provided, it defaults to the identity matrix.
        lin_tol (float, optional): The linear tolerance for closeness. Defaults to 1e-9.
        ang_tol (float, optional): The angular tolerance for closeness. Defaults to 1e-9.

    Returns:
        bool: True if the matrices are close, False otherwise.
    """
    if b is None:
        b = np.eye(4)
    d = np.linalg.inv(a) @ b
    if not np.allclose(d[:3, 3], np.zeros(3), atol=lin_tol):
        return False
    yaw = math.atan2(d[1, 0], d[0, 0])
    pitch = math.asin(-d[2, 0])
    roll = math.atan2(d[2, 1], d[2, 2])
    rpy = np.array([roll, pitch, yaw])
    return np.allclose(rpy, np.zeros(3), atol=ang_tol)


def slerp(q1, q2, t):
    """
    Spherical linear interpolation between two quaternions.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py
    """
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot = np.dot(q1, q2)

    # If the dot product is negative, use the shortest path
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Linear interpolation fallback for nearly identical quaternions
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    theta = theta_0 * t

    q3 = q2 - q1 * dot
    q3 = q3 / np.linalg.norm(q3)

    return q1 * np.cos(theta) + q3 * np.sin(theta)


def interpolate_transforms(T1, T2, alpha):
    """
    Interpolate between two 4x4 transformation matrices using SLERP + linear translation.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py

    Args:
        T1 (np.ndarray): Start transform (4x4)
        T2 (np.ndarray): End transform (4x4)
        alpha (float): Interpolation factor [0, 1]

    Returns:
        np.ndarray: Interpolated transform (4x4)
    """
    assert T1.shape == (4, 4) and T2.shape == (4, 4)
    assert 0.0 <= alpha <= 1.0

    # Translation
    t1 = T1[:3, 3]
    t2 = T2[:3, 3]
    t_interp = (1 - alpha) * t1 + alpha * t2

    # Rotation
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]
    q1 = t3d.quaternions.mat2quat(R1)
    q2 = t3d.quaternions.mat2quat(R2)

    # SLERP
    q_interp = slerp(q1, q2, alpha)
    R_interp = t3d.quaternions.quat2mat(q_interp)

    # Final transform
    T_interp = np.eye(4)
    T_interp[:3, :3] = R_interp
    T_interp[:3, 3] = t_interp

    return T_interp


def extract_pitch_roll_from_matrix(R):
    """
    Extract pitch and roll angles from a 3x3 rotation matrix using ZYX convention.

    This method avoids Euler angle conversion that can cause gimbal lock.
    Uses the same convention as are_close: ZYX (yaw around Z, pitch around Y, roll around X).

    Args:
        R (np.ndarray): 3x3 rotation matrix

    Returns:
        tuple: (pitch_rad, roll_rad) in radians
    """
    # ZYX decomposition (matches are_close convention)
    # pitch = rotation around Y axis
    pitch_rad = math.asin(-np.clip(R[2, 0], -1.0, 1.0))
    # roll = rotation around X axis
    roll_rad = math.atan2(R[2, 1], R[2, 2])

    return pitch_rad, roll_rad
