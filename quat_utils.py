"""
All quaternion operations in this py file follow Hamilton standard:
    1. q: (w, x, y, z)
    2. pq = p * q means first rotate q and then rotate p
"""
import math
import numpy as np


def angle_to_quat(angle):
    """
    Transform angle to quaternion.

    :param angle: angle to be transformed to quaternion [rad/s]
    :return: Hamilton quaternion
    """
    norm = np.linalg.norm(angle)
    if norm < 1e-6:
        quat = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        sin_theta = math.sin(0.5 * norm) / norm
        cos_theta = math.cos(0.5 * norm)
        angle *= sin_theta
        quat = np.array([cos_theta, angle[0], angle[1], angle[2]])
    return quat


def quat_multiply(p, q):
    """
    First rotate by q and then rotate by p.

    :param p: Second rotated quaternion
    :param q: First rotated quaternion
    :return: pq = p * q
    """

    pq = np.array([
        p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],  # w
        p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],  # x
        p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],  # y
        p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]   # z
    ])

    if pq[0] < 0:
        pq *= -1.0
    return pq


def quat_norm(q):
    """
    Force qw always be positive.

    :param q: quaternion to normalize
    :return: a normalized quaternion
    """
    if q[0] < 0:
        q *= -1
    return q / np.linalg.norm(q)


def skew(x):
    """
    Skew-symmetric matrix from a 3x1 array.

    :param x: 3x1 array to be skew-symmetric
    :return: a skew-symmetric matrix
    """
    return np.array([[0.0, -x[2], x[1]],
                     [x[2], 0.0, -x[0]],
                     [-x[1], x[0], 0.0]])


def omega(w):
    """
    Omega operator used for product between the angular velocity and the quaternion.

    https://ahrs.readthedocs.io/en/latest/filters/angular.html

    :param w: angular velocity [rad/s]
    :return: Omega operator for Hamilton product
    """
    mat = np.zeros((4, 4))
    mat[1:, 1:] = -skew(w)
    mat[1:, 0] = w
    mat[0, 1:] = -w
    return mat
