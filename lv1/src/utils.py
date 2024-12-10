import numpy as np
from typing import Union
import pandas as pd

from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R


def deg2rad(deg: Union[np.ndarray, float]):
    """
    :brief:         omit
    :param deg:     degree
    :return:        radian
    """
    return deg * np.pi / 180.0


def rad2deg(rad: Union[np.ndarray, float]):
    """
    :brief:         omit
    :param rad:     radian
    :return:        degree
    """
    return rad * 180.0 / np.pi

def euler2quaternion(euler):
    r = R.from_euler('xyz', euler, degrees=False)
    quaternion = r.as_quat()
    return quaternion

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=False)
    return euler


def C(x: Union[np.ndarray, float, list]):
    return np.cos(x)


def S(x: Union[np.ndarray, float, list]):
    return np.sin(x)

def euler_2_quaternion(phi, theta, psi):
    w = C(phi / 2) * C(theta / 2) * C(psi / 2) + S(phi / 2) * S(theta / 2) * S(psi / 2)
    x = S(phi / 2) * C(theta / 2) * C(psi / 2) - C(phi / 2) * S(theta / 2) * S(psi / 2)
    y = C(phi / 2) * S(theta / 2) * C(psi / 2) + S(phi / 2) * C(theta / 2) * S(psi / 2)
    z = C(phi / 2) * C(theta / 2) * S(psi / 2) - S(phi / 2) * S(theta / 2) * C(psi / 2)
    return [x, y, z, w]

def uav_odom_2_uav_state(odom: Odometry) -> np.ndarray:
    _orientation = odom.pose.pose.orientation
    _w = _orientation.w
    _x = _orientation.x
    _y = _orientation.y
    _z = _orientation.z
    rpy = quaternion2euler([_x, _y, _z, _w])
    _uav_state = np.array([
        odom.pose.pose.position.x,  # x
        odom.pose.pose.position.y,  # y
        odom.pose.pose.position.z,  # z
        odom.twist.twist.linear.x,  # vx
        odom.twist.twist.linear.y,  # vy
        odom.twist.twist.linear.z,  # vz
        rpy[0],  # phi
        rpy[1],  # theta
        rpy[2],  # psi
        odom.twist.twist.angular.x,  # p
        odom.twist.twist.angular.y,  # q
        odom.twist.twist.angular.z  # r
    ])
    return _uav_state

