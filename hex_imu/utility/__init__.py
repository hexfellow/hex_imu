#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os

ROS_VERSION = os.environ.get("ROS_VERSION")

if ROS_VERSION == "1":
    from .ros1_interface import DataInterface as DataInterface
elif ROS_VERSION == "2":
    from .ros2_interface import DataInterface as DataInterface
else:
    raise ValueError("ROS_VERSION is not set")

from .imu_util import ImuUtil as ImuUtil

__all__ = [
    DataInterface,
    ImuUtil,
]
