#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import typing
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from .interface_base import InterfaceBase
from hex_utils import HexStamp
from hex_utils import HexSensorImuQuat, HexSensorImuQuatStamped
from hex_utils import HexSensorMag, HexSensorMagStamped

class DataInterface(InterfaceBase):
    def __init__(self, name: str = "unknown"):
        super().__init__()
        ### ros node
        rclpy.init()
        self._node = Node(name)
        
        ### parameters
        self._node.declare_parameter('node_id', 0x10)
        self._node.declare_parameter('channel', 'can0')
        
        node_id = self._node.get_parameter('node_id').value
        channel = self._node.get_parameter('channel').value
        
        # canopen
        self._can_param = {
            "node_id": node_id,
            "bustype": 'socketcan',
            "channel": channel,
        }
        
        ### publisher
        self.__imu_pub = self._node.create_publisher(
            Imu,
            "imu_data",
            10
        )

        self.__magnetic_pub = self._node.create_publisher(
            MagneticField,
            "magnetic_data",
            10
        )
        
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._node:
            self._node.destroy_node()
        rclpy.shutdown()

    def sleep(self):
        try:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        except KeyboardInterrupt:
            raise
    
    def spin(self):
        rclpy.spin(self._node)
    
    def logd(self, msg, *args, **kwargs):
        self._node.get_logger().debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self._node.get_logger().info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self._node.get_logger().warn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self._node.get_logger().error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self._node.get_logger().fatal(msg, *args, **kwargs)

    def pub_imu(self, out: HexSensorImuQuatStamped):
        sec, nsec = out.stamp().get_time()
        msg = Imu()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = "imu"
        msg.orientation.x = out.imu_quat().quat()[0]
        msg.orientation.y = out.imu_quat().quat()[1]
        msg.orientation.z = out.imu_quat().quat()[2]
        msg.orientation.w = out.imu_quat().quat()[3]
        msg.angular_velocity.x = out.imu_quat().gyro()[0]
        msg.angular_velocity.y = out.imu_quat().gyro()[1]
        msg.angular_velocity.z = out.imu_quat().gyro()[2]
        msg.linear_acceleration.x = out.imu_quat().acc()[0]
        msg.linear_acceleration.y = out.imu_quat().acc()[1]
        msg.linear_acceleration.z = out.imu_quat().acc()[2]
        # publish
        self.__imu_pub.publish(msg)
        
    def pub_magnetic(self, out: HexSensorMagStamped):
        sec, nsec = out.stamp().get_time()
        msg = MagneticField()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = "imu"
        msg.magnetic_field.x = out.magnetic_field().magnetic_field()[0]
        msg.magnetic_field.y = out.magnetic_field().magnetic_field()[1]
        msg.magnetic_field.z = out.magnetic_field().magnetic_field()[2]
        # publish
        self.__magnetic_pub.publish(msg)
