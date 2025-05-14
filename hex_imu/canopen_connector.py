#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import can
import threading
import time
import sys
import struct
from datetime import datetime
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def log(msg):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {msg}")

class CanopenConnector:
    def __init__(self, node_id, bustype, channel,
                 imu_topic, mag_topic, frame_id):

        print(node_id, bustype, channel, imu_topic, mag_topic, frame_id)
        self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=10)
        self.vector3_pub = rospy.Publisher(mag_topic, Vector3, queue_size=10)
        self.frame_id = frame_id

        # CAN参数
        self.node_id = node_id
        self.bustype = bustype
        self.channel = channel
        self.sdo_client_cobid = 0x600 + node_id
        self.sdo_server_cobid = 0x580 + node_id
        self.heartbeat_cobid = 0x700 + node_id
        self.pdo_cobids = {
            1: 0x180 + node_id,
            2: 0x280 + node_id,
            3: 0x380 + node_id,
            4: 0x480 + node_id
        }

        # 状态变量
        self.map_range = None
        self.heartbeat_status = None
        self.last_heartbeat_time = 0
        self.running = False
        self.lock = threading.Lock()
        self.heartbeat_thread = None
        self.data_thread = None

        # CAN总线初始化
        self.bus_listen_heartbeat = None
        self.bus_main = None
        self._init_can()

        # ROS消息缓存
        self.imu_msg = Imu()
        self.vector3_msg = Vector3()
        self.imu_msg.header.frame_id = self.frame_id

    def _init_can(self):
        try:
            self.bus_listen_heartbeat = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.bus_main = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.running = True
            self.heartbeat_thread = threading.Thread(target=self.listen_heartbeat, daemon=True)
            self.heartbeat_thread.start()
            log("CAN初始化成功，心跳监听线程已启动")
        except can.CanError as e:
            log(f"CAN初始化失败: {e}")

    def listen_heartbeat(self):
        """线程：持续监听心跳报文，超时报警"""
        while self.running:
            try:
                msg = self.bus_listen_heartbeat.recv(timeout=0.1)
                if msg and msg.arbitration_id == self.heartbeat_cobid:
                    with self.lock:
                        self.last_heartbeat_time = time.time()
                        self.heartbeat_status = msg.data[0]
                        log(f"收到心跳报文: 状态={self.heartbeat_status}")
                with self.lock:
                    if self.last_heartbeat_time != 0 and (time.time() - self.last_heartbeat_time) > 1.5:
                        log("警告: 1.5秒未收到心跳报文")
                        self.last_heartbeat_time = 0
            except Exception as e:
                log(f"心跳监听异常: {e}")
                time.sleep(0.2)

    def get_map_range(self):
        """SDO获取map_range"""
        request_data = [0x40, 0x01, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
        msg = can.Message(
            arbitration_id=self.sdo_client_cobid,
            data=request_data,
            is_extended_id=False
        )
        try:
            self.bus_main.send(msg)
        except can.CanError as e:
            log(f"发送SDO请求失败: {e}")
            return False

        try:
            msg = self.bus_main.recv(timeout=0.5)
            if msg and msg.arbitration_id == self.sdo_server_cobid and msg.dlc == 8:
                data = msg.data
                if data[0] == 0x43 and data[1] == 0x01 and data[2] == 0x60 and data[3] == 0x01:
                    self.map_range = struct.unpack('<f', data[4:8])[0]
                    log(f"成功获取map_range: {self.map_range}")
                    return True
        except Exception as e:
            log(f"接收TSDO响应异常: {e}")
        return False

    def change_heartbeat_status(self, target_status=0x01):
        """更改心跳报文的状态"""
        msg = can.Message(
            arbitration_id=0x000,
            data=[0x01, self.node_id],
            is_extended_id=False
        )
        try:
            self.bus_main.send(msg)
        except can.CanError as e:
            log(f"发送心跳状态更改请求失败: {e}")
            return False

        log(f"已发送心跳状态更改请求，等待状态变为0x{target_status:02x}...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            print(self.heartbeat_status)
            with self.lock:
                if self.heartbeat_status == target_status:
                    log(f"心跳状态已更改为: {self.heartbeat_status}")
                    return True
            time.sleep(0.1)
        log("心跳状态更改超时")
        return False

    def map_value(value, original_min=-32767, original_max=32767, new_min=-1, new_max=1):
        """数值归一化映射"""
        return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min

    def parse_and_publish(self, msg):
        """解析PDO数据并发布ROS消息"""
        if msg.arbitration_id == self.pdo_cobids[1] and msg.dlc == 8:
            self.imu_msg.orientation.x = self.map_value(struct.unpack('<h', msg.data[0:2])[0])
            self.imu_msg.orientation.y = self.map_value(struct.unpack('<h', msg.data[2:4])[0])
            self.imu_msg.orientation.z = self.map_value(struct.unpack('<h', msg.data[4:6])[0])
            self.imu_msg.orientation.w = self.map_value(struct.unpack('<h', msg.data[6:8])[0])
            log(f"四元数: {self.imu_msg.orientation.x}, {self.imu_msg.orientation.y}, {self.imu_msg.orientation.z}, {self.imu_msg.orientation.w}")
        elif msg.arbitration_id == self.pdo_cobids[2] and msg.dlc == 6:
            self.imu_msg.angular_velocity.x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range
            self.imu_msg.angular_velocity.y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range
            self.imu_msg.angular_velocity.z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range
            log(f"角速度: {self.imu_msg.angular_velocity.x}, {self.imu_msg.angular_velocity.y}, {self.imu_msg.angular_velocity.z}")
        elif msg.arbitration_id == self.pdo_cobids[3] and msg.dlc == 6:
            self.imu_msg.linear_acceleration.x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range
            self.imu_msg.linear_acceleration.y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range
            self.imu_msg.linear_acceleration.z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range
            log(f"线性加速度: {self.imu_msg.linear_acceleration.x}, {self.imu_msg.linear_acceleration.y}, {self.imu_msg.linear_acceleration.z}")
        elif msg.arbitration_id == self.pdo_cobids[4] and msg.dlc == 6:
            self.vector3_msg.x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range
            self.vector3_msg.y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range
            self.vector3_msg.z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range
            log(f"磁场数据: {self.vector3_msg.x}, {self.vector3_msg.y}, {self.vector3_msg.z}")
            
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_pub.publish(self.imu_msg)
        self.vector3_pub.publish(self.vector3_msg)

    def read_data(self):
        """持续读取PDO数据"""
        try:
            while self.running and not rospy.is_shutdown():
                msg = self.bus_main.recv(timeout=0.1)
                if msg is not None:
                    self.parse_and_publish(msg)
        except Exception as e:
            log(f"PDO数据读取异常: {e}")

    def run(self):
        """主流程"""
        try:
            while not self.get_map_range() and self.running:
                log("等待获取map_range")
                with self.lock:
                    log(f"当前heartbeat_status: {self.heartbeat_status}")
                time.sleep(1)
            while not self.change_heartbeat_status() and self.running:
                log("更改心跳状态失败，重试中...")
                time.sleep(1)
            log("心跳状态已成功更改为0x01")
            log("开始监听PDO数据...")
            self.data_thread = threading.Thread(target=self.read_data, daemon=True)
            self.data_thread.start()
            rospy.spin()
        except KeyboardInterrupt:
            log("用户中断操作")
            self.stop()

    def stop(self):
        """安全关闭CAN连接和线程"""
        self.running = False
        rospy.signal_shutdown("节点关闭")
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        if self.bus_listen_heartbeat:
            try:
                self.bus_listen_heartbeat.shutdown()
            except Exception:
                pass
        if self.bus_main:
            try:
                self.bus_main.shutdown()
            except Exception:
                pass
        log("CAN连接已关闭")

def main():
    rospy.init_node('canopen_connector', anonymous=True)
    
    # 从 ROS 参数服务器获取参数
    node_id = rospy.get_param('~node_id', 0x10)
    bustype = rospy.get_param('~bustype', 'socketcan')
    channel = rospy.get_param('~channel', 'can0')
    imu_topic = rospy.get_param('~imu_topic', '/imu_data')
    mag_topic = rospy.get_param('~magnetic_topic', '/magnetic_data')
    frame_id = rospy.get_param('~frame_id', 'imu')
    
    connector = CanopenConnector(
        node_id=node_id,
        bustype=bustype,
        channel=channel,
        imu_topic=imu_topic,
        mag_topic=mag_topic,
        frame_id=frame_id
    )
    try:
        connector.run()
    except KeyboardInterrupt:
        log("用户中断操作")
    except Exception as e:
        log(f"运行时错误: {e}")
    finally:
        connector.stop()

if __name__ == "__main__":
    main()