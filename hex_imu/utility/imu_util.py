#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import can
import threading
import time
import sys
import struct
import numpy as np
import os

from hex_utils import HexStamp
from hex_utils import HexSensorImuQuat, HexSensorImuQuatStamped
from hex_utils import HexSensorMag, HexSensorMagStamped

class ImuUtil:

    def __init__(self, can_param: dict):
        ### parameters
        self.__can_param = can_param

        # CAN variables
        self.__node_id = self.__can_param["node_id"]
        self.__bustype = self.__can_param["bustype"]
        self.__channel = self.__can_param["channel"]
        self.__sdo_client_cobid = 0x600 + self.__node_id
        self.__sdo_server_cobid = 0x580 + self.__node_id
        self.__heartbeat_cobid = 0x700 + self.__node_id
        self.__pdo_cobids = {
            1: 0x180 + self.__node_id,
            2: 0x280 + self.__node_id,
            3: 0x380 + self.__node_id,
            4: 0x480 + self.__node_id
        }
        self.__heartbeat_status = None
        self.__bus_heartbeat = None
        self.__bus_maprange = None
        self.__bus_pdo = None

        # imu variables
        self.__map_range_gyro = None
        self.__map_range_acc = None
        self.__map_range_mag = None

        # other variables
        self.__last_heartbeat_time = 0
        self.__running = False
        self.__lock = threading.Lock()
        self.__heartbeat_thread = None
        self.__data_thread = None

        self.init_can()
        self.reset_ros()
        print(f"self.__map_range_gyro={self.__map_range_gyro}")

    def init_can(self):
        try:
            self.__bus_heartbeat = can.interface.Bus(interface=self.__bustype, channel=self.__channel)
            self.__bus_heartbeat.set_filters([{"can_id": self.__heartbeat_cobid, "can_mask": 0x77F, "extended": False}])
            self.__bus_maprange = can.interface.Bus(interface=self.__bustype, channel=self.__channel)
            self.__bus_maprange.set_filters([{"can_id": self.__sdo_server_cobid, "can_mask": 0x7FF, "extended": False}])
            self.__bus_pdo = can.interface.Bus(interface=self.__bustype, channel=self.__channel)
            self.__bus_pdo.set_filters([{"can_id": cobid, "can_mask": 0x7FF, "extended": False} for cobid in self.__pdo_cobids.values()])

            self.__running = True
            self.__heartbeat_thread = threading.Thread(target=self.listen_heartbeat, daemon=True)
            self.__heartbeat_thread.start()
            print("CAN初始化成功，心跳监听线程已启动")
        except can.CanError as e:
            print(f"CAN初始化失败: {e}")

    def reset_ros(self):
        self.__orientation_x = None
        self.__orientation_y = None
        self.__orientation_z = None
        self.__orientation_w = None           
        self.__angular_velocity_x = None
        self.__angular_velocity_y = None
        self.__angular_velocity_z = None
        self.__linear_acceleration_x = None
        self.__linear_acceleration_y = None
        self.__linear_acceleration_z = None
        self.__mag_x = None
        self.__mag_y = None
        self.__mag_z = None

    def listen_heartbeat(self):
        """线程：持续监听心跳报文，超时报警"""
        while self.__running:
            try:
                msg = self.__bus_heartbeat.recv(timeout=0.1)
                if msg and msg.arbitration_id == self.__heartbeat_cobid:
                    # print(f"持续监听心跳报文收到心跳报文: {msg}")
                    with self.__lock:
                        self.__last_heartbeat_time = time.time()
                        self.__heartbeat_status = msg.data[0]
                        # print(f"收到心跳报文: 状态={self.__heartbeat_status}")
                with self.__lock:
                    if self.__last_heartbeat_time != 0 and (time.time() - self.__last_heartbeat_time) > 1.5:
                        print("警告: 1.5秒未收到心跳报文")
                        self.__last_heartbeat_time = 0
            except Exception as e:
                print(f"心跳监听异常: {e}")
                sys.exit(0)
    
    def request_map_range(self, name, index):
        request_data = [0x40, index, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
        try:
            msg = can.Message(
                arbitration_id=self.__sdo_client_cobid,
                data=request_data,
                is_extended_id=False
            )
            self.__bus_maprange.send(msg)
            print(f"已发送 SDO 请求获取 {name} map_range")
            return True
        
        except can.CanError as e:
            print(f"发送 SDO 请求失败: {e}")
            return False
    
    def process_map_range(self, msg):
        if msg.arbitration_id != self.__sdo_server_cobid or msg.dlc != 8:
            return False

        data = msg.data
        if data[0] != 0x43:
            return False
        print(f"接收到 SDO 响应: {msg}")
        if data[1] == 0x01:
            self.__map_range_gyro = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_gyro: {self.__map_range_gyro}")
            return True
        elif data[1] == 0x02:
            self.__map_range_acc = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_acc: {self.__map_range_acc}")
            return True
        elif data[1] == 0x03:
            self.__map_range_mag = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_mag: {self.__map_range_mag}")
            return True
    
    def listen_map_range(self, timeout=3):
        start_time = time.time()
        map_range_dict = {
            'gyro': self.__map_range_gyro,
            'acc': self.__map_range_acc,
            'mag': self.__map_range_mag
        }
        while time.time() - start_time < timeout:
            for name, index in [('gyro', 0x01), ('acc', 0x02), ('mag', 0x03)]:
                # if getattr(self, f"\_\_map_range_{name}") is None:# 加了__后会报错
                if map_range_dict[name] is None:
                    if not self.request_map_range(name, index):
                        continue
                    # 成功发送请求map_range,则监听消息
                    try:
                        msg = self.__bus_maprange.recv(timeout=0.1)
                        # 处理收到的消息
                        if msg and self.process_map_range(msg):
                            print(f"{name} map_range 获取成功")
                    except Exception as e:
                        print(f"接收 SDO 响应异常: {e}")
            if self.__map_range_gyro and self.__map_range_acc and self.__map_range_mag:
                print("所有 map_range 获取成功！")
                return True
        print("获取map_range超时")
        return False
    
    def change_heartbeat_status(self, target_status=0x05):
        """更改心跳报文的状态"""
        msg = can.Message(
            arbitration_id=0x000,
            data=[0x01, self.__node_id],
            is_extended_id=False
        )
        try:
            self.__bus_heartbeat.send(msg)
            print(f"更改心跳报文的状态已发送{msg}")
        except can.CanError as e:
            print(f"发送心跳状态更改请求失败: {e}")
            return False

        print(f"已发送心跳状态更改请求，等待状态变为0x{target_status:02x}...")
        start_time = time.time()
        while time.time() - start_time < 2.0:
            with self.__lock:
                if self.__heartbeat_status == target_status:
                    print(f"心跳状态已更改为: {self.__heartbeat_status}")
                    return True
            time.sleep(0.1)
        print("心跳状态更改超时")
        return False
    
    # @staticmethod
    # def map_value(value, original_min=-32767, original_max=32767, new_min=-1, new_max=1):
    #     return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min
    def map_value(self, value, original_min=-32767, original_max=32767, new_min=-1, new_max=1):
        return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min

    def parse_and_publish(self, msg):
        """解析PDO数据并发布ROS消息"""
        if msg.arbitration_id == self.__pdo_cobids[1] and msg.dlc == 8:
            self.__orientation_x = self.map_value(struct.unpack('<h', msg.data[0:2])[0])
            self.__orientation_y = self.map_value(struct.unpack('<h', msg.data[2:4])[0])
            self.__orientation_z = self.map_value(struct.unpack('<h', msg.data[4:6])[0])
            self.__orientation_w = self.map_value(struct.unpack('<h', msg.data[6:8])[0])
            print(f"四元数: {self.__orientation_x}, {self.__orientation_y}, {self.__orientation_z}, {self.__orientation_w}")
        elif msg.arbitration_id == self.__pdo_cobids[2] and msg.dlc == 6:
            self.__angular_velocity_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_gyro
            self.__angular_velocity_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_gyro
            self.__angular_velocity_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_gyro
            print(f"角速度: {self.__angular_velocity_x}, {self.__angular_velocity_y}, {self.__angular_velocity_z}")
        elif msg.arbitration_id == self.__pdo_cobids[3] and msg.dlc == 6:
            self.__linear_acceleration_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_acc
            self.__linear_acceleration_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_acc
            self.__linear_acceleration_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_acc
            print(f"线性加速度: {self.__linear_acceleration_x}, {self.__linear_acceleration_y}, {self.__linear_acceleration_z}")
        elif msg.arbitration_id == self.__pdo_cobids[4] and msg.dlc == 6:
            self.__mag_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_mag
            self.__mag_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_mag
            self.__mag_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_mag
            print(f"磁场数据: {self.__mag_x}, {self.__mag_y}, {self.__mag_z}")
        
        acc = np.array([
            self.__linear_acceleration_x,
            self.__linear_acceleration_y,
            self.__linear_acceleration_z
        ])
        gyro = np.array([
            self.__angular_velocity_x,
            self.__angular_velocity_y,
            self.__angular_velocity_z
        ])
        quat = np.array([
            self.__orientation_x,
            self.__orientation_y,
            self.__orientation_z,
            self.__orientation_w
        ])
        magnetic_field = np.array([
            self.__mag_x,
            self.__mag_y,
            self.__mag_z
        ])

        if (np.all(acc) and np.all(gyro) and np.all(quat) and np.all(magnetic_field)):
            
            imu_quat = HexSensorImuQuat(acc, gyro, quat)
            mag = HexSensorMag(magnetic_field)
            sec = int(time.time())
            nsec = int((time.time() - sec) * 1e9)
            stamp = HexStamp(sec, nsec)
            
            imu_quat_stamped = HexSensorImuQuatStamped(stamp, imu_quat)
            mag_stamped = HexSensorMagStamped(stamp, mag)
            self.reset_ros_ros()
            return imu_quat_stamped, mag_stamped
        print("qweqw")
        return None, None
    
    def get_PDO_data(self):
        """持续读取PDO数据"""
        try:
            while self.__running :
                # 监听PDO
                msg = self.__bus_pdo.recv(timeout=0.1)
                if msg is not None:
                    # 解析数据
                    return self.parse_and_publish(msg)
        except Exception as e:
            print(f"PDO数据读取异常: {e}")
    
    def stop_imu_canopen(self):
        """安全关闭CAN连接和线程"""
        print("正在关闭CAN连接...")
        self.__running = False
        if self.__heartbeat_thread and self.__heartbeat_thread.is_alive():
            self.__heartbeat_thread.join(timeout=2)
        if self.__data_thread and self.__data_thread.is_alive():
            self.__data_thread.join(timeout=2)
        if self.__bus_heartbeat:
            try:
                self.__bus_heartbeat.shutdown()
            except Exception:
                pass
        if self.__bus_maprange:
            try:
                self.__bus_maprange.shutdown()
            except Exception:
                pass
        if self.__bus_pdo:
            try:
                self.__bus_pdo.shutdown()
            except Exception:
                pass
        print("CAN连接已关闭")

    def get_map_range(self):
        try:
            while not self.listen_map_range(3) and self.__running:
                print("等待获取map_range")
                # with self.__lock:
                #     print(f"当前heartbeat_status: {self.__heartbeat_status}")
                time.sleep(0.5)
            while not self.change_heartbeat_status(0x05) and self.__running:
                print("更改心跳状态失败，重试中...")
                time.sleep(0.5)
            print("开始监听PDO数据...")
           
        except KeyboardInterrupt:
            print("\n检测到用户中断，正在安全退出...")
            self.stop_imu_canopen()
            sys.exit(0)

        except Exception as e:
            print(f"运行时错误: {e}")
            self.stop_imu_canopen()
            sys.exit(1)