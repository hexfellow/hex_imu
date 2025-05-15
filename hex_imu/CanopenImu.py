#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import can
import threading
import time
import sys
import struct
import numpy as np
import os

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from utility import DataInterface
from hex_utils import HexStamp
from hex_utils import HexSensorImuQuat, HexSensorImuQuatStamped
from hex_utils import HexSensorMag, HexSensorMagStamped

class CanopenImu:
    def __init__(self):
        ### data interface
        self.__data_interface = DataInterface("CanopenImu")
        self.__can_param = self.__data_interface.get_can_param()
        # CAN parameters
        self.node_id = self.__can_param["node_id"]
        self.bustype = self.__can_param["bustype"]
        self.channel = self.__can_param["channel"]
        self.sdo_client_cobid = 0x600 + self.node_id
        self.sdo_server_cobid = 0x580 + self.node_id
        self.heartbeat_cobid = 0x700 + self.node_id
        self.pdo_cobids = {
            1: 0x180 + self.node_id,
            2: 0x280 + self.node_id,
            3: 0x380 + self.node_id,
            4: 0x480 + self.node_id
        }
        
        # 状态变量
        self.map_range_gyro = None
        self.map_range_acc = None
        self.map_range_mag = None
        self.heartbeat_status = None
        self.last_heartbeat_time = 0
        self.running = False
        self.lock = threading.Lock()
        self.heartbeat_thread = None
        self.data_thread = None

        # CAN总线初始化
        self.bus_heartbeat = None
        self.bus_maprange = None
        self.bus_pdo = None
        self._init_can()
        
        # ROS消息缓存
        self._init_ros()
        
        self.prep_map_range = False
        
                
    def _init_can(self):
        try:
            self.bus_heartbeat = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.bus_heartbeat.set_filters([{"can_id": self.heartbeat_cobid, "can_mask": 0x77F, "extended": False}])
            self.bus_maprange = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.bus_maprange.set_filters([{"can_id": self.sdo_server_cobid, "can_mask": 0x7FF, "extended": False}])
            self.bus_pdo = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.bus_pdo.set_filters([{"can_id": cobid, "can_mask": 0x7FF, "extended": False} for cobid in self.pdo_cobids.values()])

            
            self.running = True
            self.heartbeat_thread = threading.Thread(target=self.listen_heartbeat, daemon=True)
            self.heartbeat_thread.start()
            print("CAN初始化成功，心跳监听线程已启动")
        except can.CanError as e:
            print(f"CAN初始化失败: {e}")
    
    def _init_ros(self):
        self.orientation_x = None
        self.orientation_y = None
        self.orientation_z = None
        self.orientation_w = None           
        self.angular_velocity_x = None
        self.angular_velocity_y = None
        self.angular_velocity_z = None
        self.linear_acceleration_x = None
        self.linear_acceleration_y = None
        self.linear_acceleration_z = None
        self.mag_x = None
        self.mag_y = None
        self.mag_z = None

    def listen_heartbeat(self):
        """线程：持续监听心跳报文，超时报警"""
        while self.running:
            try:
                msg = self.bus_heartbeat.recv(timeout=0.1)
                if msg and msg.arbitration_id == self.heartbeat_cobid:
                    print(f"持续监听心跳报文收到心跳报文: {msg}")
                    with self.lock:
                        self.last_heartbeat_time = time.time()
                        self.heartbeat_status = msg.data[0]
                        # print(f"收到心跳报文: 状态={self.heartbeat_status}")
                        if self.prep_map_range == True and self.heartbeat_status != 0x05:
                            while msg and not self.change_heartbeat_status() and self.running:
                                print("更改心跳状态失败，重试中...")
                                time.sleep(0.5)
                    
                with self.lock:
                    if self.last_heartbeat_time != 0 and (time.time() - self.last_heartbeat_time) > 1.5:
                        print("警告: 1.5秒未收到心跳报文")
                        self.last_heartbeat_time = 0
            except Exception as e:
                print(f"心跳监听异常: {e}")
                time.sleep(0.2)
                
    def request_map_range(self, name, index):
        """单独请求某个map_range"""
        request_data = [0x40, index, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
        try:
            msg = can.Message(
                arbitration_id=self.sdo_client_cobid,
                data=request_data,
                is_extended_id=False
            )
            self.bus_maprange.send(msg)
            print(f"已发送 SDO 请求获取 {name} map_range")
            return True
        except can.CanError as e:
            print(f"发送 SDO 请求失败: {e}")
            return False
    
    def process_response(self, msg):
        """处理接收到的 SDO 响应消息"""
        if msg.arbitration_id != self.sdo_server_cobid or msg.dlc != 8:
            return False

        data = msg.data
        if data[0] != 0x43:
            return False
        print(f"接收到 SDO 响应: {msg}")
        if data[1] == 0x01:
            self.map_range_gyro = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_gyro: {self.map_range_gyro}")
            return True
        elif data[1] == 0x02:
            self.map_range_acc = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_acc: {self.map_range_acc}")
            return True
        elif data[1] == 0x03:
            self.map_range_mag = struct.unpack('<f', data[4:8])[0]
            print(f"成功获取 map_range_mag: {self.map_range_mag}")
            return True
    
    def get_map_range(self, timeout=5):
        start_time = time.time()

        while time.time() - start_time < timeout:
            for name, index in [('gyro', 0x01), ('acc', 0x02), ('mag', 0x03)]:
                if getattr(self, f"map_range_{name}") is None:
                    if not self.request_map_range(name, index):
                        continue
                    try:
                        msg = self.bus_maprange.recv(timeout=0.1)
                        if msg and self.process_response(msg):
                            print(f"{name} map_range 获取成功")
                    except Exception as e:
                        print(f"接收 SDO 响应异常: {e}")
            if self.map_range_gyro and self.map_range_acc and self.map_range_mag:
                print("所有 map_range 获取成功！")
                return True
        print("获取map_range超时")
        return False
        
    def change_heartbeat_status(self, target_status=0x05):
        """更改心跳报文的状态"""
        msg = can.Message(
            arbitration_id=0x000,
            data=[0x01, self.node_id],
            is_extended_id=False
        )
        try:
            self.bus_heartbeat.send(msg)
            print(f"更改心跳报文的状态已发送{msg}")
        except can.CanError as e:
            print(f"发送心跳状态更改请求失败: {e}")
            return False

        print(f"已发送心跳状态更改请求，等待状态变为0x{target_status:02x}...")
        start_time = time.time()
        while time.time() - start_time < 2.0:
            with self.lock:
                if self.heartbeat_status == target_status:
                    print(f"心跳状态已更改为: {self.heartbeat_status}")
                    return True
            time.sleep(0.1)
        print("心跳状态更改超时")
        return False
    
    @staticmethod
    def map_value(value, original_min=-32767, original_max=32767, new_min=-1, new_max=1):
        """数值归一化映射"""
        return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min
    
    def parse_and_publish(self, msg):
        """解析PDO数据并发布ROS消息"""
        if msg.arbitration_id == self.pdo_cobids[1] and msg.dlc == 8:
            self.orientation_x = self.map_value(struct.unpack('<h', msg.data[0:2])[0])
            self.orientation_y = self.map_value(struct.unpack('<h', msg.data[2:4])[0])
            self.orientation_z = self.map_value(struct.unpack('<h', msg.data[4:6])[0])
            self.orientation_w = self.map_value(struct.unpack('<h', msg.data[6:8])[0])
            print(f"四元数: {self.orientation_x}, {self.orientation_y}, {self.orientation_z}, {self.orientation_w}")
        elif msg.arbitration_id == self.pdo_cobids[2] and msg.dlc == 6:
            self.angular_velocity_x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range_gyro
            self.angular_velocity_y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range_gyro
            self.angular_velocity_z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range_gyro
            print(f"角速度: {self.angular_velocity_x}, {self.angular_velocity_y}, {self.angular_velocity_z}")
        elif msg.arbitration_id == self.pdo_cobids[3] and msg.dlc == 6:
            self.linear_acceleration_x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range_acc
            self.linear_acceleration_y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range_acc
            self.linear_acceleration_z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range_acc
            print(f"线性加速度: {self.linear_acceleration_x}, {self.linear_acceleration_y}, {self.linear_acceleration_z}")
        elif msg.arbitration_id == self.pdo_cobids[4] and msg.dlc == 6:
            self.mag_x = struct.unpack('<h', msg.data[0:2])[0] * self.map_range_mag
            self.mag_y = struct.unpack('<h', msg.data[2:4])[0] * self.map_range_mag
            self.mag_z = struct.unpack('<h', msg.data[4:6])[0] * self.map_range_mag
            print(f"磁场数据: {self.mag_x}, {self.mag_y}, {self.mag_z}")
        
        # 检查所有数据是否都已更新
        acc = np.array([
            self.linear_acceleration_x,
            self.linear_acceleration_y,
            self.linear_acceleration_z
        ])
        gyro = np.array([
            self.angular_velocity_x,
            self.angular_velocity_y,
            self.angular_velocity_z
        ])
        quat = np.array([
            self.orientation_x,
            self.orientation_y,
            self.orientation_z,
            self.orientation_w
        ])
        magnetic_field = np.array([
            self.mag_x,
            self.mag_y,
            self.mag_z
        ])

        if (np.all(acc) and np.all(gyro) and np.all(quat) and np.all(magnetic_field)):
            
            imu_quat = HexSensorImuQuat(acc, gyro, quat)
            mag = HexSensorMag(magnetic_field)
            sec = int(time.time())
            nsec = int((time.time() - sec) * 1e9)
            stamp = HexStamp(sec, nsec)
            
            imu_quat_stamped = HexSensorImuQuatStamped(stamp, imu_quat)
            mag_stamped = HexSensorMagStamped(stamp, mag)
            
            self.__data_interface.pub_imu(imu_quat_stamped)
            self.__data_interface.pub_magnetic(mag_stamped)
            self._init_ros()
        
    
    def read_data(self):
        """持续读取PDO数据"""
        try:
            while self.running and self.__data_interface.ok():
                msg = self.bus_pdo.recv(timeout=0.1)
                if msg is not None:
                    self.parse_and_publish(msg)
        except Exception as e:
            print(f"PDO数据读取异常: {e}")
    
    def stop(self):
        """安全关闭CAN连接和线程"""
        print("正在关闭CAN连接...")
        self.running = False
        self.__data_interface.shutdown()
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        if self.bus_heartbeat:
            try:
                self.bus_heartbeat.shutdown()
            except Exception:
                pass
        if self.bus_maprange:
            try:
                self.bus_maprange.shutdown()
            except Exception:
                pass
        if self.bus_pdo:
            try:
                self.bus_pdo.shutdown()
            except Exception:
                pass
        print("CAN连接已关闭")
    
    def run(self):
        """主流程"""
        try:
            while not self.get_map_range() and self.running:
                print("等待获取map_range")
                with self.lock:
                    print(f"当前heartbeat_status: {self.heartbeat_status}")
                time.sleep(0.5)
            while not self.change_heartbeat_status() and self.running:
                print("更改心跳状态失败，重试中...")
                time.sleep(0.5)
            self.prep_map_range = True
            print("开始监听PDO数据...")
            self.data_thread = threading.Thread(target=self.read_data, daemon=True)
            self.data_thread.start()
            
            while self.running and self.__data_interface.ok():
                try:
                    self.__data_interface.sleep()
                except KeyboardInterrupt:
                    print("\n检测到用户中断，正在安全退出...")
                    sys.exit(0)
                    self.running = False
                    break

        except Exception as e:
            print(f"运行时错误: {e}")
            sys.exit(1)
            self.running = False
    
    def work(self):
        """主工作函数"""
        try:
            self.run()
        except KeyboardInterrupt:
            print("用户中断操作")
            sys.exit(0)
        finally:
            self.stop()
    

def main():
    canopen_imu = CanopenImu()
    canopen_imu.work()


if __name__ == '__main__':
    main()
