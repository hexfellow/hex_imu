#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import can
import threading
import time
import struct
import numpy as np
from typing import Optional, Tuple, Dict, Any
from scipy.spatial.transform import Rotation as R
import math

from hex_utils import HexStamp
from hex_utils import HexSensorImuQuat, HexSensorImuQuatStamped
from hex_utils import HexSensorMag, HexSensorMagStamped
from utility import DataInterface

class ImuUtil:

    def __init__(self, can_param: dict, dataInterface) :
        self.__dataInterface = dataInterface
        self._init_params(can_param)
        self._init_can()
        self._reset_data()

    def _init_params(self, can_param: dict) -> None:
        # Initialize parameters
        self.__node_id = can_param["node_id"]
        self.__channel = can_param["channel"]
        
        # CAN ID configuration
        self.__sdo_client_cobid = 0x600 + self.__node_id
        self.__sdo_server_cobid = 0x580 + self.__node_id
        self.__heartbeat_cobid = 0x700 + self.__node_id
        self.__pdo_cobids = {
            1: 0x180 + self.__node_id,
            2: 0x280 + self.__node_id,
            3: 0x380 + self.__node_id,
            4: 0x480 + self.__node_id
        }
        
        # Other variables
        self.__map_range_gyro = None
        self.__map_range_acc = None
        self.__map_range_mag = None
        self.__heartbeat_status = None
        self.__last_heartbeat_time = 0
        self.__imu_running = True
        self.__lock = threading.Lock()
        self.__heartbeat_thread = None
        self.__heartbeat_change = False

    def _init_can(self) -> None:
        try:
            self.__bus_heartbeat = self._create_can_bus([self.__heartbeat_cobid])
            self.__bus_maprange = self._create_can_bus([self.__sdo_server_cobid])
            self.__bus_pdo = self._create_can_bus(list(self.__pdo_cobids.values()))
            
            self.__heartbeat_thread = threading.Thread(target=self._listen_heartbeat, daemon=True)
            self.__heartbeat_thread.start()
        except can.CanError as e:
            print(f"CAN initialization failed: {e}")
            raise

    def _create_can_bus(self, cobids: list) -> can.Bus:
        bus = can.interface.Bus(interface="socketcan", channel=self.__channel)
        filters = [{"can_id": cobid, "can_mask": 0x7FF, "extended": False} for cobid in cobids]
        bus.set_filters(filters)
        return bus

    def _reset_data(self) -> None:
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

    def get_running(self) -> None:
        return self.__imu_running
    
    def _listen_heartbeat(self) -> None:
        while self.__dataInterface.ok() and self.__imu_running:
            try:
                msg = self.__bus_heartbeat.recv(timeout=0.1)
                if msg and msg.arbitration_id == self.__heartbeat_cobid:
                    with self.__lock:
                        self.__last_heartbeat_time = time.time()
                        self.__heartbeat_status = msg.data[0]
                        if self.__heartbeat_change and self.__heartbeat_status != 0x05:
                            self.__imu_running = False
                            print("IMU disconnected, please restart the program")
                        
                with self.__lock:
                    if self.__last_heartbeat_time == 0 or time.time() - self.__last_heartbeat_time > 1.5:
                        print("Warning: No heartbeat message received")

            except Exception as e:
                print(f"Heartbeat monitoring exception: {e}")
                if not self.__dataInterface.ok():
                    break

    def _request_map_range(self, name: str, index: int) -> bool:
        request_data = [0x40, index, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
        try:
            msg = can.Message(
                arbitration_id=self.__sdo_client_cobid,
                data=request_data,
                is_extended_id=False
            )
            self.__bus_maprange.send(msg)
            return True
        except can.CanError as e:
            print(f"Failed to send SDO request: {e}")
            return False

    def _process_map_range(self, msg: can.Message) -> bool:
        if msg.arbitration_id != self.__sdo_server_cobid or msg.dlc != 8:
            return False

        data = msg.data
        if data[0] != 0x43:
            return False

        index = data[1]
        value = struct.unpack('<f', data[4:8])[0]
        
        if index == 0x01:
            self.__map_range_gyro = value
        elif index == 0x02:
            self.__map_range_acc = value
        elif index == 0x03:
            self.__map_range_mag = value
        
        return True

    def _change_heartbeat_status(self, target_status: int = 0x05) -> bool:
        if self.__heartbeat_status == target_status:
            self.__heartbeat_change = True
            return True
        
        msg = can.Message(
            arbitration_id=0x000,
            data=[0x01, self.__node_id],
            is_extended_id=False
        )
        try:
            self.__bus_heartbeat.send(msg)
        except can.CanError as e:
            print(f"Failed to send heartbeat status change request: {e}")
            return False

        start_time = time.time()
        while self.__dataInterface.ok() and time.time() - start_time < 2.0 :
            with self.__lock:
                if self.__heartbeat_status == target_status:
                    self.__heartbeat_change = True
                    return True
            time.sleep(0.1)
        return False

    @staticmethod
    def _map_value(value: float, original_min: float = -32767, original_max: float = 32767,
                   new_min: float = -1, new_max: float = 1) -> float:
        return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min

    def _parse_pdo_data(self, msg: can.Message) -> Tuple[Optional[HexSensorImuQuatStamped], Optional[HexSensorMagStamped]]:
        if msg.arbitration_id == self.__pdo_cobids[1] and msg.dlc == 8:
            self.__orientation_w = self._map_value(struct.unpack('<h', msg.data[0:2])[0])
            self.__orientation_x = self._map_value(struct.unpack('<h', msg.data[2:4])[0])
            self.__orientation_y = self._map_value(struct.unpack('<h', msg.data[4:6])[0])
            self.__orientation_z = self._map_value(struct.unpack('<h', msg.data[6:8])[0])
        elif msg.arbitration_id == self.__pdo_cobids[2] and msg.dlc == 6:
            if self.__map_range_gyro is None:
                return None, None
            self.__angular_velocity_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_gyro
            self.__angular_velocity_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_gyro
            self.__angular_velocity_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_gyro
        elif msg.arbitration_id == self.__pdo_cobids[3] and msg.dlc == 6:
            if self.__map_range_acc is None:
                return None, None
            self.__linear_acceleration_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_acc
            self.__linear_acceleration_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_acc
            self.__linear_acceleration_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_acc
        elif msg.arbitration_id == self.__pdo_cobids[4] and msg.dlc == 6:
            if self.__map_range_mag is None:
                return None, None
            self.__mag_x = struct.unpack('<h', msg.data[0:2])[0] * self.__map_range_mag
            self.__mag_y = struct.unpack('<h', msg.data[2:4])[0] * self.__map_range_mag
            self.__mag_z = struct.unpack('<h', msg.data[4:6])[0] * self.__map_range_mag
            
            # vector_magnitude = math.sqrt(self.__mag_x**2 + self.__mag_y**2)
            # if vector_magnitude == 0:
            #     self.__mag_angle = 0
            # else:
            #     angle_rad = math.atan2(self.__mag_y, self.__mag_x)
            #     angle_deg = math.degrees(angle_rad)
            #     if angle_deg < 0:
            #         self.__mag_angle = 360 + angle_deg
            #     else:
            #         self.__mag_angle = angle_deg
            # print(f"Magnetic field angle: {self.__mag_angle:.2f}°")

        if all(v is not None for v in [
            self.__orientation_x, self.__orientation_y, self.__orientation_z, self.__orientation_w,
            self.__angular_velocity_x, self.__angular_velocity_y, self.__angular_velocity_z,
            self.__linear_acceleration_x, self.__linear_acceleration_y, self.__linear_acceleration_z,
            self.__mag_x, self.__mag_y, self.__mag_z
        ]):
            acc = np.array([self.__linear_acceleration_x, self.__linear_acceleration_y, self.__linear_acceleration_z])
            gyro = np.array([self.__angular_velocity_x, self.__angular_velocity_y, self.__angular_velocity_z])
            quat = np.array([self.__orientation_x, self.__orientation_y, self.__orientation_z, self.__orientation_w])
            magnetic_field = np.array([self.__mag_x, self.__mag_y, self.__mag_z])

            sec = int(time.time())
            nsec = int((time.time() - sec) * 1e9)
            stamp = HexStamp(sec, nsec)

            imu_quat = HexSensorImuQuat(acc, gyro, quat)
            mag = HexSensorMag(magnetic_field)
            imu_quat_stamped = HexSensorImuQuatStamped(stamp, imu_quat)
            mag_stamped = HexSensorMagStamped(stamp, mag)

            self._reset_data()
            return imu_quat_stamped, mag_stamped

        return None, None

    def get_map_range(self) -> bool:
        while self.__dataInterface.ok() and  not self._listen_map_range(3) and self.__imu_running:
            time.sleep(0.1)
        while self.__dataInterface.ok() and not self._change_heartbeat_status(0x05) and self.__imu_running:
            time.sleep(0.1)
        return True

    def _listen_map_range(self, timeout: int = 3) -> bool:
        start_time = time.time()
        while self.__dataInterface.ok() and time.time() - start_time < timeout and self.__imu_running:
            for name, index in [('gyro', 0x01), ('acc', 0x02), ('mag', 0x03)]:
                map_range_dict = {
                    'gyro': self.__map_range_gyro,
                    'acc': self.__map_range_acc,
                    'mag': self.__map_range_mag
                }
                
                if map_range_dict[name] is None:
                    if not self._request_map_range(name, index):
                        continue
                    try:
                        msg = self.__bus_maprange.recv(timeout=0.1)
                        if msg :
                            self._process_map_range(msg)
                    except Exception as e:
                        print(f"SDO response exception: {e}")
            
            if all(v is not None for v in [self.__map_range_gyro, self.__map_range_acc, self.__map_range_mag]):
                return True
        return False

    def get_PDO_data(self) -> Tuple[Optional[HexSensorImuQuatStamped], Optional[HexSensorMagStamped]]:
        try:
            while self.__dataInterface.ok() and self.__imu_running :
                msg = self.__bus_pdo.recv(timeout=0.1)
                if msg is not None:
                    return self._parse_pdo_data(msg)
        except Exception as e:
            print(f"PDO reading exception: {e}")
        return None, None

    def stop_imu_canopen(self) -> None:
        if self.__heartbeat_thread and self.__heartbeat_thread.is_alive():
            self.__heartbeat_thread.join(timeout=2)
        for bus in [self.__bus_heartbeat, self.__bus_maprange, self.__bus_pdo]:
            if bus:
                try:
                    bus.shutdown()
                except Exception:
                    pass