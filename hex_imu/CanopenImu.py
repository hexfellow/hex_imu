#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import numpy as np
import os

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from utility import DataInterface
from utility import ImuUtil


class CanopenImu:
    def __init__(self):
        """Initialize CANopen IMU"""
        self.__data_interface = DataInterface("CanopenImu")
        self.__can_param = self.__data_interface.get_can_param()
        self.__imu_util = ImuUtil(self.__can_param, self.__data_interface)
    
    def work(self) -> None:
        """Main work loop"""
        try:
            if not self.__imu_util.get_map_range():
                print("Failed to get map_range")
                return

            while self.__data_interface.ok():
                if not self.__imu_util.get_running():
                    break
                imu_quat_stamped, mag_stamped = self.__imu_util.get_PDO_data()
                if imu_quat_stamped and mag_stamped:
                    self.__data_interface.pub_imu(imu_quat_stamped)
                    self.__data_interface.pub_magnetic(mag_stamped)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Runtime error: {e}")
        finally:
            self.__imu_util.stop_imu_canopen()
            self.__data_interface.shutdown()
        
def main():
    try:
        canopen_imu = CanopenImu()
        canopen_imu.work()
    except Exception as e:
        print(f"Program exited with error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()