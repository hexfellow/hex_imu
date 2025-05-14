#!/usr/bin/env python3
# -*- coding:utf-8 -*-


"""
线程一：
监听心跳报文=>看是否能收到消息，如果没收到，1.5秒就打印

主线程：
1. SDO获取maprange
    while 没获取到maprange
    1. 先发送RSDO请求
        判断发出去的是否是8字节长度
        客服端=>1100 + nodeid 
        01000000 0x6001（注意大小端） 01 4个空字节
    2. 接收TSDO响应
        判断报文是否是8字节长度
        读取服务端=>
        判断 ID+Byte（TSDO 01000011）+返回的是不是0x6001（注意大小端） 01两个地址
        读取map_range
        
2. 改变心跳报文的状态
    while 报文修改状态
        发送请求=>1110+nodeid 0x1017 00 状态0x05
        判断心脏报文有没有变化
        变成0x05就跳
        
3. 读取数据
    PDO
    判断id 对不对
    提取数据
"""
# start_time = time.perf_counter()
# print(f"points_np: {(time.perf_counter()-start_time)*1000} ms")
# pip install python-can
import can
import threading
import time
import sys
import struct
from datetime import datetime

def log(msg):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {msg}")

class CanopenConnector:
    def __init__(self, node_id, bustype='socketcan', channel='can0'):
        self.node_id = node_id
        self.bustype = bustype
        self.channel = channel
        self.bus1 = None
        self.bus2 = None
        self.sdo_client_cobid = 0x600 + node_id
        self.sdo_server_cobid = 0x580 + node_id
        self.heartbeat_cobid = 0x700 + node_id
        self.pdo1_cobid = 0x180 + node_id # PDO1
        self.pdo2_cobid = 0x280 + node_id # PDO2
        self.pdo3_cobid = 0x380 + node_id # PDO3
        self.pdo4_cobid = 0x480 + node_id # PDO4
        self.map_range = None
        self.heartbeat_status = None
        self.last_heartbeat_time = 0
        self.running = False
        self.lock = threading.Lock()
        self.heartbeat_thread = None
        try:
            self.bus1 = can.interface.Bus(interface=self.bustype, channel=self.channel)
            self.bus2 = can.interface.Bus(interface=self.bustype, channel=self.channel)
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
                msg = self.bus1.recv(timeout=0.1)
                # if msg is not None:
                #     print(f"listen_heartbeat: {msg}")
                    
                if msg and msg.arbitration_id == 0x000 :
                    with self.lock:
                        self.last_heartbeat_time = time.time()
                        self.heartbeat_status = msg.data[0]
                        log(f"收到心跳报文: 状态={self.heartbeat_status}")
                # 检查心跳超时
                with self.lock:
                    if self.last_heartbeat_time != 0 and (time.time() - self.last_heartbeat_time) > 1.5:
                        log("警告: 1.5秒未收到心跳报文")
                        self.last_heartbeat_time = 0  # 防止重复报警
            except Exception as e:
                log(f"心跳监听异常: {e}")
                time.sleep(0.2)

    def get_map_range(self):
        """步骤1：SDO获取map_range"""
        request_data = [0x40, 0x01, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
        msg = can.Message(
            arbitration_id=self.sdo_client_cobid,
            data=request_data,
            is_extended_id=False
        )
        try:
            self.bus2.send(msg)
        except can.CanError as e:
            log(f"发送SDO请求失败: {e}")
            return False

        # start_time = time.time()
        # while time.time() - start_time < 2.0:
        try:
            msg = self.bus2.recv(timeout=0.1)
            if msg is not None:
                print(f"TSDOmsg: {msg}")
            if msg and msg.arbitration_id == self.sdo_server_cobid and msg.dlc == 8:
                data = msg.data
                print(f"接收到TSDO响应: {list(data)}")
                print(data[0], data[1], data[2], data[3])
                print(hex(data[0]), hex(data[1]), hex(data[2]), hex(data[3]))
                # 看返回的是不是0x43 TODO:0x80?
                # if data[0] == 0x43 and data[1] == 0x01 and data[2] == 0x60 and data[3] == 0x01:
                    # self.map_range = int.from_bytes(data[4:8], byteorder='little')
                    # log(f"成功获取map_range: {self.map_range}")
                # 
                self.map_range = struct.unpack('<f', data[4:8])[0]
                log(f"成功获取map_range: {self.map_range}")
                
                return True
        except Exception as e:
            log(f"接收TSDO响应异常: {e}")
        # log("获取map_range超时")
        return False

    def change_heartbeat_status(self):
        """步骤2：更改心跳报文的状态，等待变为0x05=>0x01"""
        msg = can.Message(
            arbitration_id=0x000,
            data=[0x01, self.node_id],
            is_extended_id=False
        )
        try:
            self.bus2.send(msg)
        except can.CanError as e:
            log(f"发送心跳状态更改请求失败: {e}")
            return False

        log("已发送心跳状态更改请求，等待状态变为0x01...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            with self.lock:
                if self.heartbeat_status == 0x01:
                    log(f"心跳状态已更改为: {self.heartbeat_status}")
                    return True
            time.sleep(0.1)
        log("心跳状态更改超时")
        return False

    def map_value(self,value):
        original_min = -32767
        original_max = 32767
        new_min = -1
        new_max = 1

        # 线性映射公式
        mapped_value = ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min
        return mapped_value
        
    def read_data(self):
        """步骤3：持续读取PDO数据"""
        try:
            while self.running:
                msg = self.bus2.recv(timeout=0.1)
                if msg and msg.arbitration_id == self.pdo1_cobid and msg.dlc == 8:
                    # x = struct.unpack('<h', msg.data[0:2])[0]*self.map_range
                    # y = struct.unpack('<h', msg.data[2:4])[0]*self.map_range
                    # z = struct.unpack('<h', msg.data[4:6])[0]*self.map_range
                    # w = struct.unpack('<h', msg.data[6:8])[0]*self.map_range
                    x = self.map_value(struct.unpack('<h', msg.data[0:2])[0])
                    y = self.map_value(struct.unpack('<h', msg.data[2:4])[0])
                    z = self.map_value(struct.unpack('<h', msg.data[4:6])[0])
                    w = self.map_value(struct.unpack('<h', msg.data[6:8])[0])
                    log(f"收到PDO1数据: x={x}, y={y}, z={z}, w={w}")
                elif msg and msg.arbitration_id == self.pdo2_cobid and msg.dlc == 6:
                    x = struct.unpack('<h', msg.data[0:2])[0]*self.map_range
                    y = struct.unpack('<h', msg.data[2:4])[0]*self.map_range
                    z = struct.unpack('<h', msg.data[4:6])[0]*self.map_range
                    # x = struct.unpack('<h', msg.data[0:2])[0]
                    # y = struct.unpack('<h', msg.data[2:4])[0]
                    # z = struct.unpack('<h', msg.data[4:6])[0]
                    log(f"收到PDO2数据: x={x}, y={y}, z={z}")
                elif msg and msg.arbitration_id == self.pdo3_cobid and msg.dlc == 6:
                    # 同pdo2的处理
                    pass
                elif msg and msg.arbitration_id == self.pdo4_cobid and msg.dlc == 6:
                    # 同pdo2的处理
                    pass
                else:
                    # log(f"收到未知PDO数据: ID={msg.arbitration_id}, 数据={list(msg.data)}")
                    pass
        except Exception as e:
            log(f"PDO数据读取异常: {e}")

    def run(self):
        """主流程：依次完成map_range获取、心跳状态更改、PDO监听"""
        # 步骤1: 获取map_range
        while not self.get_map_range() and self.running:
            log("获取map_range失败，重试中...")
            with self.lock:
                log(f"当前heartbeat_status: {self.heartbeat_status}")
            time.sleep(1)

        # 步骤2: 更改心跳状态
        while not self.change_heartbeat_status() and self.running:
            log("更改心跳状态失败，重试中...")
            time.sleep(1)
        if not self.running:
            return
        log("心跳状态已成功更改为0x01")

        # 步骤3: 持续读取PDO数据
        log("开始监听PDO数据...")
        self.read_data()

    def stop(self):
        """安全关闭CAN连接和线程"""
        self.running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        if self.bus1:
            try:
                self.bus1.shutdown()
            except Exception:
                pass
        if self.bus2:
            try:
                self.bus2.shutdown()
            except Exception:
                pass
        log("CAN连接已关闭")

def main():
    connector = CanopenConnector(node_id=0x10, bustype='socketcan', channel='can0')
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