#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import queue
import typing
from abc import ABC, abstractmethod
from hex_utils import HexSensorImuStamped, HexSensorMagStamped

class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._can_param = {}
        
        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        self.shutdown()

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def get_can_param(self) -> dict:
        return self._can_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_imu(self, out: HexSensorImuStamped):
        raise NotImplementedError("InterfaceBase.pub_imu")

    @abstractmethod
    def pub_magnetic(self, out: HexSensorMagStamped):
        raise NotImplementedError("InterfaceBase.pub_magnetic")

   
   