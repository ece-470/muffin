#!/usr/bin/env python
from ctypes import *

BAXTER_ARM_JOINTS_NUM = 7
SY = 0
SP = 1
SR = 2
EP = 3
WY = 4
WP = 5
WY2 = 6

class JOINTS(Structure):
   _pack_ = 1
   _fields_ = [("ref",c_double),
               ("pos",c_double),
               ("tor",c_double)]

class BAXTER(Structure):
   _pack_ = 1
   _fields_ = [("arm", JOINTS*BAXTER_ARM_JOINTS_NUM),
               ("time",c_double)]

offset_right = [0.0] * BAXTER_ARM_JOINTS_NUM
offset_right[SY] = 0.78
offset_right[SP] = 0.0
offset_right[SR] = 0.0
offset_right[EP] = 1.57
offset_right[WY] = 0.0
offset_right[WP] = 0.0
offset_right[WY2] = 0.0

offset_left = [0.0] * BAXTER_ARM_JOINTS_NUM
offset_left[SY] = -0.78
offset_left[SP] = 0.0
offset_left[SR] = 0.0
offset_left[EP] = 1.57
offset_left[WY] = 0.0
offset_left[WP] = 0.0
offset_left[WY2] = 0.0

dir_right = [0.0] * BAXTER_ARM_JOINTS_NUM
dir_right[SY] = 1.0
dir_right[SP] = 1.0
dir_right[SR] = 1.0
dir_right[EP] = 1.0
dir_right[WY] = 1.0
dir_right[WP] = 1.0
dir_right[WY2] = 1.0

dir_left = [0.0] * BAXTER_ARM_JOINTS_NUM
dir_left[SY] = 1.0
dir_left[SP] = 1.0
dir_left[SR] = 1.0
dir_left[EP] = 1.0
dir_left[WY] = 1.0
dir_left[WP] = 1.0
dir_left[WY2] = 1.0

RIGHT = 1
LEFT = 2
