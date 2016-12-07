#!/usr/bin/env python
from ctypes import *

BAXTER_ARM_JOINTS_NUM = 7
NUM_ARMS = 2
RIGHT = 0
LEFT = 1
SY = 0
SP = 1
SR = 2
EP = 3
WY = 4
WP = 5
WY2 = 6

STATE = 'baxter_state'
REF = 'baxter_ref'

class JOINTS(Structure):
   _pack_ = 1
   _fields_ = [("ref",c_double),
               ("pos",c_double),
               ("tor",c_double)]

class ARM(Structure):
   _pack_ = 1
   _fields_ = [("joint",JOINTS*BAXTER_ARM_JOINTS_NUM)]

class STATE(Structure):
   _pack_ = 1
   _fields_ = [("arm",ARM*NUM_ARMS),
               ("time",c_double)]

class JOINTOFFSET(Structure):
   _pack_ = 1
   _fields_ = [("thetaOff",c_double),
               ("thetaDir",c_double)]

class ARMOFFSET(Structure):
   _pack_ = 1
   _fields_ = [("joint", JOINTOFFSET*BAXTER_ARM_JOINTS_NUM)]

class OFFSET(Structure):
   _pack_ = 1
   _fields_ = [("arm", ARMOFFSET*NUM_ARMS)]

off = OFFSET()

off.arm[RIGHT].joint[SY].thetaOff = 0.78
off.arm[RIGHT].joint[SY].thetaDir = 1.0
off.arm[RIGHT].joint[SP].thetaOff = 0.0
off.arm[RIGHT].joint[SP].thetaDir = 1.0
off.arm[RIGHT].joint[SR].thetaOff = 0.0
off.arm[RIGHT].joint[SR].thetaDir = 1.0
off.arm[RIGHT].joint[EP].thetaOff = 1.57
off.arm[RIGHT].joint[EP].thetaDir = 1.0
off.arm[RIGHT].joint[WY].thetaOff = 0.0
off.arm[RIGHT].joint[WY].thetaDir = -1.0
off.arm[RIGHT].joint[WP].thetaOff = 0.0
off.arm[RIGHT].joint[WP].thetaDir = 1.0
off.arm[RIGHT].joint[WY2].thetaOff = 0.0
off.arm[RIGHT].joint[WY2].thetaDir = -1.0

off.arm[LEFT].joint[SY].thetaOff = -0.78
off.arm[LEFT].joint[SY].thetaDir = 1.0
off.arm[LEFT].joint[SP].thetaOff = 0.0
off.arm[LEFT].joint[SP].thetaDir = 1.0
off.arm[LEFT].joint[SR].thetaOff = 0.0
off.arm[LEFT].joint[SR].thetaDir = 1.0
off.arm[LEFT].joint[EP].thetaOff = 1.57
off.arm[LEFT].joint[EP].thetaDir = 1.0
off.arm[LEFT].joint[WY].thetaOff = 0.0
off.arm[LEFT].joint[WY].thetaDir = -1.0
off.arm[LEFT].joint[WP].thetaOff = 0.0
off.arm[LEFT].joint[WP].thetaDir = 1.0
off.arm[LEFT].joint[WY2].thetaOff = 0.0
off.arm[LEFT].joint[WY2].thetaDir = -1.0

def B2A(state):
   stateOut = STATE()
   for arm in range(0,NUM_ARMS):
      for joint in range(0,BAXTER_ARM_JOINTS_NUM):
         stateOut.arm[arm].joint[joint].ref = state.arm[arm].joint[joint].ref * off.arm[arm].joint[joint].thetaDir - off.arm[arm].joint[joint].thetaOff
         stateOut.arm[arm].joint[joint].pos = state.arm[arm].joint[joint].pos * off.arm[arm].joint[joint].thetaDir - off.arm[arm].joint[joint].thetaOff
   return stateOut

def A2B(state):
   stateOut = STATE()
   for arm in range(0,NUM_ARMS):
      for joint in range(0,BAXTER_ARM_JOINTS_NUM):
         stateOut.arm[arm].joint[joint].ref = (state.arm[arm].joint[joint].ref + off.arm[arm].joint[joint].thetaOff) * off.arm[arm].joint[joint].thetaDir
         stateOut.arm[arm].joint[joint].pos = (state.arm[arm].joint[joint].pos + off.arm[arm].joint[joint].thetaOff) * off.arm[arm].joint[joint].thetaDir
   return stateOut

