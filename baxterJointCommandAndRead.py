#!/usr/bin/env python
import argparse
import sys
import rospy
import ach
import baxter_interface as bi
import baxterStructure as bs

def moveArm(ref2, arm, limb):
  ref = bs.A2B(ref2)
  if arm == bs.RIGHT:
     angles       = {'right_s0': ref.arm[bs.RIGHT].joint[bs.SY].ref,
                     'right_s1': ref.arm[bs.RIGHT].joint[bs.SP].ref, 
                     'right_w0': ref.arm[bs.RIGHT].joint[bs.WY].ref, 
                     'right_w1': ref.arm[bs.RIGHT].joint[bs.WP].ref, 
                     'right_w2': ref.arm[bs.RIGHT].joint[bs.WY2].ref, 
                     'right_e0': ref.arm[bs.RIGHT].joint[bs.SR].ref, 
                     'right_e1': ref.arm[bs.RIGHT].joint[bs.EP].ref}
     limb.move_to_joint_positions(angles)
  elif arm == bs.LEFT:
     angles       = {'left_s0': ref.arm[bs.LEFT].joint[bs.SY].ref,
                     'left_s1': ref.arm[bs.LEFT].joint[bs.SP].ref, 
                     'left_w0': ref.arm[bs.LEFT].joint[bs.WY].ref, 
                     'left_w1': ref.arm[bs.LEFT].joint[bs.WP].ref, 
                     'left_w2': ref.arm[bs.LEFT].joint[bs.WY2].ref, 
                     'left_e0': ref.arm[bs.LEFT].joint[bs.SR].ref, 
                     'left_e1': ref.arm[bs.LEFT].joint[bs.EP].ref}
     limb.move_to_joint_positions(angles)

def getState(state,ref,left,right):
  state.arm[bs.LEFT].joint[bs.SY].pos = left.joint_angle('left_s0')
  state.arm[bs.LEFT].joint[bs.SP].pos = left.joint_angle('left_s1')
  state.arm[bs.LEFT].joint[bs.WY].pos = left.joint_angle('left_w0')
  state.arm[bs.LEFT].joint[bs.WP].pos = left.joint_angle('left_w1')
  state.arm[bs.LEFT].joint[bs.WY2].pos = left.joint_angle('left_w2')
  state.arm[bs.LEFT].joint[bs.SR].pos = left.joint_angle('left_e0')
  state.arm[bs.LEFT].joint[bs.EP].pos = left.joint_angle('left_e1')

  state.arm[bs.RIGHT].joint[bs.SY].pos = right.joint_angle('right_s0')
  state.arm[bs.RIGHT].joint[bs.SP].pos = right.joint_angle('right_s1')
  state.arm[bs.RIGHT].joint[bs.WY].pos = right.joint_angle('right_w0')
  state.arm[bs.RIGHT].joint[bs.WP].pos = right.joint_angle('right_w1')
  state.arm[bs.RIGHT].joint[bs.WY2].pos = right.joint_angle('right_w2')
  state.arm[bs.RIGHT].joint[bs.SR].pos = right.joint_angle('right_e0')
  state.arm[bs.RIGHT].joint[bs.EP].pos = right.joint_angle('right_e1')

  state2 = bs.B2A(state)

  for arm in range(0,bs.NUM_ARMS):
    for joint in range(0,bs.BAXTER_ARM_JOINTS_NUM):
      state2.arm[arm].joint[joint].ref = ref.arm[arm].joint[joint].ref

  return state2

def main():
  s = ach.Channel(bs.STATE_CHANNEL)
  r = ach.Channel(bs.REF_CHANNEL)

  print("init node")
  rospy.init_node("baxter_joint_pos_set")
  left  = bi.Limb('left')
  right = bi.Limb('right')
  rate = rospy.Rate(1000)
  state = bs.STATE()
  ref = bs.STATE()

  [statuss, framesize] = s.get(state, wait=False, last=False)

  #for arm in range(0,bs.NUM_ARMS):
  #  for joint in range(0,bs.BAXTER_ARM_JOINTS_NUM):
  #    ref.arm[arm].joint[joint].ref = 0.0
  #    ref.arm[arm].joint[joint].pos = 0.0
  #    ref.arm[arm].joint[joint].tor = 0.0

  ref.arm[bs.RIGHT].joint[bs.WY2].ref = 3.0
  moveArm(ref, bs.RIGHT, right)
  state = getState(state,ref,left,right)
  r.put(ref)

  ref.arm[bs.LEFT].joint[bs.WY2].ref = 3.0
  moveArm(ref, bs.LEFT, left)
  state = getState(state,ref,left,right)
  r.put(ref)

  print left.joint_angle('left_w2')
  print state.arm[bs.LEFT].joint[bs.WY2].pos
  print state.arm[bs.LEFT].joint[bs.WY2].ref
  print right.joint_angle('right_e1')

  s.close()
  r.close()

if __name__ == '__main__':
  main()
