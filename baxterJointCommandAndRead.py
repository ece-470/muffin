#!/usr/bin/env python
import argparse
import sys
import rospy
import baxter_interface as bi
import baxterStructure as bs

def moveArm(pos, arm, limb):
  if arm == bs.RIGHT:
     angles       = {'right_s0': (pos[bs.SY]+bs.offset_right[bs.SY])*bs.dir_right[bs.SY],
                     'right_s1': (pos[bs.SP]+bs.offset_right[bs.SP])*bs.dir_right[bs.SP], 
                     'right_w0': (pos[bs.WY]+bs.offset_right[bs.WY])*bs.dir_right[bs.WY], 
                     'right_w1': (pos[bs.WP]+bs.offset_right[bs.WP])*bs.dir_right[bs.WP], 
                     'right_w2': (pos[bs.WY2]+bs.offset_right[bs.WY2])*bs.dir_right[bs.WY2], 
                     'right_e0': (pos[bs.SR]+bs.offset_right[bs.SR])*bs.dir_right[bs.SR], 
                     'right_e1': (pos[bs.EP]+bs.offset_right[bs.EP])*bs.dir_right[bs.EP]}
     limb.move_to_joint_positions(angles)
  elif arm == bs.LEFT:
     angles       = {'left_s0': (pos[bs.SY]+bs.offset_left[bs.SY])*bs.dir_left[bs.SY],
                     'left_s1': (pos[bs.SP]+bs.offset_left[bs.SP])*bs.dir_left[bs.SP], 
                     'left_w0': (pos[bs.WY]+bs.offset_left[bs.WY])*bs.dir_left[bs.WY], 
                     'left_w1': (pos[bs.WP]+bs.offset_left[bs.WP])*bs.dir_left[bs.WP], 
                     'left_w2': (pos[bs.WY2]+bs.offset_left[bs.WY2])*bs.dir_left[bs.WY2], 
                     'left_e0': (pos[bs.SR]+bs.offset_left[bs.SR])*bs.dir_left[bs.SR], 
                     'left_e1': (pos[bs.EP]+bs.offset_left[bs.EP])*bs.dir_left[bs.EP]}
     limb.move_to_joint_positions(angles)

def main():
  print("init node")
  rospy.init_node("baxter_joint_pos_set")
  left  = bi.Limb('left')
  right = bi.Limb('right')
  rate = rospy.Rate(1000)
  left_arm = bs.BAXTER()
  right_arm = bs.BAXTER()

  pos = [0.0] * bs.BAXTER_ARM_JOINTS_NUM
#  pos[bs.SY] = 1.57
#  pos[bs.SP] = 0.0
#  pos[bs.SR] = 0.0
#  pos[bs.EP] = -1.57
#  pos[bs.WY] = 0.0
#  pos[bs.WP] = 0.0
#  pos[bs.WY2] = 0.0

  moveArm(pos, bs.LEFT, left)

#  pos[bs.SY] = -1.57
#  pos[bs.EP] = -1.57

  moveArm(pos, bs.RIGHT, right)



  print left.joint_angle('left_s0')
  print right.joint_angle('right_e1')

if __name__ == '__main__':
  main()
