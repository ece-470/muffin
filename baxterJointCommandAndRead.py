#!/usr/bin/env python
import argparse
import sys
import rospy
import ach
import baxter_interface as bi
import baxterStructure as bs
import numpy as np
import math

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

def simSleep(T):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	tick = state.time
	while((state.time - tick) < T):
		[statuss, framesizes] = s.get(state, wait=True, last=False)

def RotationMatrix(theta):
	Rx = np.identity(4)
	Rx[1,1] = np.cos(theta)
	Rx[1,2] = np.sin(theta) * -1.0
	Rx[2,1] = np.sin(theta)
	Rx[2,2] = np.cos(theta)

	Ry = np.identity(4)
	Ry[0,0] = np.cos(theta)
	Ry[0,2] = np.sin(theta)
	Ry[2,0] = np.sin(theta) * -1.0
	Ry[2,2] = np.cos(theta)

	Rz = np.identity(4)
	Rz[0,0] = np.cos(theta)
	Rz[0,1] = np.sin(theta) * -1.0
	Rz[1,0] = np.sin(theta)	
	Rz[1,1] = np.cos(theta)

	return np.dot(np.dot(Rx,Ry),Rz)

def getFK(arm, theta):
	T1 = np.identity(4)
	if(arm == 'LEFT'):
		T1[1,3] = 94.5
	elif(arm == 'RIGHT'):
		T1[1,3] = -94.5
	T2 = np.identity(4)
	T3 = np.identity(4)
	T4 = np.identity(4)
	T4[2,3] = -179.14
	T5 = np.identity(4)
	T5[2,3] = -181.59
	T6 = np.identity(4)
	T7 = np.identity(4)

	Q1 = np.dot(RotationMatrix(theta[0,0]),T1)
	Q2 = np.dot(RotationMatrix(theta[1,0]),T2)
	Q3 = np.dot(RotationMatrix(theta[2,0]),T3)
	Q4 = np.dot(RotationMatrix(theta[3,0]),T4)
	Q5 = np.dot(RotationMatrix(theta[4,0]),T5)
	Q6 = np.dot(RotationMatrix(theta[5,0]),T6)
	Q7 = np.dot(RotationMatrix(theta[6,0]),T7)

	Q = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(Q1,Q2),Q3),Q4),Q5),Q6),Q7)

	position = np.transpose([Q[0,3],Q[1,3],Q[2,3]])

	return position

def getJ(arm, theta, dtheta):
	jac = np.zeros((3,7))
	for i in range((np.shape(jac))[0]):
		for j in range((np.shape(jac))[1]):
			tempTheta = np.copy(theta)
			tempTheta[j] = theta[j] + dtheta
			fk = getFK(arm, tempTheta)
			jac[i,j] = (fk[i,0]) / dtheta
	return jac

def getMet(e, G):
	met = math.sqrt(math.pow(e[0] - G[0],2) + math.pow(e[1] - G[1],2) + math.pow(e[2] - G[2],2))
	return met

def getNext(e, G, de, h):
	dx = (G[0] - e[0]) * de / h
	dy = (G[1] - e[1]) * de / h
	dz = (G[2] - e[2]) * de / h
	DE = np.transpose([dx,dy,dz])
	return DE

def getIK(arm, theta, G, ref, r):
	dtheta = 0.01
	de = 15
	e = getFK(arm, theta)
	tempTheta = np.copy(theta)
	met = getMet(e, G)
	tempMet = met
	while(met > 5):
		jac = getJ(arm, tempTheta, dtheta)
		jacInv = np.linalg.pinv(jac)
		DE = getNext(e, G, de, tempMet)
		Dtheta = np.dot(jacInv, DE)
		tempTheta = np.add(tempTheta, Dtheta)
		e = getFK(arm, tempTheta)
		met = getMet(e, G)

	if(arm == 'LEFT'):
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[0]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[1]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[2]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[3]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[4]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[5]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[6]
	elif(arm == 'RIGHT'):
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[0]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[1]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[2]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[3]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[4]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[5]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[6]

	r.put(ref)

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

  lTheta = np.zeros((7,1))
  rTheta = np.zeros((7,1))

  lGoal = np.array([[10.0],[10.0],[10.0]])
  getIK(bs.LEFT, lTheta, lGoal, ref, r)




#  ref.arm[bs.RIGHT].joint[bs.WY2].ref = 3.0
#  moveArm(ref, bs.RIGHT, right)
#  state = getState(state,ref,left,right)
#  r.put(ref)
#
#  ref.arm[bs.LEFT].joint[bs.WY2].ref = 3.0
#  moveArm(ref, bs.LEFT, left)
#  state = getState(state,ref,left,right)
#  r.put(ref)
#
#  print left.joint_angle('left_w2')
#  print state.arm[bs.LEFT].joint[bs.WY2].pos
#  print state.arm[bs.LEFT].joint[bs.WY2].ref
#  print right.joint_angle('right_e1')

  s.close()
  r.close()

if __name__ == '__main__':
  main()
