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

def RotationMatrix_x(alpha):
	Rx = np.identity(4)
	Rx[1,1] = np.cos(alpha)
	Rx[1,2] = np.sin(alpha) * -1.0
	Rx[2,1] = np.sin(alpha)
	Rx[2,2] = np.cos(alpha)
	return Rx

def RotationMatrix_z(theta):
	Rz = np.identity(4)
	Rz[0,0] = np.cos(theta)
	Rz[0,1] = np.sin(theta) * -1.0
	Rz[1,0] = np.sin(theta)	
	Rz[1,1] = np.cos(theta)
	return Rz

def TranslationMatrix_z(d):
	Tz = np.identity(4)
	Tz[2,3] = d
	return Tz

def TranslationMatrix_x(a):
	Tx = np.identity(4)
	Tx[0,3] = a
	return Tx

def getFK(arm, theta):
	T = np.array([np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4))])
	if arm == bs.LEFT:
		lalpha = np.array([-np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2])
		ld = np.array([0.27, 0.0, 0.102+0.262, 0.0, 0.104+0.262, 0.0])
		la = np.array([0.69, 0.0, 0.69, 0.0, 0.01, 0.0])
		for i in range(6):
			if i == 1:
				Rotz = RotationMatrix_z(theta[i]+np.pi/2)
			else:
				Rotz = RotationMatrix_z(theta[i])
			Rotx = RotationMatrix_x(lalpha[i])
			Tranz = TranslationMatrix_z(ld[i])
			Tranx = TranslationMatrix_x(la[i])
			R = np.dot(np.dot(np.dot(Rotx,Tranx),Rotz),Tranz)
			T[i] = R
	elif arm == bs.RIGHT:
		ralpha = np.array([-np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2])
		rd = np.array([0.27, 0.0, 0.102+0.262, 0.0, 0.104+0.262, 0.0])
		ra = np.array([0.69, 0.0, 0.69, 0.0, 0.01, 0.0])
		for i in range(6):
			if i == 1:
				Rotz = RotationMatrix_z(theta[i]+np.pi/2)
			else:
				Rotz = RotationMatrix_z(theta[i])
			Rotx = RotationMatrix_x(ralpha[i])
			Tranz = TranslationMatrix_z(rd[i])
			Tranx = TranslationMatrix_x(ra[i])
			R = np.dot(np.dot(np.dot(Rotx,Tranx),Rotz),Tranz)
			T[i] = R

	T = np.dot(np.dot(np.dot(np.dot(np.dot(T[0],T[1]),T[2]),T[3]),T[4]),T[5])

	position = np.array([[round(T[0,3],3)],[round(T[1,3],3)],[round(T[2,3],3)]])

	return position

def getJ(arm, theta, dtheta):
	jac = np.zeros((3,6))
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
	DE = np.array([[round(dx,3)],[round(dy,3)],[round(dz,3)]])
	return DE

def getIK(arm, theta, G, ref, r, limb):
	dtheta = 0.01
	de = 15
	e = getFK(arm, theta)
	print 'bFK:', e
	tempTheta = np.copy(theta)
	met = getMet(e, G)
	print 'bmet', met
	tempMet = met
	while(met > 1):
		jac = getJ(arm, tempTheta, dtheta)
		jacInv = np.linalg.pinv(jac)
		DE = getNext(e, G, de, tempMet)
		Dtheta = np.dot(jacInv, DE)
		tempTheta = np.add(tempTheta, Dtheta)
		e = getFK(arm, tempTheta)
		print 'iFK:', e
		print 'arm:', arm
		met = getMet(e, G)

	if arm == bs.LEFT:
		ref.arm[bs.LEFT].joint[bs.SY].ref = tempTheta[0]
		ref.arm[bs.LEFT].joint[bs.SP].ref = tempTheta[1]
		ref.arm[bs.LEFT].joint[bs.WY].ref = tempTheta[2]
		ref.arm[bs.LEFT].joint[bs.WP].ref = tempTheta[3]
		ref.arm[bs.LEFT].joint[bs.SR].ref = tempTheta[4]
		ref.arm[bs.LEFT].joint[bs.EP].ref = tempTheta[5]
		moveArm(ref, arm, limb)
	elif arm == bs.RIGHT:
		ref.arm[bs.RIGHT].joint[bs.SY].ref = tempTheta[0]
		ref.arm[bs.RIGHT].joint[bs.SP].ref = tempTheta[1]
		ref.arm[bs.RIGHT].joint[bs.WY].ref = tempTheta[2]
		ref.arm[bs.RIGHT].joint[bs.WP].ref = tempTheta[3]
		ref.arm[bs.RIGHT].joint[bs.SR].ref = tempTheta[4]
		ref.arm[bs.RIGHT].joint[bs.EP].ref = tempTheta[5]
		moveArm(ref, arm, limb)

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

  lTheta = np.zeros((6,1))
  rTheta = np.zeros((6,1))

  lGoal = np.array([[0.0],[0.0],[0.0]])
  getIK(bs.LEFT, lTheta, lGoal, ref, r, left)

  rGoal = np.array([[0.0],[0.0],[0.0]])
  getIK(bs.RIGHT, rTheta, rGoal, ref, r, right)

#  ref.arm[bs.RIGHT].joint[bs.WY2].ref = 3.0
#  moveArm(ref, bs.RIGHT, right)
#  state = getState(state,ref,left,right)
#  r.put(ref)

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
