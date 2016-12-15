#!/usr/bin/env python
import argparse
import sys
import rospy
import ach
import baxter_interface as bi
import baxterStructure as bs
import numpy as np
import math
import time

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

def DHMatrix(alpha,a,d,theta):
	T=np.array([
		[math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta)],
		[math.sin(theta),math.cos(theta)*math.cos(alpha),-math.cos(theta)*math.sin(alpha),a*math.sin(theta)],
		[0,math.sin(alpha),math.cos(alpha),d],
		[0,0,0,1],
		])
	#print "DHMatrix"
	#print "T:",T
	return T

def getFK(arm, theta):
	T = np.array([np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4))])

	DH = np.array([
		[-np.pi/2,0.069,0.27035,theta[0]],
		[np.pi/2,0.0,0.0,theta[1]+np.pi/2],
		[-np.pi/2,0.069,0.36435,theta[2]],
		[np.pi/2,0.0,0.0,theta[3]],
		[-np.pi/2,0.01,0.37429,theta[4]],
		[np.pi/2,0,0,theta[5]],
		[0.0, 0.0, 0.229525,theta[6]],
		])

	for i in range(7):
		T[i] = DHMatrix(DH[i][0],DH[i][1],DH[i][2],DH[i][3])

	T = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T[0],T[1]),T[2]),T[3]),T[4]),T[5]),T[6])

	position = np.array([[round(T[0,3],3)],[round(T[1,3],3)],[round(T[2,3],3)]])
	#print "getFK"
	#print "arm",arm
	#print "theta:",theta
	#print "position",position
	return position

def getJ(arm, theta, dtheta):
	jac = np.zeros((3,7))
	for i in range((np.shape(jac))[0]):
		for j in range((np.shape(jac))[1]):
			tempTheta = np.copy(theta)
			tempTheta[j] = theta[j] + dtheta
			fk = getFK(arm, tempTheta)
			jac[i,j] = (fk[i,0]) / dtheta
	#print "getJ"
	#print "arm:",arm
	#print "theta:",theta
	#print "dtheta:",dtheta
	#print "jac:",jac
	return jac

def getMet(e, G):
	met = math.sqrt(math.pow(e[0] - G[0],2) + math.pow(e[1] - G[1],2) + math.pow(e[2] - G[2],2))
	#print "getMet"
	#print "End: ",e
	#print "Goal: ",G
	#print "Distance: ",met
	return met

def getNext(e, G, de, h):
	dx = (G[0] - e[0]) * de / h
	dy = (G[1] - e[1]) * de / h
	dz = (G[2] - e[2]) * de / h
	DE = np.array([[round(dx,3)],[round(dy,3)],[round(dz,3)]]) 
	#print "getNext"
	#print "End:",e
	#print "Goal:",G
	#print "de:",de
	#print "Distance:",h
	#print "DE:",DE
	return DE

def getIK(arm, theta, G, ref, r, limb):
	dtheta = 0.05
	de = 10.0
	e = getFK(arm, theta)
	tempTheta = np.copy(theta)
	met = getMet(e, G)
	t1 = time.time()
	while(met > 0.05):
		jac = getJ(arm, tempTheta, dtheta)
		jacInv = np.linalg.pinv(jac)
		DE = getNext(e, G, de, met)
		Dtheta = np.dot(jacInv, DE)
		tempTheta = np.add(tempTheta, Dtheta)
		e = getFK(arm, tempTheta)
		met = getMet(e, G)
		if(time.time() - t1 > 15):
			print "Stuck?"
			break

	if arm == bs.LEFT:
		ref.arm[bs.LEFT].joint[bs.SY].ref = tempTheta[0]
		ref.arm[bs.LEFT].joint[bs.SP].ref = tempTheta[1]
		ref.arm[bs.LEFT].joint[bs.SR].ref = tempTheta[2]
		ref.arm[bs.LEFT].joint[bs.EP].ref = tempTheta[3]
		ref.arm[bs.LEFT].joint[bs.WY].ref = tempTheta[4]
		ref.arm[bs.LEFT].joint[bs.WP].ref = tempTheta[5]
		ref.arm[bs.LEFT].joint[bs.WY2].ref = tempTheta[6]
	elif arm == bs.RIGHT:
		ref.arm[bs.RIGHT].joint[bs.SY].ref = tempTheta[0]
		ref.arm[bs.RIGHT].joint[bs.SP].ref = tempTheta[1]
		ref.arm[bs.RIGHT].joint[bs.SR].ref = tempTheta[2]
		ref.arm[bs.RIGHT].joint[bs.EP].ref = tempTheta[3]
		ref.arm[bs.RIGHT].joint[bs.WY].ref = tempTheta[4]
		ref.arm[bs.RIGHT].joint[bs.WP].ref = tempTheta[5]
		ref.arm[bs.RIGHT].joint[bs.WY2].ref = tempTheta[6]

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

  lTheta = np.zeros((7,1))
  rTheta = np.zeros((7,1))

  rTheta[0,0] = 0.0
  rTheta[1,0] = 0.0
  rTheta[2,0] = 0.0
  rTheta[3,0] = 0.0
  rTheta[4,0] = 0.0
  rTheta[5,0] = 0.0
  rTheta[6,0] = 0.0
  print 'RIGHT FK', getFK(bs.RIGHT, rTheta)

  lTheta[0,0] = 0.0
  lTheta[1,0] = 0.0
  lTheta[2,0] = 0.0
  lTheta[3,0] = 0.0
  lTheta[4,0] = 0.0
  lTheta[5,0] = 0.0
  lTheta[6,0] = 0.0
  print 'LEFT FK', getFK(bs.LEFT, lTheta)

  rx = float(raw_input("rX: "))
  ry = float(raw_input("rY: "))
  rz = float(raw_input("rZ: "))
  rGoal = np.array([[rx],[ry],[rz]])
  getIK(bs.RIGHT, rTheta, rGoal, ref, r, right)

  lx = float(raw_input("lX: "))
  ly = float(raw_input("lY: "))
  lz = float(raw_input("lZ: "))
  lGoal = np.array([[lx],[ly],[lz]])
  getIK(bs.LEFT, lTheta, lGoal, ref, r, left)

#  ref.arm[bs.RIGHT].joint[bs.WY12].ref = 3.0
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
