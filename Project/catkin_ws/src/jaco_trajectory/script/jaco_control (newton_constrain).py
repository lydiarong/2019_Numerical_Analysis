#! /usr/bin/env python
"""Takes trajectory data from xyz and uses inverse kinematics algorithms to calculate the best state. """
#rosrun jaco_trajectory jaco_control.py
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import numpy
from scipy.linalg import pinv
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi
from math import cos
from math import sin
from sympy import *
from robot_functions import *   # Built functions for jaco model
from time import strftime, sleep

def moveJoint (jointcmds):
  topic_name = '/' + 'j2n6s300' + '/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, 6):
	jointCmd.joint_names.append('j2n6s300' +'_joint_'+str(i+1))
	point.positions.append(jointcmds[i])
	point.velocities.append(0)
	point.accelerations.append(0)
	point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
	pub.publish(jointCmd)
	count = count + 1
	rate.sleep()     

def moveFingers (jointcmds):
  topic_name = '/' + 'j2n6s300' + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, 3):
	jointCmd.joint_names.append('j2n6s300' +'_joint_finger_'+str(i+1))
	point.positions.append(jointcmds[i])
	point.velocities.append(0)
	point.accelerations.append(0)
	point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
	pub.publish(jointCmd)
	count = count + 1
	rate.sleep()

if __name__ == '__main__':
  try:
	ti = time.time()
	rospy.init_node('move_robot_using_trajectory_msg')

	# Allow gazebo to launch
	rospy.sleep(1)

	######################### EDIT THIS SECTION ###############################
	# Parameters
	numPts = 2      # Discretized path (Choose number of points)
	step   = 0.1    # Step size 
	error  = 0.000001  # Error between state and desired state (theshold value, in meters)

	# Initial state
	q10 = 0.0
	q20 = 2.9
	q30 = 1.3
	q40 = 4.2
	q50 = 1.4
	q60 = 0.0

	# Final state
	xf = -0.2
	yf = 0.6
	zf = 0.4
	###########################################################################


	# Boundary conditions (DON'T WORRY ABOUT THESE, and DON'T UNCOMMENT)
	init_q  = Matrix([[-q10], [q20-pi/2], [q30+pi/2], [q40], [q50-pi], [q60+pi/2]])
	
	# Initial position
	_, init_X = systemKinematics(init_q)
	final_X = Matrix([[xf],[yf],[zf]])
	


	######################### EDIT THIS SECTION ###############################
	# Discretize path (change this to change path shape, default is line)
	xPath = numpy.linspace(float(init_X[0]), float(final_X[0]), num=numPts)
	yPath = numpy.linspace(float(init_X[1]), float(final_X[1]), num=numPts)
	zPath = numpy.linspace(float(init_X[2]), float(final_X[2]), num=numPts)
	###########################################################################

	
	# Move robot to initial state
	moveJoint([q10,q20,q30,q40,q50,q60])
	moveFingers([1.3,1.3,1.3])

	# Initialize variables
	delta_X = Matrix([[1000],[1000],[1000]])
	
	X = []
	Y = []
	Z = []

	# Matrix calculatations
	dynMatrix,_ = systemKinematics(init_q)
	row0 = dynMatrix.row(0)
	row1 = dynMatrix.row(1)
	row2 = dynMatrix.row(2)
	DynMatrix = Matrix([[row0],[row1],[row2]])
	J_x = robotJacobian(DynMatrix[0])
	J_y = robotJacobian(DynMatrix[1])
	J_z = robotJacobian(DynMatrix[2])

	currentTime = strftime('%m_%d_%H_%M_%S')
	Output = open ('Output-' + currentTime, 'w')
	Output.write(str(init_q))
	Output.write('Initial q state:'+str(init_q)+'\n')
	Output.write('Initial x state:'+str(init_X)+'\n')
	Output.flush()
	# Connect all points in mesh path (START LOOP AT 1, NOT 0)
	for i in range(1,numPts):
		path_X = Matrix([[xPath[i]]])
		path_Y = Matrix([[yPath[i]]])
		path_Z = Matrix([[zPath[i]]])
		print(i)
		# print "init_q:"
		# print init_q
		# print init_X
		_, init_X = systemKinematics(init_q)
		delta_X = path_X - Matrix([[init_X[0]]])
		delta_Y = path_Y - Matrix([[init_X[1]]])
		delta_Z = path_Z - Matrix([[init_X[2]]])

		F=(path_X-DynMatrix[0])**2
		HJ_x = robotJacobian(F)
		H_x = robotHessian(F)
		print H_x.shape ,HJ_x.shape

		G=((Matrix([[init_X[1]]])-DynMatrix[1]))
		G_J=robotJacobian(G)
		G_H=robotHessian(G)
		print G_J.shape ,G_H.shape
		# print evalJacobian(G,init_q)

		lamba=symbols('lamba')
		L=F+G*lamba
		L_J=robotJacobian(L)
		temp=Matrix(H_x+lamba.transpose()*G_H)
		Z1=Matrix(H_x+lamba.transpose()*G_H).col_insert(7,G_J.transpose())
		Z2=Matrix(G_J).col_insert(7,zeros(1,1))
		Z3=Z1.row_insert(7,Z2)

		B1=Matrix([-L_J.transpose()]).row_insert(7,Matrix([-G]))
		print L_J.shape ,Z3.shape,B1.shape
		

		delta_v,delta_v1,delta_v2,delta_v3,delta_v4,delta_v5,delta_v6=symbols('delta_v delta_v1 delta_v2 delta_v3 delta_v4 delta_v5 delta_v6')
		delta_v=Matrix([[delta_v1],[delta_v2],[delta_v3],[delta_v4],[delta_v5],[delta_v6]])
		cond=abs(HJ_x*delta_v)+abs(lamba*G)

		itertion = 0
		new_lamba=0.1
		new_delta_v=Matrix([[0.1],[0.1],[0.1],[0.1],[0.1],[0.1]])

		print evalCond1(Z3,init_q,new_lamba),evalCond1(temp,init_q,new_lamba)

		# print evalCond(cond,init_q,new_lamba,new_delta_v)[0]
		
		while evalCond(cond,init_q,new_lamba,new_delta_v)[0]>error:
			print evalCond(cond,init_q,new_lamba,new_delta_v)[0]
			if evalCond(cond,init_q,new_lamba,new_delta_v)[0]>error:

				_, init_X = systemKinematics(init_q)
				delta_X = path_X - Matrix([[init_X[0]]])#,[init_X[1]],[init_X[2]]
				# print(Abs(delta_X.norm()))
				useful_J = evalCond1(Z3,init_q,new_lamba)
				useful_H = evalCond1(B1,init_q,new_lamba)

				KKT=inverseKinematics_type(useful_J,step,useful_H)
				print KKT.shape
				new_delta_v=Matrix(KKT[0:6])
				new_lamba=new_lamba+step*(KKT[6]-new_lamba)
				print init_q.shape,new_delta_v.shape
				init_q = init_q + step*new_delta_v


			# if (Abs(delta_X.norm()) > error):

			# 	_, init_X = systemKinematics(init_q)
			# 	delta_X = path_X - Matrix([[init_X[0]]])#,[init_X[1]],[init_X[2]]
			# 	# print(Abs(delta_X.norm()))

			# 	# Linearize dynamics matrix
			# 	useful_J = evalJacobian(HJ_x,init_q)
			# 	useful_H = evalHessian(H_x,init_q)

			# 	# Calculate joint state change and new state
			# 	# delta_q = inverseKinematics_type(useful_J,step,delta_X)
			# 	# init_q = init_q + step*delta_q

			# 	delta_q=inverseKinematics_newton(useful_H,step,useful_J)
			# 	init_q = init_q - step*delta_q
					

			# 	Convert back from DH angles to JACO angles
			# 	moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])

			# if (Abs(delta_X.norm()) > error):
			# 	_, init_X = systemKinematics(init_q)
			# 	delta_X = path_X - Matrix([[init_X[0]]])#,[init_X[1]],[init_X[2]]
			# 	# print(Abs(delta_X.norm()))

			# 	# Linearize dynamics matrix
			# 	useful_J = evalJacobian(J_x,init_q)
			# 	useful_H = evalHessian(H_x,init_q)

			# 	# Calculate joint state change and new state
			# 	delta_q = inverseKinematics_type(useful_J,step,delta_X)
			# 	init_q = init_q + step*delta_q

			# 	if (Abs(delta_Y.norm()) > error):
			# 	_, init_X = systemKinematics(init_q)
			# 	delta_Y = path_Y - Matrix([[init_X[1]]])#,[init_X[1]],[init_X[2]]
			# 	# print(Abs(delta_X.norm()))

			# 	# Linearize dynamics matrix
			# 	useful_J = evalJacobian(J_y,init_q)

			# 	# Calculate joint state change and new state
			# 	delta_q = inverseKinematics_type(useful_J,step,delta_Y)
			# 	init_q = init_q + step*delta_q
			# 	Convert back from DH angles to JACO angles
			# 	moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])

			# 	if (Abs(delta_Z.norm()) > error):
			# 	_, init_X = systemKinematics(init_q)
			# 	delta_Z = path_Z - Matrix([[init_X[2]]])#,[init_X[1]],[init_X[2]]
			# 	# print(Abs(delta_X.norm()))

			# 	# Linearize dynamics matrix
			# 	useful_J = evalJacobian(J_z,init_q)

			# 	# Calculate joint state change and new state
			# 	delta_q = inverseKinematics_type(useful_J,step,delta_Z)
			# 	init_q = init_q + step*delta_q
			# 	Convert back from DH angles to JACO angles
			# 	moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])

			# # Convert back from DH angles to JACO angles
			moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])
			print init_X
			itertion = itertion + 1
			# Trajectory plotting
			X.append(init_X[0])
			Y.append(init_X[1])
			Z.append(init_X[2])

	Output.write('Final X:'+str(X)+'\n')
	Output.write('Final Y:'+str(Y)+'\n')
	Output.write('Final Z:'+str(Z)+'\n')
	Output.write('Final q state:'+str(init_q)+'\n')
	Output.write('Final x state:'+str(init_X)+'\n')
	elapsed = time.time() - ti
	Output.write('time:'+str(elapsed)+'\n')
	Output.write('itertion:'+str(itertion)+'\n')
	Output.flush()
	print("time",elapsed)
	print("itertion", itertion)
	print init_q
	fig = plt.figure()
	ax = fig.gca(projection='3d')    
	ax.plot(X, Y, Z, label='traveled')
	ax.plot(xPath, yPath, zPath, label='path')
	ax.legend()
	plt.show()
	

  except rospy.ROSInterruptException:
	print ("program interrupted before completion")
