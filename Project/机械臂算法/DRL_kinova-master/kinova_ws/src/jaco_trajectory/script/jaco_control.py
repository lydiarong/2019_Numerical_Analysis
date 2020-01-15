#! /usr/bin/env python
"""Takes trajectory data from xyz and uses inverse kinematics algorithms to calculate the best state. """

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
from robot_functions import *	# Built functions for jaco model

if __name__ == '__main__':
  try:
    ti = time.time()
    rospy.init_node('move_robot_using_trajectory_msg')

    # Allow gazebo to launch
    rospy.sleep(1)

    ######################### EDIT THIS SECTION ###############################
    # Parameters
    numPts = 10		# Discretized path (Choose number of points)
    step   = 0.5	# Step size 
    error  = 0.001	# Error between state and desired state (theshold value, in meters)

    # Initial state
    q10 = 0.0
    q20 = 2.9
    q30 = 1.3
    q40 = 4.2
    q50 = 1.4
    q60 = 0.0

    # Final state
    xf = 0.2
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
    xPath = numpy.linspace(init_X[0], final_X[0], num=numPts)
    yPath = numpy.linspace(init_X[1], final_X[1], num=numPts)
    zPath = numpy.linspace(init_X[2], final_X[2], num=numPts)
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
    J = robotJacobian(DynMatrix)

    # Connect all points in mesh path (START LOOP AT 1, NOT 0)
    for i in range(1,numPts):
      path_X = Matrix([[xPath[i]],[yPath[i]],[zPath[i]]])
      print(i)
      _, init_X = systemKinematics(init_q)
      delta_X = path_X - Matrix([[init_X[0]],[init_X[1]],[init_X[2]]])
      
      while (Abs(delta_X.norm()) > error):
        _, init_X = systemKinematics(init_q)
        delta_X = path_X - Matrix([[init_X[0]],[init_X[1]],[init_X[2]]])
        print(Abs(delta_X.norm()))

        # Linearize dynamics matrix
        useful_J = evalJacobian(J,init_q)

        # Calculate joint state change and new state
        delta_q = inverseKinematics(useful_J,step,delta_X)
        init_q = init_q + step*delta_q

        # Convert back from DH angles to JACO angles
        moveJoint([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])
        
        # Trajectory plotting
        X.append(init_X[0])
        Y.append(init_X[1])
        Z.append(init_X[2])
    
    elapsed = time.time() - ti
    print(elapsed)

    fig = plt.figure()
    ax = fig.gca(projection='3d')    
    ax.plot(X, Y, Z, label='traveled')
    ax.plot(xPath, yPath, zPath, label='path')
    ax.legend()
    plt.show()
    

  except rospy.ROSInterruptException:
    print ("program interrupted before completion")
