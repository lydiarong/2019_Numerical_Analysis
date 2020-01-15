#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import time

import math
import argparse
import  numpy
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'j2n6s300_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
pi=math.pi

# print "new"
def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    global prefix
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    print "position", position
    print "orientation",orientation

    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # goal.pose.pose.position = geometry_msgs.msg.Point(
    #     x=0.4, y= -0.3 , z=0.5)
    # goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
    #     x=1.6, y=1.1, z=0.1, w=orientation[3])

    print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug
    client.send_goal(goal)

    # print "newone"
    # goal.pose.pose.position = geometry_msgs.msg.Point(
    #     x=0.5, y=0.5, z=0.5)
    # goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
    #     x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
    # client.send_goal(goal)
    #
    # print "newone2"
    # goal.pose.pose.position = geometry_msgs.msg.Point(
    #     x=0.5, y=-0.5, z=0.5)
    # goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
    #     x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
    # client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None
def joint_angle_client(angle_set):#add_argument
    """Send a joint angle goal to the action server."""
    global prefix
    action_address = '/' + prefix + 'driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]*180/math.pi
    goal.angles.joint2 = angle_set[1]*180/math.pi
    goal.angles.joint3 = angle_set[2]*180/math.pi
    goal.angles.joint4 = angle_set[3]*180/math.pi
    goal.angles.joint5 = angle_set[4]*180/math.pi
    goal.angles.joint6 = angle_set[5]*180/math.pi

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(30.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None



def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_



def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    global prefix
    topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6s300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='mq',
                        choices={'mq', 'mdeg', 'mrad'},
                        help='Unit of Cartesian pose command, in mq(Position meter, Orientation Quaternion),  mdeg(Position meter, Orientation Euler-XYZ in degree), mrad(Position meter, Orientation Euler-XYZ in radian)]')
    parser.add_argument('pose_value', nargs='*', type=float, help='Cartesian pose values: first three values for position, and last three(unit mdeg or mrad)/four(unit mq) for Orientation')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display Cartesian pose values in alternative convention(mq, mdeg or mrad)')
    parser.add_argument('--broken_angle',help='put the calc broken angle here',type=float)
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    # print('pose_mq in argumentParser 1: {}'.format(args_.pose_value))  # debug
    return args_


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, pose_value_, relative_):
    """ Argument unit """
    global currentCartesianCommand

    position_ = pose_value_[:3]
    orientation_ = pose_value_[3:]

    for i in range(0,3):
        if relative_:
            position_[i] = pose_value_[i] + currentCartesianCommand[i]
        else:
            position_[i] = pose_value_[i]

    # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

    if unit_ == 'mq':
        if relative_:
            orientation_XYZ = Quaternion2EulerXYZ(orientation_)
            orientation_xyz_list = [orientation_XYZ[i] + currentCartesianCommand[3+i] for i in range(0,3)]
            orientation_q = EulerXYZ2Quaternion(orientation_xyz_list)
        else:
            orientation_q = orientation_

        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))

    elif unit_ == 'mdeg':
        if relative_:
            orientation_deg_list = list(map(math.degrees, currentCartesianCommand[3:]))
            orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
        else:
            orientation_deg = orientation_

        orientation_rad = list(map(math.radians, orientation_deg))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    elif unit_ == 'mrad':
        if relative_:
            orientation_rad_list =  currentCartesianCommand[3:]
            orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
        else:
            orientation_rad = orientation_

        orientation_deg = list(map(math.degrees, orientation_rad))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    else:
        raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

    pose_mq_ = position_ + orientation_q
    pose_mdeg_ = position_ + orientation_deg
    pose_mrad_ = position_ + orientation_rad

    # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

    return pose_mq_, pose_mdeg_, pose_mrad_


def verboseParser(verbose, pose_mq_):
    """ Argument verbose """
    position_ = pose_mq_[:3]
    orientation_q = pose_mq_[3:]
    if verbose:
        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))
        print('Cartesian position is: {}'.format(position_))
        print('Cartesian orientation in Quaternion is: ')
        print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]))
        print('Cartesian orientation in Euler-XYZ(radian) is: ')
        print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
        print('Cartesian orientation in Euler-XYZ(degree) is: ')
        print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))

#some kinematics and IK functions
def systemKinematics(state):
    # Symbolize state variables
    q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

    # Define geometric parameters for j2n6s300
    D1 = 0.2755
    D2 = 0.4100
    D3 = 0.2073
    D4 = 0.0741
    D5 = 0.0741
    D6 = 0.1600
    e2 = 0.0098

    aa = 30.0*pi/180.0
    ca = cos(aa)
    sa = sin(aa)
    c2a = cos(2*aa)
    s2a = sin(2*aa)
    d4b = D3 + sa/s2a*D4
    d5b = sa/s2a*D4 + sa/s2a*D5
    d6b = sa/s2a*D5 + D6

    # Define DH parameters for all joints
    alpha = numpy.array([pi/2, pi, pi/2, 2*aa, 2*aa, pi])
    a = numpy.array([0, D2, 0, 0, 0, 0])
    d = numpy.array([D1, 0, -e2, -d4b, -d5b, -d6b])

    Q1 = DHMatrix(d[0],q1,a[0],alpha[0])
    Q2 = DHMatrix(d[1],q2,a[1],alpha[1])
    Q3 = DHMatrix(d[2],q3,a[2],alpha[2])
    Q4 = DHMatrix(d[3],q4,a[3],alpha[3])
    Q5 = DHMatrix(d[4],q5,a[4],alpha[4])
    Q6 = DHMatrix(d[5],q6,a[5],alpha[5])

    T = Q1 * Q2 * Q3 * Q4 * Q5 * Q6
    base_state = Matrix([[0],[0],[0],[1]])
    dyn = T * base_state
    init_x = dyn.subs(q1,state[0])
    # print "1",init_q
    init_x = init_x.subs(q2,state[1])
    # print "2",init_q
    init_x = init_x.subs(q3,state[2])
    # print "3",init_q
    init_x = init_x.subs(q4,state[3])
    init_x = init_x.subs(q5,state[4])
    init_x = init_x.subs(q6,state[5])
    return dyn, init_x.evalf()

def DHMatrix(d,theta,a,alpha):
    d_mat = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
    theta_mat = Matrix([[cos(theta),-sin(theta),0,0],[sin(theta),cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    a_mat = Matrix([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    alpha_mat = Matrix([[1,0,0,0],[0,cos(alpha),-sin(alpha),0],[0,sin(alpha),cos(alpha),0],[0,0,0,1]])
    DH_mat = d_mat * theta_mat * a_mat * alpha_mat
    return DH_mat

def robotJacobian(DH_mat):
    # Symbolize state variables
    q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')
    q = Matrix([[q1],[q2],[q3],[q4],[q5],[q6]])
    DH_mat_lin = DH_mat.jacobian(q)
    return DH_mat_lin

def evalJacobian(J,state):
    q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')
    J = J.subs(q1,state[0])
    J = J.subs(q2,state[1])
    J = J.subs(q3,state[2])
    J = J.subs(q4,state[3])
    J = J.subs(q5,state[4])
    J = J.subs(q6,state[5])
    return J.evalf()

def Sym2NumArray(F):
    #Function to convert symbolic expression with numerical data to numpy array
    shapeF=numpy.shape(F)
    B=zeros(shapeF[0],shapeF[1])
    for i in range(0,shapeF[0]):
        for j in range(0,shapeF[1]):
            B[i,j]=N(F[i,j])
    return B

def inverseKinematics(J,step,delta_X):
    J = Sym2NumArray(J)				# Convert to numpy.array
    J = numpy.array(J).astype(numpy.float64)	# Convert to numpy.matrix
    Jplus = J.transpose()				# Calculate transpose
    Jplus = numpy.linalg.pinv(J)				# Calculate pseudoinverse
    delta_q = Jplus * delta_X			# Calculate change in joint space
    return delta_q