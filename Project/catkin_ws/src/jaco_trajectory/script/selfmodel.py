#! /usr/bin/env python
"""Takes trajectory data from xyz and uses inverse kinematics algorithms to calculate the best state. """

from robot_functions import *  # Built functions for jaco model

""" Global variable """
arm_joint_number = 0
finger_number = 0
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
pi=math.pi

if __name__ == '__main__':

    args = argumentParser(None)
    print 'args', args.broken_angle
    kinova_robotTypeParser(args.kinova_robotType)
    rospy.init_node(prefix + 'pose_action_client')

    # if args.unit == 'mq':
    #     if len(args.pose_value) != 7:
    #         print('Number of input values {} is not equal to 7 (3 position + 4 Quaternion).'.format(len(args.pose_value)))
    #         sys.exit(0)
    # elif (args.unit == 'mrad') | (args.unit == 'mdeg'):
    #     if len(args.pose_value) != 6:
    #         print('Number of input values {} is not equal to 6(3 position + 3 EulerAngles).'.format(len(args.pose_value)))
    #         sys.exit(0)
    # else:
    #     raise Exception('Cartesian value have to be in unit: mq, mdeg or mrad')

    getcurrentCartesianCommand(prefix)
    # print "prefix",prefix
    # pose_mq, pose_mdeg, pose_mrad = unitParser(args.unit, args.pose_value, args.relative)

    try:
        ti = time.time()
        # rospy.init_node('move_robot_using_trajectory_msg')

        # Allow gazebo to launch
        # rospy.sleep(1)

        ######################### EDIT THIS SECTION ###############################
        # Parameters
        numPts = 5		# Discretized path (Choose number of points)
        step = 0.5  # Step size
        # Error between state and desired state (theshold value, in meters)
        error = 0.001

        # Initial state
        q10 = -0.5
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
        init_q = Matrix([[-q10], [q20-pi/2], [q30+pi/2],
                         [q40], [q50-pi], [q60+pi/2]])

        # Initial position
        _, init_X = systemKinematics(init_q)
        final_X = Matrix([[xf], [yf], [zf]])

        ######################### EDIT THIS SECTION ###############################
        # Discretize path (change this to change path shape, default is line)
        xPath = numpy.linspace(float(init_X[0]), float(final_X[0]), num=numPts)
        yPath = numpy.linspace(float(init_X[1]), float(final_X[1]), num=numPts)
        zPath = numpy.linspace(float(init_X[2]), float(final_X[2]), num=numPts)
        ###########################################################################

        # print "11"
        # Move robot to initial state
        # moveJoint([q10,q20,q30,q40,q50,q60])
        joint_angle_client([q10, q20, q30, q40, q50, q60])

        # moveFingers([1.3,1.3,1.3])
        # print "22"
        # Initialize variables
        delta_X = Matrix([[1000], [1000], [1000]])

        X = []
        Y = []
        Z = []

        trajectory = []
        # Matrix calculatations
        dynMatrix, _ = systemKinematics(init_q)
        row0 = dynMatrix.row(0)
        row1 = dynMatrix.row(1)
        row2 = dynMatrix.row(2)
        DynMatrix = Matrix([[row0], [row1], [row2]])
        J = robotJacobian(DynMatrix)
        #print "J", J
        print "working!"
        print "computing1..."
        # Connect all points in mesh path (START LOOP AT 1, NOT 0)
        for i in range(1, numPts):
            path_X = Matrix([[xPath[i]], [yPath[i]], [zPath[i]]])
            # print(i)
            _, init_X = systemKinematics(init_q)
            delta_X = path_X - Matrix([[init_X[0]], [init_X[1]], [init_X[2]]])
            iteration = 0
            while (Abs(delta_X.norm()) > error):
                # print "iteration",iteration,init_q
                iteration += 1
                _, init_X = systemKinematics(init_q)
                delta_X = path_X - \
                    Matrix([[init_X[0]], [init_X[1]], [init_X[2]]])
                # print("Abs(delta_X.norm())",Abs(delta_X.norm()))
                print "in iteration", init_X
                # Linearize dynamics matrix
                useful_J = evalJacobian(J, init_q)

                # Calculate joint state change and new state
                delta_q = inverseKinematics(useful_J, step, delta_X)
                init_q = init_q + step*delta_q
                trajectory.append(init_q)
                X.append(init_X[0])
                Y.append(init_X[1])
                Z.append(init_X[2])
        print 'pose sent!'
        for q in trajectory:
            joint_angle_client(
                [-q[0], q[1]+pi/2, q[2]-pi/2, q[3], q[4]+pi, q[5]-pi/2])

        # joint_angle_client([-init_q[0], init_q[1]+pi/2, init_q[2]-pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2])
        print "joints", [-init_q[0], init_q[1]+pi/2, init_q[2] -
                         pi/2, init_q[3], init_q[4]+pi, init_q[5]-pi/2]
        print "cartesian", init_X
        elapsed = time.time() - ti
        print "elapsed", elapsed
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(X, Y, Z, label='traveled_normal')
        ax.plot(xPath, yPath, zPath, label='path_normal')
        ax.legend()
        plt.show()
    except rospy.ROSInterruptException:
      print "program interrupted before completion"
