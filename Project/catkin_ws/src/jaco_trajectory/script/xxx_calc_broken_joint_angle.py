#!/usr/bin/env python  
import roslib; roslib.load_manifest('kinova_demo')
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('kinova_tf_listener')

    listener = tf.TransformListener()

    e2qrcode_trans_ = [0, 0, 0]
    e2qrcode_rot_ = [0, 0, 0, 0]
    while not rospy.is_shutdown():
        try:
            (e2qrcode_trans,e2qrcode_rot) = listener.lookupTransform('j2n6s300_end_effector', 'tag_3', rospy.Time(0))
            step_trans = [e2qrcode_trans[i] - e2qrcode_trans_[i] for i in range(len(e2qrcode_trans))]
            step_rot = [e2qrcode_rot[i] - e2qrcode_rot_[i] for i in range(len(e2qrcode_rot))]
            e2qrcode_trans_ = e2qrcode_trans
            e2qrcode_rot_ = e2qrcode_rot
            #print(type(e2qrcode_trans[0]))#3 float elem 1 list
            #print(type(e2qrcode_rot[0]))#4 float elem 1 list
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("did not find tag_3")
            continue

        #angular = 4 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #cmd = geometry_msgs.msg.Twist()
        #cmd.linear.x = linear
        #cmd.angular.z = angular
        #turtle_vel.publish(cmd)

        #rate.sleep()