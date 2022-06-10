#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import sensor_msgs.msg
import numpy


rospy.init_node('tf_listener')
br = tf.TransformBroadcaster()
tf_listener_ = tf.TransformListener()
pi = math.pi
while not rospy.is_shutdown():
    br.sendTransform((0, 0, 0.16115),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "frame1",
                        "frame0")
    br.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(0, 0, pi/2),
                        rospy.Time.now(),
                        "frame2",
                        "frame1")
    br.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(pi/2, -pi/2, 0),
                        rospy.Time.now(),
                        "frame3",
                        "frame2")
    br.sendTransform((0, 0, 0.49603),
                        tf.transformations.quaternion_from_euler(pi/2, -pi/2, 0),
                        rospy.Time.now(),
                        "frame4",
                        "frame3")
    br.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(pi/2, 0, 0),
                        rospy.Time.now(),
                        "frame5",
                        "frame4")
    br.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(pi/2, pi/2, 0),
                        rospy.Time.now(),
                        "frame6",
                        "frame5")
    br.sendTransform((0, -0.11090, 0),
                        tf.transformations.quaternion_from_euler(-pi/2, pi, 0),
                        rospy.Time.now(),
                        "frameT",
                        "frame6")

    # T_in_base = tf_listener_.asMatrix("/frameT", "/frame0")
    # print(T_in_base)

