#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import sensor_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    upperTwist, lowerTwist  = geometry_msgs.msg.Twist()
    rospy.Subscriber("/imu_1", sensor_msgs.msg.Imu(), callback)
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            listener.lookupTwist('/lower_arm', '/body_fixed', rospy.Time(0),rospy.Duration(0.01),upperTwist)
            listener.lookupTwist('/upper_arm', '/body_fixed', rospy.Time(0),rospy.Duration(0.01),lowerTwist)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        print(upperTwist.x)
        print(upperTwist.y)
        print(upperTwist.z)
        print(lowerTwist.x)
        print(lowerTwist.y)
        print(lowerTwist.z)

        rate.sleep()