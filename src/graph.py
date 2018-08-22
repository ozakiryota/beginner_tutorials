#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

i = 0
x = []
y = []

def callback(msg):
    global i
    global x
    global y 
    i += 1
    x.append(i)
    y.append(msg.twist.twist.linear.x)

if __name__ == '__main__':
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber("/imu_odom", Odometry, callback)

    while not rospy.is_shutdown():
        rospy.spin()
    plt.plot(x, y)
    plt.show()
