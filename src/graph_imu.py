#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
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
    y.append(msg.angular_velocity.x)

if __name__ == '__main__':
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, callback)

    while not rospy.is_shutdown():
        rospy.spin()

    # plt.xlim([0,10000])
    plt.plot(x, y)
    plt.show()
