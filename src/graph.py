#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
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
    y.append(msg.data)

if __name__ == '__main__':
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber("/graph_y", Float64, callback)

    while not rospy.is_shutdown():
        rospy.spin()

    # plt.xlim([0,10000])
    plt.plot(x, y)
    plt.show()
