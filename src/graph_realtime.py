#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt

i = 0
x = []
y = []
firstcallback = True

def callback(msg):
    global i
    global x
    global y

    i += 1
    x.append(i)
    y.append(msg.data)

    if firstcallback == True




def graph():
    rospy.init_node('graph_realtime', anonymous=True)
    rospy.Subscriber("/graph_y", Float64, callback)

    global x
    global y

    fig, ax = plt.subplots(1, 1)
    # x.append(0)
    # y.append(0)

    while not rospy.is_shutdown():
        rospy.spin()

    # plt.xlim([0,10000])
    # plt.plot(x, y)
    # plt.show()

if __name__ == '__main__':
    graph()
