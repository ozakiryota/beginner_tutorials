#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt

i = 0
x = []
y = []

def callback(msg):
    # print('callback')

    global x
    global y
    global i
    
    i += 1
    x.append(i)
    x.pop(0)
    y.append(msg.data)
    y.pop(0)


def graph():
    print('realtimegraph')
    rospy.init_node('realtimegraph', anonymous=True)
    rospy.Subscriber("/graphmsg", Float64, callback)
    
    global x
    global y
    x = [0 for i in range(100)]
    y = [0 for i in range(100)]
    
    plt.ion()
    plt.figure()
    li, = plt.plot(x, y)

    plt.ylim(-1,1)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("real time plot")

    while not rospy.is_shutdown():
        # print('loop')

        li.set_xdata(x)
        li.set_ydata(y)
        plt.xlim(min(x), max(x))
        plt.draw()

        plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    graph()
