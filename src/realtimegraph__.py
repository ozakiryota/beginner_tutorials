#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt

i = 0
x = [0 for j in range(1000)]
y = [0 for k in range(1000)]

def callback(msg):
    # print('callback')

    global x
    global y
    global i

    i += 1
    x.append(i)
    x.pop(0)
    y.append(msg.data/np.pi*180.0)
    y.pop(0)

def graph():
    print('realtimegraph')
    rospy.init_node('realtimegraph', anonymous=True)
    rospy.Subscriber("/graphmsg", Float64, callback)
    
    global x
    global y
    
    plt.ion()
    plt.figure()
    li, = plt.plot(x, y)

    plt.ylim(-30,30)
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
