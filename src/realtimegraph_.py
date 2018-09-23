#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = 0.0
x_ = 0.0
y_ = 0.0

def callback(msg):
    # print('callback')

    global start_time
    global x_
    global y_
    
    if start_time>0:
        x_ = time.time() - start_time
        y_ = msg.data

def graph():
    print('realtimegraph')
    rospy.init_node('realtimegraph', anonymous=True)
    rospy.Subscriber("/graphmsg", Float64, callback)
    
    x = [0 for j in range(100)]
    y = [0 for k in range(100)]
    
    plt.ion()
    plt.figure()
    li, = plt.plot(x, y)

    plt.ylim(-30,30)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("real time plot")

    global x_
    global y_
    global start_time

    start_time = time.time()

    while not rospy.is_shutdown():
        # print('loop')
        
        x.append(x_)
        x.pop(0)
        y.append(y_/np.pi*180.0)
        y.pop(0)

        li.set_xdata(x)
        li.set_ydata(y)
        plt.xlim(min(x), max(x))
        plt.draw()

        plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    graph()
