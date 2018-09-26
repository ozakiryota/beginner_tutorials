#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = 0.0
t_ = 0.0
imu = Imu()
first_callback = True

def callback(msg):
    # print('callback')

    global start_time
    global t_
    global imu
    
    if first_callback:
        imu = msg

    lowpass_ratio = 0.0
    imu.angular_velocity.x = (1.0 - lowpass_ratio)*msg.angular_velocity.x + lowpass_ratio * imu.angular_velocity.x
    imu.angular_velocity.y = (1.0 - lowpass_ratio)*msg.angular_velocity.y + lowpass_ratio * imu.angular_velocity.y
    imu.angular_velocity.z = (1.0 - lowpass_ratio)*msg.angular_velocity.z + lowpass_ratio * imu.angular_velocity.z

    if start_time>0:
        t_ = time.time() - start_time

def graph():
    print('graph_imu')
    rospy.init_node('graph_imu', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, callback)
   
    datasize = 1000
    t = [0 for i in range(datasize)]
    wx = [0 for j in range(datasize)]
    wy = [0 for k in range(datasize)]
    wz = [0 for l in range(datasize)]
    
    plt.ion()
    plt.figure()

    ymin = -0.5
    ymax = 0.5

    ### wx ###
    plt.subplot(3, 1, 1)
    plt.title("wx")
    plt.xlabel("time[s]")
    plt.ylabel("wx[rad/s]")
    plt.ylim(ymin, ymax)
    plt.grid(True)
    li_wx, = plt.plot(t, wx)

    ### wy ###
    plt.subplot(3, 1, 2)
    plt.title("wy")
    plt.xlabel("time[s]")
    plt.ylabel("wy[rad/s]")
    plt.ylim(ymin, ymax)
    plt.grid(True)
    li_wy, = plt.plot(t, wy)
    
    ### wz ###
    plt.subplot(3, 1, 3)
    plt.title("wz")
    plt.xlabel("time[s]")
    plt.ylabel("wz[rad/s]")
    plt.ylim(ymin, ymax)
    plt.grid(True)
    li_wz, = plt.plot(t, wz)
    
    global start_time
    global t_
    global imu

    start_time = time.time()

    while not rospy.is_shutdown():
        # print('loop')
        
        t.append(t_)
        t.pop(0)
        wx.append(imu.angular_velocity.x)
        wx.pop(0)
        wy.append(imu.angular_velocity.y)
        wy.pop(0)
        wz.append(imu.angular_velocity.z)
        wz.pop(0)
        
        ### wx ###
        plt.subplot(3,1,1)
        li_wx.set_xdata(t)
        li_wx.set_ydata(wx)
        plt.xlim(min(t), max(t))

        ### wy ###
        plt.subplot(3,1,2)
        li_wy.set_xdata(t)
        li_wy.set_ydata(wy)
        plt.xlim(min(t), max(t))

        ### wz ###
        plt.subplot(3,1,3)
        li_wz.set_xdata(t)
        li_wz.set_ydata(wz)
        plt.xlim(min(t), max(t))
        
        plt.draw()
        plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    graph()
