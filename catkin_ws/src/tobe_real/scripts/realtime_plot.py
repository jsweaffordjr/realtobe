#!/usr/bin/python3
import roslib
import rospy
import matplotlib.pyplot as plt
import math
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

# this script subscribes to the /push and /lean topics, generates real-time plot(s)
class Visualizer:
    def __init__(self):
        self.calib=False
        rospy.sleep(5)
        self.sub_cal=rospy.Subscriber('/calibration',Vector3,self._calibrate, queue_size=1)
        while self.calib == False:
            rospy.sleep(0.1)
        rospy.sleep(10) 
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, sharex=True)
        self.fig.set_size_inches(10, 10)
        self.a1, = self.ax1.plot([], [], 'b')
        self.a2, = self.ax2.plot([], [], 'b')
        self.a3, = self.ax3.plot([], [], label='Right ankle', color='red')
        self.a4, = self.ax3.plot([], [], label='Left ankle', color='blue')
        self.t1, self.t2, self.t3, self.t4 = [], [], [], []
        self.z_lean, self.z_acc, self.r_ankle, self.l_ankle = [], [], [], []
        self.initial_time = rospy.get_time()
        self.fig.suptitle('Real-time plots')
        
    def _calibrate(self,msg):
        x=msg.x
        y=msg.y
        z=msg.z
        if x > 2 and y > 2 and z > 2:
            self.calib=True
            
    def init_plot(self):
        self.ax1.set_title('Body acceleration in Z-direction')
        self.ax1.set_xlim(0,90)
        self.ax1.set_ylim(-8,8)
        self.ax2.set_title('Longitudinal lean')
        self.ax2.set_xlim(0,90)
        self.ax2.set_ylim(-0.2,0.2)
        self.ax3.set_title('Ankle commands (radians)')
        self.ax3.set_xlim(0,90)
        self.ax3.set_ylim(-1.5,1.5)
        self.ax3.legend(loc='center right')
        return self.a1, self.a2, self.a3
        
    def plot_update(self, frame):
        self.a1.set_data(self.t1,self.z_acc)
        self.a2.set_data(self.t2,self.z_lean)
        self.a3.set_data(self.t3,self.l_ankle)
        self.a4.set_data(self.t4,self.r_ankle)
        return self.a1, self.a2, self.a3, self.a4
        
    def push_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t1.append(tnow)     
        self.z_acc.append(msg.z)
        
    def lean_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t2.append(tnow)
        self.z_lean.append(msg.z)

    def l_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t3.append(tnow)
        self.l_ankle.append(msg.data)
    
    def r_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t4.append(tnow)
        self.r_ankle.append(msg.data)
                           
if __name__ == "__main__":
    rospy.init_node("plot_data")
    rospy.sleep(1)

    rospy.loginfo("Real-time data plot begins after calibration")
    viz = Visualizer()
    sub1 = rospy.Subscriber('/push', Vector3, viz.push_callback)
    sub2 = rospy.Subscriber('/lean', Vector3, viz.lean_callback)
    sub3 = rospy.Subscriber('/tobe/l_ankle_swing_joint_position_controller/command', Float64, viz.l_ankle_callback)
    sub4 = rospy.Subscriber('/tobe/r_ankle_swing_joint_position_controller/command', Float64, viz.r_ankle_callback) 
    ani = FuncAnimation(viz.fig, viz.plot_update, init_func=viz.init_plot)
    plt.show(block=True)
    

