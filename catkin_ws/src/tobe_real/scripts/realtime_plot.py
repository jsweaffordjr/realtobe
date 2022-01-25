#!/usr/bin/python3
import roslib
import rospy
import matplotlib.pyplot as plt
import math
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Vector3
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
        self.a1, = self.ax1.plot([], [], label='Lean rate', color='red')
        self.a2, = self.ax1.plot([], [], label='Lean', color='black')
        self.a3, = self.ax2.plot([], [], 'b')
        self.a4, = self.ax3.plot([], [], label='Right ankle', color='black')
        self.a5, = self.ax3.plot([], [], label='Left hip', color='green')
        self.a6, = self.ax3.plot([], [], label='Right shoulder', color='red')
        self.a7, = self.ax3.plot([], [], label='Left shoulder', color='blue')
        self.a8, = self.ax3.plot([], [], label='Right hip', color='red')
        self.a9, = self.ax3.plot([], [], label='Left ankle', color='green')
        self.t1, self.t2, self.t3, self.t4, self.t5, self.t6, self.t7, self.t8, self.t9 = [], [], [], [], [], [], [], [], []
        self.z_lean, self.discrete, self.acc, self.r_ankle, self.l_hip, self.r_shoulder, self.l_shoulder, self.r_hip, self.l_ankle = [], [], [], [], [], [], [], [], []
        self.initial_time = rospy.get_time()
        self.fig.suptitle('Real-time plots')
        
    def _calibrate(self,msg):
        x=msg.x
        y=msg.y
        z=msg.z
        if x > 2 and y > 2 and z > 2:
            self.calib=True
            
    def init_plot(self):
        self.ax1.set_title('Longitudinal lean and lean rate')
        self.ax1.set_xlim(0,90)
        self.ax1.set_ylim(-2,2)
        self.ax1.legend(loc='center right')
        self.ax2.set_title('Torso forward acceleration (m/s^2)')
        self.ax2.set_xlim(0,90)
        self.ax2.set_ylim(-6,6)
        self.ax3.set_title('Joint commands (radians)')
        self.ax3.set_xlim(0,90)
        self.ax3.set_ylim(-1.5,1.5)
        self.ax3.legend(loc='center right')
        return self.ax1, self.ax2, self.ax3
        
    def plot_update(self, frame):
        self.a2.set_data(self.t1,self.z_lean)
        self.a1.set_data(self.t2,self.discrete)
        self.a3.set_data(self.t3,self.acc)
        self.a4.set_data(self.t4,self.r_ankle)
        self.a5.set_data(self.t5,self.l_hip)
        self.a6.set_data(self.t6,self.r_shoulder)
        self.a7.set_data(self.t7,self.l_shoulder)
        self.a8.set_data(self.t8,self.r_hip)
        self.a9.set_data(self.t9,self.l_ankle)
        return self.a1, self.a2, self.a3, self.a4, self.a5, self.a6, self.a7, self.a8, self.a9
        
    def lean_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t1.append(tnow)
        self.z_lean.append(msg.z)

    def ldata_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t2.append(tnow)
        self.discrete.append(msg.data)

    def pdata_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t3.append(tnow)
        self.acc.append(msg.data)
            
    def r_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t4.append(tnow)
        self.r_ankle.append(msg.data)
        
    def l_hip_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t5.append(tnow)
        self.l_hip.append(msg.data)  

    def r_shoulder_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t6.append(tnow)
        self.r_shoulder.append(msg.data) 
               
    def l_shoulder_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t7.append(tnow)
        self.l_shoulder.append(msg.data)
                        
    def r_hip_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t8.append(tnow)
        self.r_hip.append(msg.data)        
               
    def l_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t9.append(tnow)
        self.l_ankle.append(msg.data)

                                       
if __name__ == "__main__":
    rospy.init_node("plot_data")
    rospy.sleep(1)

    rospy.loginfo("Real-time data plot begins after calibration")
    viz = Visualizer()
    sub1 = rospy.Subscriber('/lean', Vector3, viz.lean_callback)
    sub2 = rospy.Subscriber('/leandata', Float64, viz.ldata_callback)
    sub3 = rospy.Subscriber('/pushdata', Float64, viz.pdata_callback)
    sub4 = rospy.Subscriber('/tobe/l_ankle_swing_joint_position_controller/command', Float64, viz.l_ankle_callback)
    sub5 = rospy.Subscriber('/tobe/r_ankle_swing_joint_position_controller/command', Float64, viz.r_ankle_callback) 
    sub6 = rospy.Subscriber('/tobe/l_hip_swing_joint_position_controller/command', Float64, viz.l_hip_callback) 
    sub7 = rospy.Subscriber('/tobe/r_hip_swing_joint_position_controller/command', Float64, viz.r_hip_callback) 
    sub8 = rospy.Subscriber('/tobe/l_shoulder_swing_joint_position_controller/command', Float64, viz.l_shoulder_callback)
    sub9 = rospy.Subscriber('/tobe/r_shoulder_swing_joint_position_controller/command', Float64, viz.r_shoulder_callback)
    ani = FuncAnimation(viz.fig, viz.plot_update, init_func=viz.init_plot)
    plt.show(block=True)
    

