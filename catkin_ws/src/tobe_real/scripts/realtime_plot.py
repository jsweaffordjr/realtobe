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
        # pause until after IMU calibration is completed:
        self.calib=False
        rospy.sleep(5)
        self.sub_cal=rospy.Subscriber('/calibration',Vector3,self._calibrate, queue_size=1)
        while self.calib == False:
            rospy.sleep(0.1)
        rospy.sleep(10) 
        
        # plot initialization:
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, sharex=True)
        self.fig.set_size_inches(10, 10)
        self.a1, = self.ax1.plot([], [], label='Lean rate', color='black')
        self.a2, = self.ax1.plot([], [], label='Lean angle', color='red')
        self.a3, = self.ax2.plot([], [], label='Z-direction', color='blue')
        self.a4, = self.ax3.plot([], [], label='Right ankle', color='blue')
        #self.a5, = self.ax3.plot([], [], label='Left hip', color='green')
        #self.a6, = self.ax3.plot([], [], label='Right shoulder', color='red')
        #self.a7, = self.ax3.plot([], [], label='Left shoulder', color='black')
        #self.a8, = self.ax3.plot([], [], label='Right hip', color='red')
        self.a9, = self.ax3.plot([], [], label='Left ankle', color='green')
        self.t1, self.t2, self.t4, self.t9 = [], [], [], []
        self.lean, self.lean_dot, self.acc, self.r_ankle, self.l_ankle = [], [], [], [], []
        self.initial_time = rospy.get_time()
        self.plot_length = 10 # duration of the plot window
        self.t_start=0
        self.t_end=self.plot_length
        self.fig.suptitle('Real-time plots')
        
    def _calibrate(self,msg):
        x=msg.x
        y=msg.y
        z=msg.z
        if x > 2 and y > 2 and z > 2:
            self.calib=True
            
    def init_plot(self):
        self.ax1.set_title('Longitudinal lean angle and rate')
        self.ax1.set_xlim(0,self.plot_length)
        self.ax1.set_ylim(-0.8,0.8)
        self.ax1.legend(loc='center left')
        self.ax2.set_title('Torso forward acceleration (m/s^2)')
        self.ax2.set_xlim(0,self.plot_length)
        self.ax2.set_ylim(-3,3)
        self.ax2.legend(loc='center left')
        self.ax3.set_title('Commanded joint angles')
        self.ax3.set_xlim(0,self.plot_length)
        self.ax3.set_ylim(-2,2)
        self.ax3.legend(loc='center left')
        return self.ax1, self.ax2, self.ax3        

    def update_plot_length(self):
        self.ax1.set_xlim(self.t_start,self.t_end)
        self.ax2.set_xlim(self.t_start,self.t_end)
        self.ax3.set_xlim(self.t_start,self.t_end)
        return self.ax1, self.ax2, self.ax3  
                
    def plot_update(self, frame):
        self.a2.set_data(self.t1,self.lean)
        self.a1.set_data(self.t1,self.lean_dot)
        self.a3.set_data(self.t2,self.acc)
        self.a4.set_data(self.t4,self.r_ankle)
        #self.a5.set_data(self.t5,self.l_hip)
        #self.a6.set_data(self.t6,self.r_shoulder)
        #self.a7.set_data(self.t7,self.l_shoulder)
        #self.a8.set_data(self.t8,self.r_hip)
        self.a9.set_data(self.t9,self.l_ankle)
        self.update_plot_length()
        return self.a1, self.a2, self.a3, self.a4, self.a9 #, self.a5, self.a6, self.a7, self.a8

    def ldata_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time       
        self.t1.append(tnow)
        self.lean.append(msg.x)
        self.lean_dot.append(msg.y)
        
        # for scrolling plot: (NOTE: if scrolling plot is not desired, comment out the remainder of this callback)
        # after the length of plot window has passed, remove the oldest data point each time a new one comes in
        if tnow > self.plot_length: 
            self.t1.pop(0)
            self.lean.pop(0)
            self.lean_dot.pop(0)
            self.t_end=tnow
            self.t_start=tnow-self.plot_length

    def pdata_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t2.append(tnow)
        self.acc.append(msg.data)
        
        # for scrolling plot: (NOTE: if scrolling plot is not desired, comment out the remainder of this callback)
        # after the length of plot window has passed, remove the oldest data point each time a new one comes in
        if tnow > self.plot_length: 
            self.t2.pop(0)
            self.acc.pop(0)
            
    def r_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t4.append(tnow)
        self.r_ankle.append(msg.data)
        
        # for scrolling plot: (NOTE: if scrolling plot is not desired, comment out the remainder of this callback)
        # after the length of plot window has passed, remove the oldest data point each time a new one comes in
        if tnow > self.plot_length: 
            self.t4.pop(0)
            self.r_ankle.pop(0)
        
    #def l_hip_callback(self, msg):
    #    tnow=rospy.get_time()-self.initial_time
    #    self.t5.append(tnow)
    #    self.l_hip.append(msg.data)  

    #def r_shoulder_callback(self, msg):
    #    tnow=rospy.get_time()-self.initial_time
    #    self.t6.append(tnow)
    #    self.r_shoulder.append(msg.data) 
               
    #def l_shoulder_callback(self, msg):
    #    tnow=rospy.get_time()-self.initial_time
    #    self.t7.append(tnow)
    #    self.l_shoulder.append(msg.data)
                        
    #def r_hip_callback(self, msg):
    #    tnow=rospy.get_time()-self.initial_time
    #    self.t8.append(tnow)
    #    self.r_hip.append(msg.data)        
               
    def l_ankle_callback(self, msg):
        tnow=rospy.get_time()-self.initial_time
        self.t9.append(tnow)
        self.l_ankle.append(msg.data)
        
        # for scrolling plot: (NOTE: if scrolling plot is not desired, comment out the remainder of this callback)
        # after the length of plot window has passed, remove the oldest data point each time a new one comes in
        if tnow > self.plot_length:
            self.t9.pop(0)
            self.l_ankle.pop(0)


                                       
if __name__ == "__main__":
    rospy.init_node("plot_data")
    rospy.sleep(1)

    rospy.loginfo("Real-time data plot begins after calibration")
    viz = Visualizer()
    sub2 = rospy.Subscriber('/leandata', Vector3, viz.ldata_callback)
    sub3 = rospy.Subscriber('/pushdata', Float64, viz.pdata_callback)
    sub4 = rospy.Subscriber('/tobe/l_ankle_swing_joint_position_controller/command', Float64, viz.l_ankle_callback)
    sub5 = rospy.Subscriber('/tobe/r_ankle_swing_joint_position_controller/command', Float64, viz.r_ankle_callback) 
    #sub6 = rospy.Subscriber('/tobe/l_hip_swing_joint_position_controller/command', Float64, viz.l_hip_callback) 
    #sub7 = rospy.Subscriber('/tobe/r_hip_swing_joint_position_controller/command', Float64, viz.r_hip_callback) 
    #sub8 = rospy.Subscriber('/tobe/l_shoulder_swing_joint_position_controller/command', Float64, viz.l_shoulder_callback)
    #sub9 = rospy.Subscriber('/tobe/r_shoulder_swing_joint_position_controller/command', Float64, viz.r_shoulder_callback)
    ani = FuncAnimation(viz.fig, viz.plot_update, init_func=viz.init_plot)
    plt.show(block=True)
    

