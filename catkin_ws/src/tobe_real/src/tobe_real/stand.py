#!/usr/bin/python3

from threading import Thread
import rospy
import math
import numpy as np
import random
from numpy import array
from numpy.linalg import norm
from tobe_real.tobe import Tobe
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion

class StandFunc:
    """
    Stand Function
    Provides parameters for standing
    """

    def __init__(self):
        # set initial joint angles
        angles = {} # empty array for angle commands
        
        # array of initial joint angle commands
        f = [-0.2, -1.1, -1.7264, 0.5064, -0.15, 0, 0, 0.15, -0.5064, 1.7264,
             1.1, 0.2, -0.3927, -0.3491, -0.5236, 0.3927, -0.3491, 0.5236]

        # assign command value to each joint
        angles["l_ankle_lateral_joint"] = f[0]
        angles["l_ankle_swing_joint"] = f[1]
        angles["l_knee_joint"] = f[2]
        angles["l_hip_swing_joint"] = f[3]
        angles["l_hip_lateral_joint"] = f[4]
        angles["l_hip_twist_joint"] = f[5]
        angles["r_hip_twist_joint"] = f[6]
        angles["r_hip_lateral_joint"] = f[7]
        angles["r_hip_swing_joint"] = f[8]
        angles["r_knee_joint"] = f[9]
        angles["r_ankle_swing_joint"] = f[10]
        angles["r_ankle_lateral_joint"] = f[11]
        angles["l_shoulder_swing_joint"] = f[12]
        angles["l_shoulder_lateral_joint"] = f[13]
        angles["l_elbow_joint"] = f[14]
        angles["r_shoulder_swing_joint"] = f[15]
        angles["r_shoulder_lateral_joint"] = f[16]
        angles["r_elbow_joint"] = f[17]     
        
        self.init_angles = angles  # set initial array to assigned 'angles'
        self.qz_min = 0.02 # lean threshold: values less than this (~1.15 deg.) are ignored
        self.az_min = 0.1 # push threshold: values less than this are ignored
        
    def get_angles(self, z_lean, l_deriv, l_ddot, gain1):
        # controller gains
        Kpa = gain1 # proportional gain for ankles    
        Kda = 0 # derivative gain for ankles
        
        # add 'diff' and 'diff2' to initial ankle, hip joint commands, respectively:
        diff = 0
        diff2 = 0
        
        # compute ankle adjustments:
        h = 0.29 # height of IMU from ground
        g = 9.81 # acceleration due to gravity
        omega_inv = math.sqrt(h/g)
        diff = Kpa*(z_lean[4]+omega_inv*l_deriv) + Kda*(l_deriv+omega_inv*l_ddot)  # PD control of ankles (capture point) 

        # set initial joint angles
        angles = self.init_angles

        f = [-0.2, -1.1, -1.7264, 0.5064, -0.15, 0, 0, 0.15, -0.5064, 1.7264,
             1.1, 0.2, -0.3927, -0.3491, -0.5236, 0.3927, -0.3491, 0.5236]
        
        # left ankle angle should increase (f1 + diff), right ankle angle should decrease (f10 - diff) when forward lean occurs
        # left hip angle should increase (f3 + diff2), right hip angle should decrease (f8 - diff2) when forward acceleration occurs
        # left shoulder angle should increase (f12 + diff3), right shoulder angle should decrease (f15 - diff3) when forward acc. occurs
        
        f1 = angles["l_ankle_swing_joint"]
        f2 = angles["r_ankle_swing_joint"]
        
        angles["l_ankle_swing_joint"] = f1 + diff
        angles["r_ankle_swing_joint"] = f2 - diff
        angles["l_hip_swing_joint"] = f[3] + diff2
        angles["r_hip_swing_joint"] = f[8] - diff2
        #angles["l_shoulder_swing_joint"] = f[12] + diff3
        #angles["r_shoulder_swing_joint"] = f[15] - diff3

        return angles

class Stand:
    """
    Class for making Tobe stand
    """

    def __init__(self, tobe):
        self.func = StandFunc()
        
        # pause until IMU is calibrated:
        self.calib=False
        rospy.sleep(5)
        self.sub_cal=rospy.Subscriber('/calibration',Vector3,self._calibrate, queue_size=1)
        #rospy.loginfo("Vc: ")
        #rospy.loginfo(self.func.Vc)
        while self.calib == False:
            rospy.sleep(0.1)
        rospy.sleep(10) 
    
        # initialization parameters:
        self.tobe = tobe
        self.active = False
        self.standing = False
        self.ready_pos = self.func.init_angles
        self._th_stand = None
        self.response = self.ready_pos

        # other variables, parameters:
        self.push=0 # smoothed z-acceleration value
        self.lean=[0,0,0,0,0] # last 5 values from 'lean' publisher
        self.l_deriv=0
        self.l_ddot=0
        self.l_int=0
        self.ldata=Vector3()
        self.z_next=[0,0,0,0]
        self.var1=0.008
        self.var2=0.0
        self.qz_min = self.func.qz_min # lean threshold: values less than this are rounded down to zero
        self.az_min = self.func.az_min # push threshold: values less than this are rounded down to zero
        self.time=0
        
        # subscribers and publishers:
        self._sub_quat = rospy.Subscriber("/lean", Vector3, self._update_orientation, queue_size=5) # subscribe to lean topic
        self._sub_acc = rospy.Subscriber("/push", Vector3, self._update_acceleration, queue_size=5) # subscribe to push topic
        self.leandata = rospy.Publisher('leandata', Vector3, queue_size=1)
        self.pushdata = rospy.Publisher('pushdata', Float64, queue_size=1)
        self.Ktuning = rospy.Publisher('var1', Float64, queue_size=1)
        self.Ktuning2 = rospy.Publisher('var2', Float64, queue_size=1)
        self._sub_gain = rospy.Subscriber("/var1", Float64, self._update_gain1, queue_size=5)
        self._sub_gain2 = rospy.Subscriber("/var2", Float64, self._update_gain2, queue_size=5)
        
        # switch to 'active' status, move to initial 'home' position, if not already there:
        self.start()

    def _update_gain1(self, msg):
        # updates 'var1' during experiment
        self.var1 = msg.data
        
    def _update_gain2(self, msg):
        # updates 'var2' during experiment
        self.var2 = msg.data
           
    def _update_acceleration(self, msg):
        """
        Catches acceleration data and applies exponentially weighted moving average to smooth out data output
        """
        w = 0.6 # weight of current data point's contribution to moving average output
        az = msg.z # get acceleration in z-direction      
        if abs(az) < self.az_min: # apply threshold
            az = 0    
        acc=w*az+(1-w)*self.push # update exponentially weighted moving average
        self.pushdata.publish(acc) # publish current (smoothed) acceleration value
        self.push=acc # save current value in 'push'
                    
    def _update_orientation(self, msg):
        """
        Catches lean angle data and updates robot orientation
        """
        q = msg.z # get z-(i.e., forward/backward direction)component of initially vertical x-axis
        if abs(q) < self.qz_min: # apply threshold
            q = 0
            if self.lean[1]+self.lean[2]+self.lean[3]+self.lean[4] == 0:
                self.l_int = 0 # reset integrator if past five values are zero
        
        #qz=q # use lean factor instead of lean angle
        qz = math.asin(q) # convert lean factor to lean angle (inverse sine of z-component of IMU x-axis [which is a unit vector])   
        self.lean.pop(0) # remove oldest value from array of five previous lean angle values
        self.lean.append(qz) # append newest value to end of the array
        
        # derivative and integral estimates:
        dt=0.02 # approximate dt between data points
        
        area=0.5*dt*(self.lean[3]+self.lean[4]) # trapezoidal integration between two values
        prev=self.l_int # get previous value of integral
        self.l_int=prev+area # updated integral value
        
        # homogeneous discrete sliding-mode-based differentiator:
        L=5 # Lipschitz constant
        
        # option 1: if only 1 derivative (z0dot) is needed, use this:
        #z0=self.z_next[0]
        #z1=self.z_next[1]
        #z2=self.z_next[2]
        #z0dot=z1-2.12*(L**(1/3))*(abs(z0-qz)**(2/3))*np.sign(z0-qz)
        #z1dot=z2-2*(L**(2/3))*(abs(z0-qz)**(1/3))*np.sign(z0-qz)
        #z2dot=-1.1*L*np.sign(z0-qz)
        #self.z_next[0]=z0+dt*z0dot+0.5*dt*dt*z2
        #self.z_next[1]=z1+dt*z1dot
        #self.z_next[2]=z2+dt*z2dot
        
        # option 2: if 2 derivatives (z0dot and z1dot) are needed, use this:
        z0=self.z_next[0]
        z1=self.z_next[1]
        z2=self.z_next[2]
        z3=self.z_next[3]
        z0dot=z1-3*(L**(1/4))*(abs(z0-qz)**(3/4))*np.sign(z0-qz)
        z1dot=z2-4.16*(L**(1/2))*(abs(z0-qz)**(1/2))*np.sign(z0-qz)
        z2dot=z3-3.06*(L**(3/4))*(abs(z0-qz)**(1/4))*np.sign(z0-qz)
        z3dot=-1.1*L*np.sign(z0-qz)
        self.z_next[0]=z0+dt*z0dot+0.5*dt*dt*z2+(1/6)*(dt*dt*dt*z3)
        self.z_next[1]=z1+dt*z1dot+0.5*dt*dt*z3
        self.z_next[2]=z2+dt*z2dot
        self.z_next[3]=z3+dt*z3dot

        # HDD output:
        self.l_deriv = z0dot 
        self.l_ddot = z1dot # only use this if using option 2
        
        # assign lean angle and derivative(s) and/or integral:
        self.ldata.x=qz
        self.ldata.y=self.l_deriv
        self.ldata.z=self.l_ddot
        self.leandata.publish(self.ldata)

    def _calibrate(self,msg):
        x=msg.x
        y=msg.y
        z=msg.z
        if x > 2 and y > 2 and z > 2:
            self.calib=True

    def start(self):
        if not self.active:
            self.active = True
            self.init_stand()
            self._th_stand = Thread(target=self._do_stand)
            self._th_stand.start()
            self.standing = True
                    
    def init_stand(self):
        """
        If not already there yet, go to initial standing position
        """
        rospy.loginfo("Going to initial stance")
        if self.get_dist_to_ready() > 0.02:
            self.tobe.set_angles(self.ready_pos)
            rospy.loginfo("Done")

    def _do_stand(self):
        """
        Main standing control loop
        """
        samplerate = 50
        r = rospy.Rate(samplerate)
        rospy.loginfo("Started standing thread")
        func = self.func
        while not rospy.is_shutdown() and self.standing: # control loop     
            # compute appropriate joint commands and execute commands:           
            self.response=func.get_angles(self.lean,self.l_deriv,self.l_ddot,self.var1)
            self.tobe.set_angles(self.response) # set TOBE joint commands to 'response'     
            r.sleep()
        rospy.loginfo("Finished standing control thread")

        self._th_walk = None

    def get_dist_to_ready(self):
        angles = self.tobe.get_angles()
        return get_distance(self.ready_pos, angles)

def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j]-anglesa[j])
    d /= len(joints)
    return d


