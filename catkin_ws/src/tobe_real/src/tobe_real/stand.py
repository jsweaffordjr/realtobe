#!/usr/bin/python3

from threading import Thread
import rospy
import math
import numpy as np
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
        angles = {}

        f = [-0.2, -1.1, -1.7264, 0.5064, -0.15, 0, 0, 0.15, -0.5064, 1.7264,
             1.1, 0.2, -0.3927, -0.3491, -0.5236, 0.3927, -0.3491, 0.5236]

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
        self.init_angles = angles  
        self.qz_min = 0.02 # lean threshold: values less than this (~1.15 deg.) are ignored
        self.az_min = 0.2 # push threshold: values less than this are ignored
        
    def get_ankles_hips(self, z_lean, l_deriv):
        # controller gains
        Kpa = 0.01 # proportional gain for sag. ankles
        Kda = 0 # derivative gain for sag. ankles

        Kph = 0.3 # proportional gain for sag. hips
        Kdh = 0   #derivative gain for sag. hips
   
        # thresholds for ankle, hip control:
        minqz = 0.02
        maxqz = 0.1
        minaz = 2
        
        # compute ankle, hip adjustments:
        diff = 0
        diff2 = 0
        
        if (abs(z_lean[4]) > minqz):
            """ P control of lean via sagittal ankles""" 
            diff = Kpa*z_lean[4] #+ Kda*l_deriv # PD control of lean
            if (abs(z_lean[4]) > maxqz):
                diff2 = Kph*z_lean[4] #+ Kdh*l_deriv           
            
        """ Set joint angles"""
        # set initial joint angles
        angles = self.init_angles

        f = [-0.2, -1.1, -1.7264, 0.5064, -0.15, 0, 0, 0.15, -0.5064, 1.7264,
             1.1, 0.2, -0.3927, -0.3491, -0.5236, 0.3927, -0.3491, 0.5236]
        
        # left ankle angle should increase (f1 + diff), right ankle angle should decrease (f10 - diff) when forward lean occurs
        # left hip angle should increase (f3 + diff2), right hip angle should decrease (f8 - diff2) when forward acceleration occurs
        f1 = angles["l_ankle_swing_joint"]
        f2 = angles["r_ankle_swing_joint"]
        
        angles["l_ankle_swing_joint"] = f1 + diff
        angles["r_ankle_swing_joint"] = f2 - diff
        angles["l_hip_swing_joint"] = f[3] + diff2
        angles["r_hip_swing_joint"] = f[8] - diff2

        return angles

class Stand:
    """
    Class for making Tobe stand
    """

    def __init__(self, tobe):
        self.tobe = tobe
        self.active = False
        self.standing = False
        self.func = StandFunc()
        self.ready_pos = self.func.init_angles

        self._th_stand = None
        self.response = self.ready_pos

        self.lean=[0,0,0,0,0] # last 5 values from 'lean' publisher
        self.l_deriv=0
        self.z_next=[0,0,0]
        self.qz_min = self.func.qz_min # lean threshold: values less than this are rounded down to zero
        self.az_min = self.func.az_min # push threshold: values less than this are rounded down to zero
        
        self._sub_quat = rospy.Subscriber("/lean", Vector3, self._update_orientation, queue_size=5) # subscribe to lean detector topic
        self.leandata = rospy.Publisher('leandata', Float64, queue_size=1)

        self.start()
             
    def _update_orientation(self, msg):
        """
        Catches lean detection data and updates robot orientation
        """
        qz = msg.z
        if abs(qz) < self.qz_min:
            qz = 0          
        self.lean.pop(0)
        self.lean.append(qz) # update the newest orientation data
        
        # derivative estimates:
        dt=0.02 # approximate dt between data points
        
        # homogeneous discrete differentiator, assuming 50 Hz rate
        L=5 # Lipschitz constant
        z0=self.z_next[0]
        z1=self.z_next[1]
        z2=self.z_next[2]
        z0dot=z1-2.12*(L**(1/3))*(abs(z0-qz)**(2/3))*np.sign(z0-qz)
        z1dot=z2-2*(L**(2/3))*(abs(z0-qz)**(1/3))*np.sign(z0-qz)
        z2dot=-1.1*L*np.sign(z0-qz)
        self.z_next[0]=z0+dt*z0dot+0.5*dt*dt*z1
        self.z_next[1]=z1+dt*z1dot
        self.z_next[2]=z2+dt*z2dot

        self.l_deriv = z0dot # HDD output
        self.leandata.publish(self.l_deriv)

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
            self.response=func.get_ankles_hips(self.lean,self.l_deriv)
            self.tobe.set_angles(self.response)      
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


