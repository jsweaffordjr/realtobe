#!/usr/bin/python3

from threading import Thread
import rospy
import math
import numpy as np
from tobe_real.tobe import Tobe
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

class StandFunc:
    """
    Stand Function
    Provides parameters for standing
    """

    def __init__(self):
        # set initial joint angles
        angles = {}

        f = [-0.2, -1.1932, -1.7264, 0.4132, -0.15, 0, 0, 0.15, -0.4132, 1.7264,
             1.1932, 0.2, -0.3927, -0.3491, -0.5236, 0.3927, -0.3491, 0.5236]

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
        
    def get_ankles(self, z_lean, z_push):
        
        """ Obtain the joint angles for sagittal ankles"""        
        # determine magnitude of response (PD-ctrl)
        Kp = 0.2
        Kd = 0.2
        
        """ Estimate derivative using Savitzky-Golay filter on past five 'lean' points"""
        deriv_z_lean = -0.2*z_lean[0]-0.1*z_lean[1]+0.1*z_lean[3]+0.2*z_lean[4]
        
        diff = Kp*z_lean[4] + Kd*deriv_z_lean 
        angles = self.init_angles

        f1 = angles["l_ankle_swing_joint"]
        f2 = angles["r_ankle_swing_joint"]
        
        # left ankle angle should increase (f1 + diff), right ankle angle should decrease (f2 - diff) when forward lean occurs
        angles["l_ankle_swing_joint"] = f1 + diff
        angles["r_ankle_swing_joint"] = f2 - diff

        return angles

class Stand:
    """
    Class for making Tobe stand
    """

    def __init__(self, tobe):
        self.tobe = tobe
        self.active = False
        self.standing = False
        self.responding = False # this denotes whether robot is currently responding to "off-balance" signals
        self.func = StandFunc()
        self.ready_pos = self.func.init_angles

        self._th_stand = None
        self.response = self.ready_pos

        self.push=[0,0,0,0,0] # last five values (prev. ~0.1 sec.) from 'push' publisher
        self.lean=[0,0,0,0,0] # last five values (prev. ~0.1 sec.) from 'lean' publisher
        self.qz_min = 0.02 # lean threshold: values less than this are rounded down to zero
        self.az_min = 0.2 # push threshold: values less than this are rounded down to zero
        
        self._sub_acc = rospy.Subscriber("/push", Vector3, self._update_acceleration, queue_size=5) # subscribe to push detector topic
        self._sub_quat = rospy.Subscriber("/lean", Vector3, self._update_orientation, queue_size=5) # subscribe to lean detector topic

        self.start()

    def _update_acceleration(self, msg):
        """
        Catches push detection data and updates robot state
        """
        az = msg.z
        if abs(az) < self.az_min:
            az = 0 
        self.push.pop(0) # remove the first element of self.push array
        self.push.append(az) # add the newest acceleration data to the end of the array
        
        
    def _update_orientation(self, msg):
        """
        Catches lean detection data and updates robot orientation
        """
        qz = msg.z
        if abs(qz) < self.qz_min:
            qz = 0
        self.lean.pop(0) # remove the first element of self.push array
        self.lean.append(qz) # add the newest acceleration data to the end of the array

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
        i = 0
        while not rospy.is_shutdown() and self.standing: # control loop
            # if small disturbance or imbalance is detected, move sagittal ankles reflexively:
            if self.responding == False: 
                if abs(self.lean[4]) >= self.qz_min: # nontrivial lean is detected
                    self.responding = True
                    self.response=func.get_ankles(self.lean,self.push)
                    self.tobe.set_angles(self.response)
                    i = 0
                else:
                    if i > 50:
                        self.tobe.set_angles(self.ready_pos)
                        i = 0
                    else:     
                        i = i+1            
            else:
                if i > 20:
                    if abs(self.lean[4]) < self.qz_min: 
                        self.responding = False
                        self.tobe.set_angles(self.ready_pos)  
                        i = 0
                    else:
                        self.response=func.get_ankles(self.lean,self.push)
                        self.tobe.set_angles(self.response)
                        i = 0
                else:     
                    i = i+1       
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


