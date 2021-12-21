#!/usr/bin/python3
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import serial
import math
import numpy as np
from geometry_msgs.msg import Vector3

ser = serial.Serial('/dev/ttyUSB0', 115200)
runrate = 10 # run loop at this rate in Hz

def talker():
 orient = rospy.Publisher('orientation', String, queue_size=1)
 rotate = rospy.Publisher('rotation', String, queue_size=1)
 linear = rospy.Publisher('linear', String, queue_size=1)
 accel = rospy.Publisher('acceleration', String, queue_size=1)
 calib = rospy.Publisher('calibration', Vector3, queue_size=1)
 rospy.init_node('talker', anonymous=True)
 rate=rospy.Rate(runrate)
 rospy.sleep(2.0)
 while not rospy.is_shutdown():
   # read six lines from serial port:
   data1= ser.readline() # read single line from serial
   data2= ser.readline() # read single line from serial
   data3= ser.readline() # read single line from serial
   data4= ser.readline() # read single line from serial
   data5= ser.readline() # read single line from serial
   data6= ser.readline() # read single line from serial
   
   # convert serial communication lines to strings
   msg1=str(data1)
   msg2=str(data2)
   msg3=str(data3)
   msg4=str(data4)
   msg5=str(data5)
   msg6=str(data6)
   msgs=[msg1,msg2,msg3,msg4,msg5,msg6] # compile strings into list
   
   # determine strings to search for to determine appropriate publisher
   cstr="Calib"
   ostr="Orient"
   rstr="Rot"
   lstr="Linear"
   astr="Accel"
   
   # create blank messages
   cmsg=Vector3()
   #omsg=Vector3()
   #rmsg=Vector3()
   #lmsg=Vector3()
   #amsg=Vector3()

   # assign each message to the appropriate publisher
   try:
      for j in msgs:
         if j.find(cstr)>0:
            gloc=j.find('Gyro=')
            aloc=j.find('Accel=')
            mloc=j.find('Mag=')
            cmsg.x=int(j[gloc+5:gloc+6])
            cmsg.y=int(j[aloc+6:aloc+7])
            cmsg.z=int(j[mloc+4:mloc+5])
            calib.publish(cmsg)
         elif j.find(ostr)>0:
            xloc=j.find('x=')
            xstr=j[xloc:xloc+10]
            xend=xstr.find(' |')
            yloc=j.find('y=')
            ystr=j[yloc:yloc+10]
            yend=ystr.find(' |')
            zloc=j.find('z=')
            zstr=j[zloc:zloc+10]
            zend=zstr.find('\\')
            omsgx=xstr[3:xend]
            omsgy=ystr[3:yend]
            omsgz=zstr[3:zend]
            omsg="%s %s %s"%(omsgx,omsgy,omsgz)
            orient.publish(omsg)
         elif j.find(rstr)>0:
            xloc=j.find('x=')
            xstr=j[xloc:xloc+10]
            xend=xstr.find(' |')
            yloc=j.find('y=')
            ystr=j[yloc:yloc+10]
            yend=ystr.find(' |')
            zloc=j.find('z=')
            zstr=j[zloc:zloc+10]
            zend=zstr.find('\\')
            rmsgx=xstr[3:xend]
            rmsgy=ystr[3:yend]
            rmsgz=zstr[3:zend]
            rmsg="%s %s %s"%(rmsgx,rmsgy,rmsgz)
            rotate.publish(rmsg)
         elif j.find(lstr)>0:
            xloc=j.find('x=')
            xstr=j[xloc:xloc+10]
            xend=xstr.find(' |')
            yloc=j.find('y=')
            ystr=j[yloc:yloc+10]
            yend=ystr.find(' |')
            zloc=j.find('z=')
            zstr=j[zloc:zloc+10]
            zend=zstr.find('\\')
            lmsgx=xstr[3:xend]
            lmsgy=ystr[3:yend]
            lmsgz=zstr[3:zend]
            lmsg="%s %s %s"%(lmsgx,lmsgy,lmsgz)
            linear.publish(lmsg)
         elif j.find(astr)>0:
            xloc=j.find('x=')
            xstr=j[xloc:xloc+10]
            xend=xstr.find(' |')
            yloc=j.find('y=')
            ystr=j[yloc:yloc+10]
            yend=ystr.find(' |')
            zloc=j.find('z=')
            zstr=j[zloc:zloc+10]
            zend=zstr.find('\\')
            amsgx=xstr[3:xend]
            amsgy=ystr[3:yend]
            amsgz=zstr[3:zend]
            amsg="%s %s %s"%(amsgx,amsgy,amsgz)
            accel.publish(amsg)
   except:
      cmsg.x=0
      cmsg.y=0
      cmsg.z=0
      calib.publish(cmsg)
      msg='-1 -1 -1'
      orient.publish(msg)
      rotate.publish(msg)
      linear.publish(msg)
      accel.publish(msg)
      
   rate.sleep()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
