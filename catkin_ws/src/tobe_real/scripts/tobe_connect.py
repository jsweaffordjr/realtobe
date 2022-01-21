#!/usr/bin/python3
# this script can be used to control TOBE by subscribing to joint position controllers
# and executing those commands on the physical Dynamixel motors 
# via a USB connection to the PC

import rospy
import math
import array as arr
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel
DXL2_ID                     = 2                 # Dynamixel
DXL3_ID                     = 3                 # Dynamixel
DXL4_ID                     = 4                 # Dynamixel
DXL5_ID                     = 5                 # Dynamixel
DXL6_ID                     = 6                 # Dynamixel
DXL7_ID                     = 7                 # Dynamixel
DXL8_ID                     = 8                 # Dynamixel
DXL9_ID                     = 9                 # Dynamixel
DXL10_ID                     = 10                 # Dynamixel
DXL11_ID                     = 11                 # Dynamixel
DXL12_ID                     = 12                 # Dynamixel
DXL13_ID                     = 13                 # Dynamixel
DXL14_ID                     = 14                 # Dynamixel
DXL15_ID                     = 15                 # Dynamixel
DXL16_ID                     = 16                 # Dynamixel
DXL17_ID                     = 17                 # Dynamixel
DXL18_ID                     = 18                 # Dynamixel
BAUDRATE                    = 1000000             # Dynamixel AX12 default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller, Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# initialize goal position list for all 18 motors
vals=[281,741,272,750,409,613,511,511,540,482,610,412,174,848,297,726,550,472]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def movemotor1(ang):
    b=60
    c=1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[0]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor2(ang):
    b=240
    c=1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[1]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor3(ang):
    b=60
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[2]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor4(ang):
    b=240
    c=-1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[3]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor5(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[4]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor6(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[5]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor7(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[6]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor8(ang):
    b=150
    c=1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[7]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor9(ang):
    b=150
    c=-1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[8]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor10(ang):
    b=150
    c=-1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[9]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor11(ang):
    b=150
    c=1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[10]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor12(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[11]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor13(ang):
    b=150
    c=1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[12]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor14(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[13]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor15(ang):
    b=150
    c=-1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[14]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor16(ang):
    b=150
    c=1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[15]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor17(ang):
    b=150
    c=1
    angdeg=ang.data*(180/math.pi) # convert command angle from radians to degrees
    vals[16]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
    
def movemotor18(ang):
    b=150
    c=-1
    angdeg=ang.data*(-180/math.pi) # convert command angle from radians to degrees
    vals[17]=int((1023/300)*(b+angdeg*c)) # convert command angle to 10-bit motor value
 
    
def listener():

    rospy.init_node('tobe_connect',anonymous=True)
             
    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL2_ID)

    # Enable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL3_ID)

    # Enable Dynamixel#4 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL4_ID)

    # Enable Dynamixel#5 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL5_ID)

    # Enable Dynamixel#6 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL6_ID)

    # Enable Dynamixel#7 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL7_ID)

    # Enable Dynamixel#8 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL8_ID)

    # Enable Dynamixel#9 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL9_ID)

    # Enable Dynamixel#10 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL10_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL10_ID)

    # Enable Dynamixel#11 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL11_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL11_ID)

    # Enable Dynamixel#12 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL12_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL12_ID)
    
    # Enable Dynamixel#13 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL13_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL13_ID)

    # Enable Dynamixel#14 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL14_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL14_ID)

    # Enable Dynamixel#15 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL15_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL15_ID)

    # Enable Dynamixel#16 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL16_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL16_ID)

    # Enable Dynamixel#17 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL17_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL17_ID)

    # Enable Dynamixel#18 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL18_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL18_ID)

    rospy.Subscriber('/tobe/l_ankle_lateral_joint_position_controller/command',Float64,movemotor18)
    rospy.Subscriber('/tobe/l_ankle_swing_joint_position_controller/command',Float64,movemotor16)
    rospy.Subscriber('/tobe/l_knee_joint_position_controller/command',Float64,movemotor14)
    rospy.Subscriber('/tobe/l_hip_swing_joint_position_controller/command',Float64,movemotor12)
    rospy.Subscriber('/tobe/l_hip_lateral_joint_position_controller/command',Float64,movemotor10)
    rospy.Subscriber('/tobe/l_hip_twist_joint_position_controller/command',Float64,movemotor8)
    rospy.Subscriber('/tobe/r_hip_twist_joint_position_controller/command',Float64,movemotor7)
    rospy.Subscriber('/tobe/r_hip_lateral_joint_position_controller/command',Float64,movemotor9)
    rospy.Subscriber('/tobe/r_hip_swing_joint_position_controller/command',Float64,movemotor11)
    rospy.Subscriber('/tobe/r_knee_joint_position_controller/command',Float64,movemotor13)
    rospy.Subscriber('/tobe/r_ankle_swing_joint_position_controller/command',Float64,movemotor15)
    rospy.Subscriber('/tobe/r_ankle_lateral_joint_position_controller/command',Float64,movemotor17)
    rospy.Subscriber('/tobe/l_shoulder_swing_joint_position_controller/command',Float64,movemotor2)
    rospy.Subscriber('/tobe/l_shoulder_lateral_joint_position_controller/command',Float64,movemotor4)
    rospy.Subscriber('/tobe/l_elbow_joint_position_controller/command',Float64,movemotor6)
    rospy.Subscriber('/tobe/r_shoulder_swing_joint_position_controller/command',Float64,movemotor1)
    rospy.Subscriber('/tobe/r_shoulder_lateral_joint_position_controller/command',Float64,movemotor3)
    rospy.Subscriber('/tobe/r_elbow_joint_position_controller/command',Float64,movemotor5)
     
    r=rospy.Rate(20) # set loop rate
    
    while not rospy.is_shutdown():
        # Allocate goal position value into byte array
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(vals[0])), DXL_HIBYTE(DXL_LOWORD(vals[0])), DXL_LOBYTE(DXL_HIWORD(vals[0])), DXL_HIBYTE(DXL_HIWORD(vals[0]))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(vals[1])), DXL_HIBYTE(DXL_LOWORD(vals[1])), DXL_LOBYTE(DXL_HIWORD(vals[1])), DXL_HIBYTE(DXL_HIWORD(vals[1]))]
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(vals[2])), DXL_HIBYTE(DXL_LOWORD(vals[2])), DXL_LOBYTE(DXL_HIWORD(vals[2])), DXL_HIBYTE(DXL_HIWORD(vals[2]))]
        param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(vals[3])), DXL_HIBYTE(DXL_LOWORD(vals[3])), DXL_LOBYTE(DXL_HIWORD(vals[3])), DXL_HIBYTE(DXL_HIWORD(vals[3]))]
        param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(vals[4])), DXL_HIBYTE(DXL_LOWORD(vals[4])), DXL_LOBYTE(DXL_HIWORD(vals[4])), DXL_HIBYTE(DXL_HIWORD(vals[4]))]
        param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(vals[5])), DXL_HIBYTE(DXL_LOWORD(vals[5])), DXL_LOBYTE(DXL_HIWORD(vals[5])), DXL_HIBYTE(DXL_HIWORD(vals[5]))]
        param_goal_position7 = [DXL_LOBYTE(DXL_LOWORD(vals[6])), DXL_HIBYTE(DXL_LOWORD(vals[6])), DXL_LOBYTE(DXL_HIWORD(vals[6])), DXL_HIBYTE(DXL_HIWORD(vals[6]))]
        param_goal_position8 = [DXL_LOBYTE(DXL_LOWORD(vals[7])), DXL_HIBYTE(DXL_LOWORD(vals[7])), DXL_LOBYTE(DXL_HIWORD(vals[7])), DXL_HIBYTE(DXL_HIWORD(vals[7]))]
        param_goal_position9 = [DXL_LOBYTE(DXL_LOWORD(vals[8])), DXL_HIBYTE(DXL_LOWORD(vals[8])), DXL_LOBYTE(DXL_HIWORD(vals[8])), DXL_HIBYTE(DXL_HIWORD(vals[8]))]
        param_goal_position10 = [DXL_LOBYTE(DXL_LOWORD(vals[9])), DXL_HIBYTE(DXL_LOWORD(vals[9])), DXL_LOBYTE(DXL_HIWORD(vals[9])), DXL_HIBYTE(DXL_HIWORD(vals[9]))]
        param_goal_position11 = [DXL_LOBYTE(DXL_LOWORD(vals[10])), DXL_HIBYTE(DXL_LOWORD(vals[10])), DXL_LOBYTE(DXL_HIWORD(vals[10])), DXL_HIBYTE(DXL_HIWORD(vals[10]))]
        param_goal_position12 = [DXL_LOBYTE(DXL_LOWORD(vals[11])), DXL_HIBYTE(DXL_LOWORD(vals[11])), DXL_LOBYTE(DXL_HIWORD(vals[11])), DXL_HIBYTE(DXL_HIWORD(vals[11]))]
        param_goal_position13 = [DXL_LOBYTE(DXL_LOWORD(vals[12])), DXL_HIBYTE(DXL_LOWORD(vals[12])), DXL_LOBYTE(DXL_HIWORD(vals[12])), DXL_HIBYTE(DXL_HIWORD(vals[12]))]
        param_goal_position14 = [DXL_LOBYTE(DXL_LOWORD(vals[13])), DXL_HIBYTE(DXL_LOWORD(vals[13])), DXL_LOBYTE(DXL_HIWORD(vals[13])), DXL_HIBYTE(DXL_HIWORD(vals[13]))]
        param_goal_position15 = [DXL_LOBYTE(DXL_LOWORD(vals[14])), DXL_HIBYTE(DXL_LOWORD(vals[14])), DXL_LOBYTE(DXL_HIWORD(vals[14])), DXL_HIBYTE(DXL_HIWORD(vals[14]))]
        param_goal_position16 = [DXL_LOBYTE(DXL_LOWORD(vals[15])), DXL_HIBYTE(DXL_LOWORD(vals[15])), DXL_LOBYTE(DXL_HIWORD(vals[15])), DXL_HIBYTE(DXL_HIWORD(vals[15]))]
        param_goal_position17 = [DXL_LOBYTE(DXL_LOWORD(vals[16])), DXL_HIBYTE(DXL_LOWORD(vals[16])), DXL_LOBYTE(DXL_HIWORD(vals[16])), DXL_HIBYTE(DXL_HIWORD(vals[16]))]
        param_goal_position18 = [DXL_LOBYTE(DXL_LOWORD(vals[17])), DXL_HIBYTE(DXL_LOWORD(vals[17])), DXL_LOBYTE(DXL_HIWORD(vals[17])), DXL_HIBYTE(DXL_HIWORD(vals[17]))]   
          
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()

        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()

        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
            quit()

        # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param_goal_position5)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
            quit()

        # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL6_ID, param_goal_position6)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
            quit()
            
        # Add Dynamixel#7 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL7_ID, param_goal_position7)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL7_ID)
            quit()

        # Add Dynamixel#8 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL8_ID, param_goal_position8)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL8_ID)
            quit()

        # Add Dynamixel#9 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL9_ID, param_goal_position9)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL9_ID)
            quit()

        # Add Dynamixel#10 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL10_ID, param_goal_position10)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL10_ID)
            quit()

        # Add Dynamixel#11 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL11_ID, param_goal_position11)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL11_ID)
            quit()

        # Add Dynamixel#12 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL12_ID, param_goal_position12)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL12_ID)
            quit()
            
        # Add Dynamixel#13 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL13_ID, param_goal_position13)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL13_ID)
            quit()

        # Add Dynamixel#14 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL14_ID, param_goal_position14)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL14_ID)
            quit()

        # Add Dynamixel#15 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL15_ID, param_goal_position15)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL15_ID)
            quit()

        # Add Dynamixel#16 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL16_ID, param_goal_position16)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL16_ID)
            quit()

        # Add Dynamixel#17 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL17_ID, param_goal_position17)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL17_ID)
            quit()

        # Add Dynamixel#18 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL18_ID, param_goal_position18)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL18_ID)
            quit()
            
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        
        r.sleep()

    
    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#4 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#5 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#6 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#7 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#8 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#9 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#10 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL10_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#11 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL11_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#12 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL12_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#13 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL13_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#14 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL14_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#15 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL15_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#16 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL16_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#17 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL17_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#18 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL18_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()
    
    rospy.spin()

if __name__=="__main__":
    listener()

