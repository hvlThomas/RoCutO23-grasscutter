#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

import rospy
from sensor_msgs.msg import JointState
from dynamixel_sdk import *

# Factory default ID of all DYNAMIXEL is 1
DXL1_ID                      = 62
DXL2_ID                      = 63

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL1_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL1_MAXIMUM_POSITION_VALUE  = 4095     # Refer to the Maximum Position Limit of product eManual
DXL2_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL2_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 1000000

DEVICENAME = '/dev/ttyUSB0'

PROTOCOL_VERSION            = 2.0

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def set_goal_position(DXL1_ID, position):
    dxl1_goal_position = int(position)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("Failed to set goal position for ID %d: %s" % (DXL1_ID, packetHandler.getTxRxResult(dxl_comm_result)))
    elif dxl_error != 0:
        rospy.logerr("Error in set goal position for ID %d: %s" % (DXL1_ID, packetHandler.getRxPacketError(dxl_error)))
    else:
        rospy.logerr("Goal position set to %d for ID %s:" % (dxl1_goal_position, DXL1_ID))

def set_goal_position(DXL2_ID, position):
    dxl2_goal_position = int(position)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("Failed to set goal position for ID %d: %s" % (DXL2_ID, packetHandler.getTxRxResult(dxl_comm_result)))
    elif dxl_error != 0:
        rospy.logerr("Error in set goal position for ID %d: %s" % (DXL2_ID, packetHandler.getRxPacketError(dxl_error)))
    else:
        rospy.logerr("Goal position set to %d for ID %s:" % (dxl2_goal_position, DXL2_ID))

def joint_state_callback(data):
    if len(data.position) >= 2:
        joint_position_1 = data.position[0]
        joint_position_2 = data.position[1]
        set_goal_position(DXL1_ID, joint_position_1)
        set_goal_position(DXL2_ID, joint_position_2)

def dynamixel_node():
    rospy.init_node('dynamixel_node', anonymous=True)
    rospy.Subscriber('joint_states', JointState, joint_state_callback)

    if portHandler.openPort():
        rospy.loginfo("Dynamixel port opened")
        if portHandler.setBaudRate(BAUDRATE):
            rospy.loginfo("Baudrate set to %d" % BAUDRATE)
            rospy.spin()
        else:
            rospy.logerr("Failed to set baudrate")
    else:
        rospy.loggerr("Failed to open port")
if __name__=='__main__':
    try:
        dynamixel_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        portHandler.closePort()