#!/usr/bin/env python
import rospy
from dynamixel_workbench_msgs.msg import DynamixelStateList
from std_msgs.msg import Float64
from sensor_msgs.msg import /robot/joint_state
# from sensor_msgs.msg import JointState
import math

# Define arm lengths
L1 = 0.35 # m
L2 = 0.35 # m

joint = /robot/joint_state()

theta1 = joint.position[0] * 56.25
theta2 = joint.position[1] * 56.25

# Define desired tool position
x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + 
theta2)
y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

# Calculate inverse kinematics
c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
s2 = math.sqrt(1 - c2**2)
theta2 = math.atan2(s2, c2)
theta1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)

# Convert angles to degrees
theta1 = math.degrees(theta1)
theta2 = math.degrees(theta2)

# Define publisher node
rospy.init_node('publisher')
pub1 = rospy.Publisher('/dynamixel_workbench/dynamixel_state',
DynamixelStateList, queue_size=10)
pub2 = rospy.Publisher('/dynamixel_workbench/joint_command', Float64,
queue_size=10)
rate = rospy.Rate(10) # 10 Hz

# Publish angles to motors
while not rospy.is_shutdown():
	msg1 = DynamixelStateList()
	msg1.dynamixel_state[0].id = 62 # First Motor id
	msg1.dynamixel_state[0].present_position = theta1 # First motor angle
        msg1.dynamixel_state[1].id = 63 # Second Motor id
        msg1.dynamixel_state[1].present_position = theta2 # Second motor angle
	pub1.publish(msg1)
	msg2 = Float64
	msg2.data = theta1 # First motor angle
	pub2.publish(msg2)
	rate.sleep()
