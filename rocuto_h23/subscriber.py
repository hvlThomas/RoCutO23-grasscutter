#!/usr/bin/env python
import rospy
from dynamixel_workbench_msgs.msg import DynamixelStateList
from std_msgs.msg import Float64
import math
import tf

# Define callback functions
def callback1(data):
	# Get motor angles from DynamixelStateList message
	theta1 = data.dynamixel_state[0].present_position
        theta2 = data.dynamixel_state[1].present_position
	# Convert angles to radians
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)
	# Broadcast transform from baselink to tool frame
	tf_broadcaster.send.Transform((L1 * math.cos(theta1) + L2 * math.cos(theta1 + 
theta2), L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2), 0),
tf.transformations.quaternion_from_euler(0, 0, theta1 + theta2), 
rospy.Time.now(), "tool", "frame")

def callback2(data):
	# Get motor angle from Float64
	theta1 = data.data
	# Convert angle to radians
	theta1 = math.radians(theta1)
	# Broadcast transform form baselink to first motor frame
	tf_broadcaster.sendTransform((L1 * math.cos(theta1), L1 * math.sin(theta1),
0), tf.transformations.quarternion_form_euler(0, 0, theta1), rospy.Time.now(),
"motor1", "frame")

# Define subscriber node
rospy.init_node('subscriber')
sub1 = rospy.Subscriber('/dynamixel_workbench/dynamixel_state',
DynamixelStateList, callback1)
sub2 = rospy.Subscriber('/dynamixel_workbench/joint_command', Float64, callback2)
tf_broadcaster = tf.TransformBroadcaster()

# Spin the node
rospy.spin()



