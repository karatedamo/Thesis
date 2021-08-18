#!/usr/bin/env python2

import rospy, actionlib, cv2, time

import numpy as np

from PIL import Image
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal, ManipulatorState
from geometry_msgs.msg import PoseStamped


# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the driver
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'

# Populate with target position/orientation (READY POSE)
target.pose.position.x = 0.307
target.pose.position.y = 0.000
target.pose.position.z = 0.590

target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00

# Create goal from target pose
goal = MoveToPoseGoal(goal_pose=target)

# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
