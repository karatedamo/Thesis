#!/usr/bin/env python2

import rospy, actionlib, cv2, time
import subprocess 

import matplotlib.pyplot as plt
import numpy as np
import time
import math

from PIL import Image
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal, ManipulatorState, JointVelocity, ActuateGripperAction, ActuateGripperGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

def move (x,y,z,speed_arm = 0.2,ox=-1.00,oy=0.00,oz=0.00,ow=0.00):

    # Create a target pose
    target = PoseStamped()
    target.header.frame_id = 'panda_link0'

    # Populate with target position/orientation (READY POSE)
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z

    target.pose.orientation.x = ox
    target.pose.orientation.y =  oy
    target.pose.orientation.z =  oz
    target.pose.orientation.w =  ow

    # Create goal from target pose
    goal = MoveToPoseGoal(goal_pose=target, speed=speed_arm)

    # Send goal and wait for it to finish
    client.send_goal(goal)
    client.wait_for_result()

def open_gripper():
    goal = ActuateGripperGoal(mode=0, width=0.08)
    gripper_client.send_goal_and_wait(goal)

def close_gripper():
    goal = ActuateGripperGoal(mode=1)
    gripper_client.send_goal_and_wait(goal)

def save_image(image):
    label = "RoboDude"
    im = Image.fromarray(image.astype(np.uint8))
    im.save('{}{}.png'.format(label, int(time.time())))

def get_image(hold_sec):
    # 1D array from publisher - convert into RGB 3D array
    publisher = '/camera/color/image_raw' 
    im_str = np.array(subprocess.getoutput('rostopic echo -n 1 {}'.format(publisher)).split('\n')[11][7:-1].split(','))
    # im_str = np.array(subprocess.getoutput('rostopic echo -n 1 {}'.format(publisher)))
    # print(im_str)
    im_num = im_str.astype(np.float)
    arr_3d = im_num.reshape(720, 1280, 3)
    save_image(arr_3d)

    # if not False:
    arr_3d[:,:,0] = arr_3d[:,:,0]/255
    arr_3d[:,:,1] = arr_3d[:,:,1]/255
    arr_3d[:,:,2] = arr_3d[:,:,2]/255
   
    cv2.imshow('Camera', arr_3d)
    cv2.waitKey(hold_sec*1000)
    
def wide():
    move(0.200, 0.100, 0.845,0.97,0.0095,0.21,-0.008)

rospy.init_node('move_to_points_example')
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

gripper_client = actionlib.SimpleActionClient('/arm/gripper', ActuateGripperAction)
gripper_client.wait_for_server()

home = rospy.ServiceProxy('/arm/home', Empty)







#Camera Wide Angle
# position: 
#       x: 0.26475553704
#       y: 0.205504967856
#       z: 0.808742628205
# orientation: 
#       x: 0.980653085489
#       y: 0.0404335586293
#       z: 0.191467628084
#       w: -0.00437738838989

# NEW Wide Angle
#   position: 
#       x: 0.216201362979
#       y: 0.0994715580669
#       z: 0.844613595075
#     orientation: 
#       x: 0.976578399385
#       y: 0.00950369087089
#       z: 0.214787401332
#       w: -0.00806491282948

# Home = 0.307, 0.000, 0.487128015616
# rosservice call /arm/home





### MAIN()

home()
# close_gripper()
# # Towel Off table
# # position: 
# #       x: 0.18680658286922014
# #       y: -0.7837937001706344
# #       z: 0.5986125918199744
# #     orientation: 
# #       x: 0.7670299357652974
# #       y: -0.5190202230581105
# #       z: 0.15194280805463317
# #       w: 0.34524024917802243
# # print('Go to off Table')
# # move(0.180, -0.783, 0.598,0.76,-0.51,0.15,0.34)
# move(0.6, -0, 0.020,0.2)
# move(0.6, -0, 0.017,0.01)





# points = 9 #square numbers only
# point = int(math.sqrt(points))
# z1 = 0.03
# z2 = 0.019
# # z3 = z1+0.

# x_offset_toTable = 0.34
# table_max = (0.75-0.1)/2
# # each_dist =table_max/(1+point) 

# # dists_y = np.linspace(-table_max/2,table_max/2,point)
# dists_y = np.linspace(-table_max/2, table_max/2, point)
# print('d_y',dists_y)

# dists_x = dists_y+x_offset_toTable+table_max
# print('d_x',dists_x)

# for col in range(point):
#     for rows in range(point):
#         print("x,y",dists_x[rows],dists_y[col])
#         move(dists_x[rows], dists_y[col], z1,0.2) # fast down nearby
#         move(dists_x[rows], dists_y[col], z2,0.01) # slow down exact
#         move(dists_x[rows], dists_y[col], z1,0.2) # fast upnearby

# home()






# home()
#Camera Wide Position
# move(0.530, 0.065, 0.725,1,0.00,0.10,0.00)
# move(0.200, 0.100, 0.845,0.97,0.0095,0.21,-0.008)
# wide()

# arm_state = subprocess.getoutput('rostopic echo -n 1 /arm/state')
# print(arm_state)

# # get_image(15)

# home()
# open_gripper()
# move(0.550, 0.000, 0.487128015616)
# close_gripper()
# home()


# time.sleep(1)

# open_gripper()

# 
# arm_state = subprocess.getoutput('rostopic echo -n 1 /arm/state')
# print(arm_state)

# home()
# close_gripper()
# time.sleep(2)
# open_gripper()
# time.sleep(1)
# home()