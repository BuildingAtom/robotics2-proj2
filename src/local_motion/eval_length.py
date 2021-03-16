#!/usr/bin/env python
# todo: license here

# Polls gazebo for the robot state at rate, calculates it's distance to the last point,
# and pushes the cumulative function to an np array which is plotted and both plot and data exported
# It also stores and plots position. The final path length and time are saved in the exported pickle
# and displayed

import threading
import time
import sys
import os
import math

import pickle
import matplotlib.pyplot as plt
import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

from skimage.io import imread
import numpy as np

lengths = [0]
positions = []
# 20 readings per second
rate = 20

def calc_length_store_pos():
    global get_model_state, positions, lengths
    robot = get_model_state('maru2','')
    current_pos = np.array([robot.pose.position.x, robot.pose.position.y], dtype=float)
    length_delta = np.linalg.norm(current_pos - positions[-1])
    lengths.append(lengths[-1] + length_delta)
    positions.append(current_pos)


def path_eval():
    global get_model_state, positions, start_time, stop_time, room_map, room_voxel, rate

    rospy.init_node('eval_length', anonymous=True)

    # load in the map
    map_path = rospy.get_param('/map/free_space_map', "./room.pgm")
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    room_map = imread(map_path)
    # rotate it to match the room (room_map[x][y])
    room_map = np.rot90(room_map, k=-1, axes=(0,1))

    # we're running off the assumption that everything is already running
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    start_time = rospy.Time.now()
    robot = get_model_state('maru2','')
    positions.append(np.array([robot.pose.position.x, robot.pose.position.y], dtype=float))
    
    # start running at rate 
    next = time.time()
    while not rospy.is_shutdown():
        try:
            calc_length_store_pos()
            next = next + 1.0/rate
            time.sleep(next-time.time())
        except (rospy.ROSInterruptException, rospy.ServiceException):
            break

    # store the end time
    stop_time = rospy.Time.now()

if __name__ == '__main__':
    try:
        path_eval()
    except rospy.ROSInterruptException:
        pass


# finish the evaluation here
print("finishing!")

#save the visualizations
#and dump the stats locally
# for each list, get the numpy representation
positions = np.array(positions)
lengths = np.array(lengths)
timedelta = stop_time.to_sec() - start_time.to_sec()
date = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())

print("Run took", timedelta, "seconds, and went for", lengths[-1], "meters")

x = positions[:,0]
y = positions[:,1]
t = np.linspace(0, np.size(lengths)/rate, np.size(lengths))

if not os.path.isdir("paths/"):
    os.makedirs("paths/")

#plot path
plt.figure()
plt.imshow(room_map)
plt.plot(y/room_voxel, x/room_voxel, 'r')
plt.title('Path taken by algorithm (all states)')
plt.savefig("paths/"+date+"_path.png")
plt.draw()

#plot path over time
plt.figure()
plt.plot(t, x, 'b')
# label the final number
plt.annotate('%0.2f' % x[-1], xy=(1, x[-1]), xytext=(8, 0),
             xycoords=('axes fraction', 'data'), textcoords='offset points')
plt.plot(t, y, 'r')
# label the final number
plt.annotate('%0.2f' % y[-1], xy=(1, y[-1]), xytext=(8, 0),
             xycoords=('axes fraction', 'data'), textcoords='offset points')
#plot the results from this estimate
plt.legend(['X', 'Y'])
plt.ylabel('Position (m)')
plt.xlabel('Time (s)')
plt.title('Path taken by algorithm (positions over time)')
plt.savefig("paths/"+date+"_positions.png")
plt.draw()

#plot length over time
plt.figure()
plt.plot(t, lengths, 'k')
# label the final number
plt.annotate('%0.2f' % lengths[-1], xy=(1, lengths[-1]), xytext=(8, 0),
             xycoords=('axes fraction', 'data'), textcoords='offset points')
#plot the results from this estimate
plt.ylabel('Distance (m)')
plt.xlabel('Time (s)')
plt.title('Path length of algorithm over time')
plt.savefig("paths/"+date+"_length.png")
plt.draw()
    
#dump the raw stats locally
filehandler = open("paths/"+date+"_run.pkl", "wb")
pickle.dump((positions,lengths,timedelta), filehandler)
filehandler.close()
