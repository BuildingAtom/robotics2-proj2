#!/usr/bin/env python
# todo: license here

# given a target frame, build potential field and follow da way

import rospy
import time
import numpy as np
import math
import sys
import os
import matplotlib.pyplot as plt

from skimage.io import imread, imsave

# ignore localization stuff for now and use Odometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

# do we pause and show the stuff?
pause_show = True

# robot move speed
move_speed = 1.0

# in the below specified range of the goal, don't low-pass filter the gradient
in_target_range = 1.0
at_target_range = 0.05

# low pass filter size for the gradient. should be odd, and simply averages the gradient values around the robot point
lp_size = 1
#develop it
lp_low = -int(lp_size/2)
lp_high = int(lp_low + lp_size)
lp_ind = np.reshape(np.transpose(np.mgrid[lp_low:lp_high,lp_low:lp_high]),(-1,2))

# wavefront expansion on map as numpy array, with map and goal (voxel)
def plotwave(room_map, goalx, goaly):
    # all directions to expand (8 way)
    directions = np.array([[1,0], [1,1],  [0,1], [-1,1],
                           [-1,0],[-1,-1],[0,-1],[1,-1]], dtype=np.int8)
    # 4 directions to expand (4 way)    
    #directions = np.array([[1,0], [0,1],
    #                       [-1,0],[0,-1]], dtype=np.int8)


    # current search (bfs)
    heap = np.array([[goalx,goaly]], dtype=np.int64)
    #max_search = np.shape(room_map)

    # all walls are -1, free space is 0, and expand from goal starting with 1
    room_map = (room_map.astype(np.int64)/255)-1
    val = 1;

    while True:
        # too lazy to vectorize this operation
        mask = np.ones(int(np.size(heap)/2), dtype=bool)
        for i in range(int(np.size(heap)/2)):
            if room_map[tuple(heap[i])] == 0:
                room_map[tuple(heap[i])] = val
            else:
                mask[i] = 0
        # increment the value
        val = val + 1
        # mask out and keep only the ones that were changed
        heap = heap[mask]

        # make sure there's reason to continue, otherwise break
        if not np.size(heap):
            break

        # expand the heap
        heap = np.expand_dims(heap, axis=1) + directions
        heap = np.reshape(heap, (-1, 2))
        heap = np.unique(heap, axis=0)

    # error out if the value is less than 3 (basically did not make a map)
    if val < 3:
        print("Invalid target location!")
        sys.exit(1)
    # return as uint64 to blow out the walls to "infinity"
    return room_map.astype(np.uint64)

# given the robot frame update, output a new command velocity based on the potential field
def update_pos(msg_pos):
    global drive_cmd, room_voxel, wave_grad, target, move_speed, in_target_range, lp_ind, pause_show, at_target_range
    pos = np.array([msg_pos.pose.pose.position.x, msg_pos.pose.pose.position.y])
    map_pos = (pos / room_voxel).astype(int)

    # Get gradient value
    distance = target-pos
    if np.sqrt(distance.dot(distance)) < in_target_range:
        # within range, only get current cell gradient
        xgrad = wave_grad[0][tuple(map_pos)]
        ygrad = wave_grad[1][tuple(map_pos)]
    else:
        # out of range, get the low-pass of it.
        ind = lp_ind + map_pos
        ind = np.transpose(ind)
        xgrad = np.mean(wave_grad[0][tuple(ind)])
        ygrad = np.mean(wave_grad[1][tuple(ind)])
        # if in the odd case it all cancels and something goes wrong, check the specific position
        if xgrad == 0 and ygrad == 0:
            xgrad = wave_grad[0][tuple(map_pos)]
            ygrad = wave_grad[1][tuple(map_pos)]

    #otherwise, if the gradient is 0, then we have a problem, or we have arrived
    if xgrad == 0 and ygrad == 0:
        if np.sqrt(distance.dot(distance)) >= in_target_range:
            print("HELP")
            return

    if np.sqrt(distance.dot(distance)) < at_target_range:
        #with a larger voxel size, here would be a good place to
        #just "push" the robot to place, but it seems unnecessary
        #with the current voxel size
        drive_cmd.publish(Twist())
        print("Arrived at target!")
        rospy.signal_shutdown("Arrived at target!")

    # otherwise, use the gradient to get the target vector (gradient descent)
    target_vec = -np.array([xgrad, ygrad])
    target_vec =  target_vec / np.linalg.norm(target_vec)

    # generate the twist move command
    move = Twist()
    move.linear.x = move_speed * target_vec[0]
    move.linear.y = move_speed * target_vec[1]

    drive_cmd.publish(move)


def wavefront(targetx, targety):
    global drive_cmd, room_voxel, wave_grad, target, pause_show

    rospy.init_node('bug2_algo')

    # load in the map
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    room_map = imread(map_path)
    # rotate it to match the room (room_map[x][y])
    room_map = np.rot90(room_map, k=-1, axes=(0,1))

    # store and transform the goal
    target = np.array([targetx, targety], dtype=float)
    goalx = int(target[0] / room_voxel)
    goaly = int(target[1] / room_voxel)

    # build the wavefront map
    wave = plotwave(room_map, goalx, goaly)

    # show it
    cmap = plt.cm.OrRd
    cmap.set_under(color='black')
    # add one to overflow the -1 int value/max uint value to 0. Well finishes at 2.
    plt.imshow(wave+1, interpolation='none', cmap=cmap, vmin=1)
    # plot start and end
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model_state = get_model_state('maru2','')
    plt.plot(target[1]/room_voxel, target[0] / room_voxel,'bo')
    plt.plot(model_state.pose.position.y/room_voxel, model_state.pose.position.x/room_voxel,'ro')
    # Give a chance to save the map
    if pause_show:
        plt.show()
    else:
        date = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        if not os.path.isdir("generated_wavefront/"):
            os.makedirs("generated_wavefront/")
        plt.savefig("generated_wavefront/"+date+"x"+str(target[0])+"y"+str(target[1])+".png")
        plt.draw()

    # get the gradients (xgrad in 0, ygrad in 1)
    wave_grad = np.gradient(wave.astype(float))

    # prepare the movement output
    drive_cmd = rospy.Publisher('maru2/cmd_vel', Twist, queue_size=10)

    # update position of the robot
    rospy.Subscriber('/maru2/move_odom', Odometry, update_pos)
    # perform all movement logic in here

    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            print("Please enter a position x y")
        if len(argv) > 3 and argv[3] == "no_pause":
            pause_show = False
        wavefront(argv[1], argv[2])
    except rospy.ROSInterruptException:
        pass
