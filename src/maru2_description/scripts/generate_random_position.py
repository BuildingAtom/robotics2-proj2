#!/usr/bin/env python
# todo: license here

import random
import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from tf.transformations import quaternion_from_euler

from skimage.io import imread
import numpy as np

# mini helper function to generate the random starting position.

def generate_random_position():

    # get the random noise and location
    xyY_mean = rospy.get_param('spawn_mean_gauss', [0.45, 2.0, 0.0])
    xyY_stddev = rospy.get_param('spawn_stddev_gauss', [0.1, 0.1, 0.00])
    robot = rospy.get_param('robot_name', 'maru2')
    bounds = rospy.get_param('spawn_bounds', [0.4, 0.2, 10.0, 9.8])
    rospy.set_param('spawn_mean_gauss', xyY_mean)
    rospy.set_param('spawn_stddev_gauss', xyY_stddev)
    rospy.set_param('robot_name', robot)
    rospy.set_param('spawn_bounds', bounds)

    # load in the map
    map_path = rospy.get_param('/map/configuration_map', "./room.pgm")
    room_voxel = rospy.get_param('/map/voxel_size', 0.05)
    room_map = imread(map_path)
    # rotate it to match the room (room_map[x][y])
    room_map = np.rot90(room_map, k=-1, axes=(0,1))

    while True:
        # generate values
        xyY = []
        for mean, stddev in zip(xyY_mean, xyY_stddev):
            xyY.append(random.gauss(mean, stddev))

        # don't let it get out of bounds otherwise robot spawns in a broken position
        if xyY[0] < bounds[0]: #xmin
            xyY[0] = bounds[0]
        if xyY[1] < bounds[1]: #ymin
            xyY[1] = bounds[1]
        if xyY[0] > bounds[2]: #xmax
            xyY[0] = bounds[2]
        if xyY[1] > bounds[3]: #ymax
            xyY[1] = bounds[3]

        # check the configuration space
        if room_map[int(xyY[0] / room_voxel)][int(xyY[1] / room_voxel)]:
            break

    print("-x " + str(xyY[0]) + " -y " + str(xyY[1]) + " -Y " + str(xyY[2]))

    q = quaternion_from_euler(0, 0, xyY[2])

    # generate the modelstate information
    state = ModelState()
    state.model_name = robot
    state.pose.position.x = xyY[0]
    state.pose.position.y = xyY[1]
    state.pose.position.z = 0
    state.pose.orientation.x = q[0]
    state.pose.orientation.y = q[1]
    state.pose.orientation.z = q[2]
    state.pose.orientation.w = q[3]

    # now try to add it to the robot
    print("waiting for gazeobo")
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state(state)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    try:
        generate_random_position()
    except rospy.ROSInterruptException:
        pass
