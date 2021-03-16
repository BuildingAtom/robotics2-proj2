#!/usr/bin/env python
# todo: license here

# Adds simulated GPS sensors for the robot with Gazebo
# A middleman that subscribes to the gazebo world ground truth position of the robot, calculates its distance from the top 4 corners of the room, and
# scales the gaussian noise by distance, with the provided distribution being the noise at 1 meter. It publishes to /robot/gps in a GPS message. Each
# one representing the (0,0), (0,10), (10,10), and (10,0) coordinates based on matrix positions respectively ([0,0], [0,1], [1,1], [1,0]), as beacons.

import math
import random
import threading
import time

import rospy
from maru2_msgs.msg import GPS
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState


beacon = Point()
gps = GPS()

# Calculate the distance between two points
def euclidean_norm(p1, p2):
    x = p1.x - p2.x
    y = p1.y - p2.y
    z = p1.z - p2.z
    return math.sqrt(x*x + y*y + z*z)


# generate the local gps data based on the model state
def calc_pos_and_publish():
    global pub, gps_cov, get_model_state, beacon, robot, gps

    try:
        # get the position of the link chosen
        model_state = get_model_state(robot,'')
        # remember, the model origin is 0.2 towards the rear of the geometric center.

        beacon.x = 0
        beacon.y = 0
        beacon.z = 4
        
        # create the GPS message and calculate each beacon distance
        gps.header = model_state.header
        gps.beacon00 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.x = 10
        gps.beacon10 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.y = 10
        gps.beacon11 = euclidean_norm(beacon, model_state.pose.position)
        
        beacon.x = 0
        gps.beacon01 = euclidean_norm(beacon, model_state.pose.position)
        
        # peturb each beacon distance
        gps.beacon00 = gps.beacon00 + gps.beacon00 * random.gauss(0.0, gps_cov)
        gps.beacon10 = gps.beacon10 + gps.beacon10 * random.gauss(0.0, gps_cov)
        gps.beacon01 = gps.beacon01 + gps.beacon01 * random.gauss(0.0, gps_cov)
        gps.beacon11 = gps.beacon11 + gps.beacon11 * random.gauss(0.0, gps_cov)
        
        # publish the message
        pub.publish(gps)

    except rospy.ServiceException as e:
        print("Service call failed: %s, robot may not have been spawned yet"%e)


def local_gps():
    global pub, gps_cov, get_model_state, robot

    rospy.init_node('local_gps')
    
    # get the rate (it's a slow gps)
    rate = rospy.get_param('sim/gps_rate', 5)
    
    # get the random noise (it scales with distance. this is noise at 1m)
    gps_cov = rospy.get_param('sim/gps_cov', 0.01)
    
    # get the robot name, gps link
    robot = rospy.get_param('robot_name', 'maru2')

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('sim/gps_rate', rate)
    rospy.set_param('sim/gps_cov', gps_cov)

    # sqrt the cov to avoid calculating later
    gps_cov = math.sqrt(gps_cov)

    # get what to publish to and wait for the gazebo service to start
    pub = rospy.Publisher('gps', GPS, queue_size=10)
    
    rospy.wait_for_service('/gazebo/get_link_state')
    model_exists = False
    while not model_exists:
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            get_model_state(robot,'')
            model_exists = True
        except rospy.ROSInterruptException:
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s, robot may not have been spawned yet"%e)
        time.sleep(1)
    
    # start running at rate 
    next = time.time()
    while not rospy.is_shutdown():
        calc_pos_and_publish()
        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except rospy.ROSInterruptException:
            return
        except:
            # if timing is off, just skip
            pass
    
    #keep the process alive
    rospy.spin()


if __name__ == '__main__':
    try:
        local_gps()
    except rospy.ROSInterruptException:
        pass
