#!/usr/bin/env python
# todo: license here

# given a target frame, attempt to make thy way over

import rospy
import time
import numpy as np
import math
import sys

# ignore localization stuff for now and use Odometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Vector3,Twist

target = np.zeros(2)
pos = np.zeros(2)
move_speed = 1
dir_vec = np.zeros(2)
arrive_thres = 0.05

init = False
done = False

target_vec = np.zeros(2)
ang_thres = math.pi/150
in_obstacle_realm = False
in_obstacle_realm_thres = 0.4
obstacle_threshold = 0.22
obstacle_mid_reg = (obstacle_threshold + in_obstacle_realm_thres)/2
obstacle_mid_low_reg = obstacle_mid_reg - obstacle_mid_reg/2
obstacle_mid_high_reg = obstacle_mid_reg + obstacle_mid_reg/2
obstacle_push_pull = 0.3
seperating = time.time()
seperating_time = 1

seperation_points = []
seperation_thres = 0.4

# some helper functions, source: https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
def unit_vector(v):
    """ Returns the unit vector of the vector.  """
    return v / np.linalg.norm(v)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

# given the robot frame, update
def update_pos(msg_pos):
    global pos, init, target_vec
    pos[0] = msg_pos.pose.pose.position.x
    pos[1] = msg_pos.pose.pose.position.y

    # if this is the first call to this, say we're ready
    if not init:
        target_vec = target - pos
        target_vec = unit_vector(target_vec)
        init = True


# within the laser callback, determine distance from the nearest obstacle,
# and if it's within a certain threshold, treat it as a window where we have
# entered the realm of an obstacle. Then
def bug2_op(laser):
    global pos, drive_cmd, init, done, target, pos, move_speed, dir_vec, target_vec, ang_thres, in_obstacle_realm, in_obstacle_realm_thres, obstacle_threshold, obstacle_mid_low_reg, obstacle_mid_high_reg, obstacle_push_pull, seperation_points, seperation_thres, seperating, seperating_time, arrive_thres

    # skip if we're not ready
    if not init:
        return

    # if we're done, shutdown
    if done:
        rospy.signal_shutdown("Arrived at target!")
    
    # find the main axis projection of each range, and save the minimum
    ang = laser.angle_min
    inc = laser.angle_increment
    min_range = math.inf
    min_ang = ang
    for range in laser.ranges:
        # find if this is the minimum range
        if range < min_range:
            min_range = range
            min_ang = ang
        # find the angle of the next point
        ang = ang + inc

    # threshold check
    if min_range < in_obstacle_realm_thres and not in_obstacle_realm and not (time.time() - seperating < seperating_time):
        print("Near obstacle!")
    if min_range < obstacle_threshold and not in_obstacle_realm and not (time.time() - seperating < seperating_time):
        print("Following obstacle to left until reattached to m-line!")
        in_obstacle_realm = True
    # clear if we're within a seperaing time
    if time.time() - seperating < seperating_time:
        in_obstacle_realm = False
    # are we free to get out if we are in the obstacle realm?
    if abs(angle_between(target_vec, target-pos)) < ang_thres and in_obstacle_realm:
        #check if that direction is the blocked way, use dir_vec temporarily
        dir_vec[0] = math.cos(min_ang)
        dir_vec[1] = math.sin(min_ang)
        if abs(angle_between(target-pos, dir_vec)) < math.pi/2:
            # literally do nothing
            pass
        #otherwise, if current position is near a seperation point
        #see if we've used it already
        else:
            seperate = True
            for s_point in seperation_points:
                distance = s_point - pos
                if np.sqrt(distance.dot(distance)) < seperation_thres:
                    seperate = False
            # if not, then save the point and mark us as exiting the obstacle_realm
            if seperate:
                print("Splitting from obstacle")
                seperation_points.append(pos.copy())
                in_obstacle_realm = False
                seperating = time.time()

    if in_obstacle_realm:
        # get the 90 degree to left value
        ang = min_ang + math.pi/2
        # try to get the robot between the thresholds
        if min_range < obstacle_mid_low_reg:
            # push the vector a bit to the outside
            push_pull = min_ang + math.pi
            # create the vector
            dir_vec[0] = math.cos(ang) + obstacle_push_pull * math.cos(push_pull)
            dir_vec[1] = math.sin(ang) + obstacle_push_pull * math.sin(push_pull)
        elif min_range > obstacle_mid_high_reg:
            # push the vector a bit to the inside
            push_pull = min_ang
            # create the vector
            dir_vec[0] = math.cos(ang) + obstacle_push_pull * math.cos(push_pull)
            dir_vec[1] = math.sin(ang) + obstacle_push_pull * math.sin(push_pull)
        else:
            dir_vec[0] = math.cos(ang)
            dir_vec[1] = math.sin(ang)
        dir_vec = unit_vector(dir_vec)
    # otherwise just make the normal target, unit vec
    else:
        dir_vec = unit_vector(target - pos)

    # unless, we're close enough to the target, at which we stop the script
    distance = target - pos
    if np.sqrt(distance.dot(distance)) < arrive_thres:
        done = True
        dir_vec = np.zeros(2)

    # generate the twist move command
    move = Twist()
    move.linear.x = move_speed * dir_vec[0]
    move.linear.y = move_speed * dir_vec[1]

    drive_cmd.publish(move)


def bug2(targetx, targety):
    global drive_cmd

    rospy.init_node('bug2_algo')

    # target pos
    target[0] = targetx
    target[1] = targety

    # prepare the movement output
    drive_cmd = rospy.Publisher('maru2/cmd_vel', Twist, queue_size=10)

    # update position of the robot
    rospy.Subscriber('/maru2/move_odom', Odometry, update_pos)

    # grab laser scanner messages as proximity and angle
    rospy.Subscriber('/maru2/laser', LaserScan, bug2_op)
    # All the logic will take place in there

    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            print("Please enter a position x y")
        bug2(argv[1], argv[2])
    except rospy.ROSInterruptException:
        pass
