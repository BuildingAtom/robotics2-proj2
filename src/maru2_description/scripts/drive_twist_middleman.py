#!/usr/bin/env python
# todo: license here

# A middleman that subscribes to the cmd_vel topic and publishes to a "hidden" controller/cmd_vel and controller/cmd_vel_raw topic.
# It peturbs the input based on the controller/forward_left_var parameter, defining the variance for the forward (x) vector and the left (y) vector.

import random
import time
import math

import rospy
from geometry_msgs.msg import Twist
from maru2_msgs.msg import ControlInput
from gazebo_msgs.srv import GetModelState


state_changed = False
fwd = 0.0
left = 0.0

#correction for rotation
prop = 2

#convert incomplete quaternion (w,z) to z rotation
def quat_rot(quat):
    return math.atan2(2*(quat[0]*quat[1]),1-2*(quat[1]*quat[1]))

def callback_relay(in_msg):
    global state_changed, fwd, left
    
    # Block while state_changed is already true
    while state_changed:
        time.sleep(0.0005)
    # the only input movements are going to be forwards/backwards and sideways velocity
    fwd = in_msg.linear.x
    left = in_msg.linear.y
    state_changed = True


def drive_twist_middleman():
    global state_changed, fwd, left, prop

    rospy.init_node('drive_twist_middleman')
    
    # get the rate
    rate = rospy.get_param('controller/drive_rate', 20)
    
    # get the random noise (holonomic, no turning)
    forward_left_var = rospy.get_param('controller/forward_left_var', [0.2, 0.2])
    
    # setup publishers and subscribers
    pub = rospy.Publisher('controller/cmd_vel', Twist, queue_size=10)
    pub_raw = rospy.Publisher('controller/cmd_vel_raw', ControlInput, queue_size=10) #got a little lazy here

    rospy.Subscriber('cmd_vel', Twist, callback_relay)

    # store the parameters back so that the parameter server can be updated with default values if not present
    rospy.set_param('controller/drive_rate', rate)
    rospy.set_param('controller/forward_left_var', forward_left_var)

    # convert variance to stddev ro reduce complexity in loop
    forward_left_var[0] = math.sqrt(forward_left_var[0])
    forward_left_var[1] = math.sqrt(forward_left_var[1])

    # get feedback on actual robot state so we can remove rotations
    rospy.wait_for_service('/gazebo/get_link_state')
    model_exists = False
    while not model_exists:
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            get_model_state("maru2",'')
            model_exists = True
        except rospy.ROSInterruptException:
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s, robot may not have been spawned yet"%e)
        time.sleep(1)

    # start running at rate 
    next = time.time()
    msg = Twist()
    msg_raw = ControlInput()
    while not rospy.is_shutdown():
        # update control state if changed
        model_state = get_model_state("maru2",'')

        msg.angular.z = -prop*quat_rot([model_state.pose.orientation.w, model_state.pose.orientation.z])

        if state_changed:
            msg_raw.forward = fwd
            msg_raw.left = left
            state_changed = False

        #time
        msg_raw.header.stamp = rospy.Time.now()

        # generate noise based on input
        fwd_noise = random.gauss(0.0, forward_left_var[0]) * msg_raw.forward
        left_noise = random.gauss(0.0, forward_left_var[1]) * msg_raw.left
        
        # update and publish the message with noise
        msg.linear.x = msg_raw.forward + fwd_noise
        msg.linear.y = msg_raw.left + left_noise
        
        pub.publish(msg)
        pub_raw.publish(msg_raw)
        
        # delay
        next = next + 1.0/rate
        try:
            time.sleep(next-time.time())
        except rospy.ROSInterruptException:
            return
        except:
            # if timing is off, just skip
            pass


if __name__ == '__main__':
    try:
        drive_twist_middleman()
    except rospy.ROSInterruptException:
        pass
