#! /usr/bin/env python
"""A helper program to test joint tragectory goals for the JACO arms."""

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys

import math

import actionlib
import control_msgs.msg

import goal_generators


def joint_angle_trajectory_client(data):
    """Send a joint angle goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/joint_trajectory_angles/joint_velocity_controller'
    client = actionlib.SimpleActionClient(action_address,
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    client.send_goal(data)

    # goal = jaco_msgs.msg.ArmJointAnglesGoal()

    # goal.angles.joint1 = angle_set[0]
    # goal.angles.joint2 = angle_set[1]-0.17
    # goal.angles.joint3 = angle_set[2]
    # goal.angles.joint4 = angle_set[3]
    # goal.angles.joint5 = angle_set[4]
    # goal.angles.joint6 = angle_set[5]

    # print('goal: {}'.format(goal))

    # client.send_goal(goal)
    # if client.wait_for_result(rospy.Duration(20.0)):
    #     return client.get_result()
    # else:
    #     print('        the joint angle action timed-out')
    #     client.cancel_all_goals()
    #     return None

def callback(data):
    joint_angle_trajectory_client(data)

def receiver():
    rospy.init_node(str(sys.argv[1]) + '_joint_trajectory_workout')

    rospy.Subscriber("jaco_mp_io/joint_trajectory", control_msgs.msg.FollowJointTrajectoryGoal, callback)

    rospy.spin()

if __name__ == '__main__':

    try:        
        receiver()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
