#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL

COORD_X_OBJET = -0.40
COORD_Y_OBJET = 0.15
COORD_Z_OBJET = 0.35
XC = COORD_X_OBJET
YC = COORD_Y_OBJET
D = 0.50
R = D/2

def main():
    input("Hello man, press enter when you are radis:")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("trajectory_gen_moveit", anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "hc10_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    
    rospy.sleep(2)

    add_box_to_scene(scene, robot)

    # tourne autour de l'objet
    for i in range(9):
        x_i = XC + R * math.cos(2*math.pi * i / 8 )
        y_i = YC + R * math.sin(2*math.pi * i / 8 )

        new_pose = generate_new_pose(
            move_group.get_current_pose(),
            x_i, y_i
        )

        add_pose_to_traj(move_group ,new_pose)

def add_box_to_scene(scene, robot):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = COORD_X_OBJET
    box_pose.pose.position.y = COORD_Y_OBJET
    box_pose.pose.position.z = COORD_Z_OBJET 
    box_name = "box_0"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))


def generate_new_pose(current_pose, x, y):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = current_pose.pose.orientation.x 
    pose_goal.orientation.y = current_pose.pose.orientation.y
    pose_goal.orientation.z = current_pose.pose.orientation.z
    pose_goal.orientation.w = current_pose.pose.orientation.w 
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = current_pose.pose.position.z 

    return pose_goal

def add_pose_to_traj(move_group,  pose_goal):
    move_group.set_pose_target(pose_goal)
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets() 

    


if __name__ == "__main__":
    main()
