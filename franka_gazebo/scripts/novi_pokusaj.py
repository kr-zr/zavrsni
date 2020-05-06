#!/usr/bin/env python

import sys
import copy
import rospy
import numpy
from numpy import array
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from math import pi
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import gazebo_msgs.msg

def listener():
    rospy.init_node('listener_new',anonymous=True)
    brojac = 0
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message('/ostvarenje_kontakta',std_msgs.msg.Bool)
        print(msg.data)
        Kretanje()
        brojac = brojac +1

def Kretanje(kontakt,brojac)
    if brojac == 0:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20
        move_arm = moveit_commander.MoveGroupCommander("arm")
        move_hand = moveit_commander.MoveGroupCommander("hand")

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = geometry_msgs.msg.PoseStamped()
        sensor_data_left = gazebo_msgs.msg.ContactsState()
        sensor_data_right = gazebo_msgs.msg.ContactsState()
        sensor_data_left_staro = gazebo_msgs.msg.ContactsState()
        sensor_data_right_staro = gazebo_msgs.msg.ContactsState()

        pose_goal.position.z = 0.6
        pose_goal.position.x = 0.65
        pose_goal.position.y = 0.0
        lijevo = True
        desno = False
        pamti = std_msgs.msg.Bool
        kontakt = False
        brojac_lijevo = 0
        brojac_desno = 0 
        kreni = False
    if brojac_lijevo == 0:
        move_arm.set_named_target("start_searching_position")
        plan = move_arm.go(wait=True)
        move_arm.stop()
        current_pose = move_arm.get_current_pose()
        pose_goal.orientation = self.current_pose.pose.orientation
        move_hand.set_named_target("open")
        plan2 = move_hand.go(wait=True)
        move_hand.stop()
        brojac_lijevo = brojac_lijevo + 1
                
    if not kontakt:
        #self.current_pose = self.move_arm.get_current_pose()
        #print(len(self.sensor_data_left.states))
        #print(len(self.sensor_data_right.states))
        move_arm.set_pose_target(pose_goal)
        plan = move_arm.go()
        move_arm.stop()
        if  pose_goal.position.x > (0.05) and pose_goal.position.y < 0.6 and lijevo :
            pose_goal.position.x = pose_goal.position.x - 0.05
            pose_goal.position.y = pose_goal.position.y + 0.05
        elif pose_goal.position.x > (0.05) and pose_goal.position.y > (-0.6) and desno:
            pose_goal.position.x = pose_goal.position.x - 0.05
            pose_goal.position.y = pose_goal.position.y - 0.05
        else:
            pose_goal.position.x = 0.65
            pose_goal.position.y = 0.0
            pamti = lijevo
            lijevo = desno
            desno = pamti
            
    if kontakt:
        move_arm.set_named_target("start_searching_position")
        plan = self.move_arm.go(wait=True)
        move_arm.stop()
        rospy.spin()




















if __name__=='__main__':
  try:
    listener()
  except rospy.ROSInterruptException:
    pass