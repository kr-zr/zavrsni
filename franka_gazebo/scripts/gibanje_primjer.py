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

class Kretanje():
    def __init__(self):
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        #self.kontakt_subscriber = rospy.Subscriber('/ostvarenje_kontakta',std_msgs.msg.Bool,self.kontakt_callback)

        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_hand = moveit_commander.MoveGroupCommander("hand")

        self.pose_goal = geometry_msgs.msg.Pose()
        self.current_pose = geometry_msgs.msg.PoseStamped()
        self.sensor_data_left = gazebo_msgs.msg.ContactsState()
        self.sensor_data_right = gazebo_msgs.msg.ContactsState()
        self.sensor_data_left_staro = gazebo_msgs.msg.ContactsState()
        self.sensor_data_right_staro = gazebo_msgs.msg.ContactsState()

        self.pose_goal.position.z = 0.6
        self.pose_goal.position.x = 0.65
        self.pose_goal.position.y = 0.0
        self.lijevo = True
        self.desno = False
        self.pamti = std_msgs.msg.Bool
        self.kontakt = False
        self.brojac_lijevo = 0
        self.brojac_desno = 0 
        self.kreni = False

    #def kontakt_callback(self,data):
       # print("1")
        #self.kontakt = data
    
    def run(self):
        while not rospy.is_shutdown():
            self.msg = rospy.wait_for_message('/ostvarenje_kontakta',std_msgs.msg.Bool)
            print(self.msg.data)
            if self.brojac_lijevo == 0:
                self.move_arm.set_named_target("start_searching_position")
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.orientation = self.current_pose.pose.orientation
                self.move_hand.set_named_target("open")
                self.plan2 = self.move_hand.go(wait=True)
                self.move_hand.stop()
                self.brojac_lijevo = self.brojac_lijevo + 1
                
            if self.msg.data == False:
                #self.current_pose = self.move_arm.get_current_pose()
                #print(len(self.sensor_data_left.states))
                #print(len(self.sensor_data_right.states))
                self.move_arm.set_pose_target(self.pose_goal)
                self.plan = self.move_arm.go()
                self.move_arm.stop()
                if  self.pose_goal.position.x > (0.05) and self.pose_goal.position.y < 0.6 and self.lijevo :
                    self.pose_goal.position.x = self.pose_goal.position.x - 0.05
                    self.pose_goal.position.y = self.pose_goal.position.y + 0.05
                elif self.pose_goal.position.x > (0.05) and self.pose_goal.position.y > (-0.6) and self.desno:
                    self.pose_goal.position.x = self.pose_goal.position.x - 0.05
                    self.pose_goal.position.y = self.pose_goal.position.y - 0.05
                else:
                    self.pose_goal.position.x = 0.65
                    self.pose_goal.position.y = 0.0
                    self.pamti = self.lijevo
                    self.lijevo = self.desno
                    self.desno = self.pamti
            
            if self.msg.data == True :
                self.move_arm.set_named_target("start_searching_position")
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                rospy.spin()
            


#group_names = robot.get_group_names()
#print ("============ Available Planning Groups:", robot.get_group_names())
#move_arm.set_named_target("start_searching_position")

#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 1.0
#pose_goal.position.x = 0.4
#pose_goal.position.y = 0.1
#pose_goal.position.z = 0.4

#move_arm.set_pose_target(pose_goal)


## Now, we call the planner to compute the plan and execute it.
#plan = move_arm.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
#move_arm.stop()

#current_pose = move_arm.get_current_pose()
#while not rospy.is_shutdown():
    #current_pose = move_arm.get_current_pose()
    #print("current pose is ", current_pose)


if __name__ == "__main__":
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('zadano_gibanje_robota', anonymous=True)
    
    try:
        ne = Kretanje()
        ne.run()
    except rospy.ROSInterruptException: pass