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
import random
import sensor_msgs.msg
from geometry_msgs.msg import Point32
class Kretanje():
    def __init__(self):
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        self.p_cloud = rospy.Publisher('/franka_pozicije_kontakata',sensor_msgs.msg.PointCloud,queue_size=1)                                        #potrebni subscriberi i publisheri
        #self.left_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState,self.left_sensor_callback,queue_size=1)   
        #self.right_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState,self.right_sensor_callback,queue_size=1) #queue_size je 1 kako bi uvijek 
                                                                                                                                                             #najnoviju poruku dobili
        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_hand = moveit_commander.MoveGroupCommander("hand")

        self.pose_goal = geometry_msgs.msg.Pose()
        self.current_pose = geometry_msgs.msg.PoseStamped()
        self.sensor_data_left = gazebo_msgs.msg.ContactsState()                    #sve varijable koje koristim
        self.sensor_data_right = gazebo_msgs.msg.ContactsState()
        self.pozicije = sensor_msgs.msg.PointCloud()
        #self.pozicije.points.append(Point32(0.0, 0.0, 0.0))
        #self.pozicije.channels[0].name = 'kontakti'
        self.tocka = Point32()
        #self.header = std_msgs.msg.Header()
        #self.header.stamp = rospy.Time.now()
        #self.header.seq = 1
        #self.header.frame_id = 'base_link'
        #self.pozicije.header = self.header

        #self.sensor_data_left_staro = gazebo_msgs.msg.ContactsState()
        #self.sensor_data_right_staro = gazebo_msgs.msg.ContactsState()

        #self.lijevo = True
        #self.desno = False
        #self.pamti = std_msgs.msg.Bool
        #self.kontakt = False
        self.brojac = 0
        self.kreni = False
        self.pocetni = False
        self.novi_brojac = 0
        #self.nova_lista = []
        #self.pomocna_lista =['1']

     
    #def left_sensor_callback(self,data):
        #print("1")
        #self.sensor_data_left = data
        #print(self.sensor_data_left)                 
        
        
    #def right_sensor_callback(self,data):
        #print("4")
        #self.sensor_data_right = data                   #posto je gripper spojeni, i lijevi i desni finger daju otprilike istu poziciju, pa nije potrebno koristiti oba fingera

    
    def run(self):
        
         while not rospy.is_shutdown():
            
            self.sensor_data_left = rospy.wait_for_message('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState)
            self.sensor_data_right = rospy.wait_for_message('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState)
            if self.brojac == 0:                                       #izvrsenje samo kod prvog pokretanja
                self.move_arm.set_named_target("new_searching_position")          #prije definirani polozaj
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.orientation = self.current_pose.pose.orientation           #kopiram orijentaciju end effectora u ovom polozaju koju poslije cijelo vrijeme koristim
                self.pose_goal.position.z = self.current_pose.pose.position.z
                self.brojac = self.brojac + 1
                #self.pocetni = True
            if self.kreni == False and self.brojac > 0:                      #ovo radimo na pocetku kad biramo nasumicni x,y u prostoru u nekom range-u
                self.pose_goal.position.x = random.uniform(0.35, 0.5)
                self.pose_goal.position.y = random.uniform(-0.2, 0.2)
                self.pose_goal.position.z = 0.85
                self.kreni = True
            if not self.sensor_data_left.states and not self.sensor_data_right.states and self.pose_goal.position.z > 0.77 and self.brojac > 0:     #ako nismo ostvarili dodir ili nismo se jos dovoljno spustili, nastavi se spustati
                #print("2")
                self.pose_goal.position.z = self.pose_goal.position.z - 0.008
                self.move_arm.set_pose_target(self.pose_goal)
                self.plan = self.move_arm.go()
                self.move_arm.stop()
            if (self.sensor_data_left.states or self.sensor_data_right.states or self.pose_goal.position.z <= 0.77) and self.brojac > 0:   #ako smo se dovoljno spustili i nismo nista nasli ili smo nasli tocku
                #print("3")
                self.move_arm.set_named_target("new_searching_position")                     #vracamo se u pocetni polozaj
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.position.z = self.current_pose.pose.position.z
                self.kreni = False                                                       #biramo novi x i y
                if self.sensor_data_left.states:                                         #ako je bio ostvareni kontakt publishamo poziciju kao PointCloud poruku
                    print(self.sensor_data_left.states[0].contact_positions[0].x)
                    self.pozicije.header = self.sensor_data_left.header
                    self.tocka.x = self.sensor_data_left.states[0].contact_positions[0].x
                    self.tocka.y = self.sensor_data_left.states[0].contact_positions[0].y
                    self.tocka.z = self.sensor_data_left.states[0].contact_positions[0].z
                    #self.nova_lista.append(copy.deepcopy(self.tocka))
                    #print self.nova_lista
                    self.pozicije.points.append(copy.deepcopy(self.tocka))
                    #self.pozicije.points[0].x = self.sensor_data_left.states[0].contact_positions[0].x
                    #self.pozicije.points[0].y = self.sensor_data_left.states[0].contact_positions[0].y 
                    #self.pozicije.points[0].z = self.sensor_data_left.states[0].contact_positions[0].z
                    self.p_cloud.publish(self.pozicije)
                    #rospy.sleep(1)
                if self.sensor_data_right.states:                                         #ako je bio ostvareni kontakt publishamo poziciju kao PointCloud poruku
                    print(self.sensor_data_right.states[0].contact_positions[0].x)
                    self.pozicije.header = self.sensor_data_right.header
                    self.tocka.x = self.sensor_data_right.states[0].contact_positions[0].x
                    self.tocka.y = self.sensor_data_right.states[0].contact_positions[0].y
                    self.tocka.z = self.sensor_data_right.states[0].contact_positions[0].z
                    #self.nova_lista.append(copy.deepcopy(self.tocka))
                    #print self.nova_lista
                    self.pozicije.points.append(copy.deepcopy(self.tocka))
                    #self.pozicije.points[0].x = self.sensor_data_left.states[0].contact_positions[0].x
                    #self.pozicije.points[0].y = self.sensor_data_left.states[0].contact_positions[0].y 
                    #self.pozicije.points[0].z = self.sensor_data_left.states[0].contact_positions[0].z
                    self.p_cloud.publish(self.pozicije)
                    #rospy.sleep(1)
           #print(self.pose_goal.position.z)         #ovo samo za provjeru            


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
    rospy.init_node('novo_gibanje_robota', anonymous=True)
    
    try:
        ne = Kretanje()
        ne.run()
    except rospy.ROSInterruptException: pass