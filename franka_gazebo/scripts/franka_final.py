#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import cv2
import os
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
import tensorflow as tf
import matplotlib.pyplot as plt

class Kretanje():
    def __init__(self):
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
                                         #necessary subscribers and publishers
 
        self.move_arm = moveit_commander.MoveGroupCommander("arm")        #move_it commanders
        self.move_hand = moveit_commander.MoveGroupCommander("hand")

        self.pose_goal = geometry_msgs.msg.Pose()
        self.current_pose = geometry_msgs.msg.PoseStamped()
        self.sensor_data_left = gazebo_msgs.msg.ContactsState()                    #all the variables I'm using
        self.sensor_data_right = gazebo_msgs.msg.ContactsState()
        
        self.tocka = Point32()
       
        self.brojac = 0
        self.kreni = False
        self.kreni_x = False
        self.kreni_y_desno = False
        
        self.kreni_y_lijevo = False
        self.pocetni = False
        self.novi_brojac = 0
      
        self.x_koord = 0
        self.y_koord = 0
        self.x_piksel = 0
        self.y_piksel = 0
        self.broj_tocaka = 0
        self.y_min = -0.4
        self.y_max = 0.4
        self.x_min = 0.1
        self.x_max = 0.9

        self.IMG_SIZE = 80                      #preparing variables for analysing data
        self.testing_data = []
        self.CATEGORIES = ["ravno", "racvanje", "plod"]


    
    def run(self):
         
        self.img = cv2.imread('/home/karlo/pocetna_slika.jpg')   #insert a completely black image as a beginning image
        self.img = cv2.resize(self.img,(80,80))      #resize it so that each pixel represents a specific x-y coordinate in franka's workspace
         
        while not rospy.is_shutdown() and self.broj_tocaka <= 200:
            self.sensor_data_left = rospy.wait_for_message('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState)         #wait_for_message works better than subscribing in this case
            self.sensor_data_right = rospy.wait_for_message('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState)
            if self.brojac == 0:                                       #this runs only when starting
                self.move_arm.set_named_target("new_searching_position")          #before defined position which serves as a good position for franka to return to after searching
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.orientation = self.current_pose.pose.orientation           #copy the starting orientation of the end effector which won't change anymore
                self.pose_goal.position.z = self.current_pose.pose.position.z
                self.pose_goal.position.x = 0.31 
                self.pose_goal.position.y = -0.145 
                self.brojac = self.brojac + 1
                
            if self.kreni == False and self.brojac > 0:                      #we do these at the beginning when we're choosing an x-y coordinate in some range
                if not self.kreni_x and not self.kreni_y_desno and not self.kreni_y_lijevo:       #with self_kreni_x, self.kreni_y_desno and self.kreni_y_lijevo we insure that the robot
                    self.pose_goal.position.x = self.pose_goal.position.x + 0.01                  #will search the whole of his workspace in a lawnmower sort of way (goes right to left, increases x, goes left to right, etc.)
                    self.pose_goal.position.y = self.pose_goal.position.y + 0.01
                    self.kreni_y_desno = True
                if not self.kreni_x and self.kreni_y_desno and not self.kreni_y_lijevo:
                    self.pose_goal.position.y = self.pose_goal.position.y + 0.01
                if self.pose_goal.position.y > 0.15:
                    self.kreni_x = True
                if self.kreni_x and self.kreni_y_desno and not self.kreni_y_lijevo:
                    self.pose_goal.position.x = self.pose_goal.position.x + 0.015
                    self.pose_goal.position.y = self.pose_goal.position.y - 0.01
                    self.kreni_y_desno = False
                    self.kreni_y_lijevo = True
                    self.kreni_x = False
                if not self.kreni_x and not self.kreni_y_desno and self.kreni_y_lijevo:
                    self.pose_goal.position.y = self.pose_goal.position.y - 0.01
                if self.pose_goal.position.y < (-0.15):
                    self.kreni_x = True
                if self.kreni_x and not self.kreni_y_desno and self.kreni_y_lijevo:
                    self.pose_goal.position.x = self.pose_goal.position.x + 0.015
                    self.pose_goal.position.y = self.pose_goal.position.y + 0.01
                    self.kreni_y_desno = True
                    self.kreni_y_lijevo = False
                    self.kreni_x = False
                if self.pose_goal.position.x > 0.58:
                    self.kreni_x = False
                    self.kreni_y_desno = False
                    self.kreni_y_lijevo = False
                    self.pose_goal.position.x = 0.31 + 0.005
                    self.pose_goal.position.y = -0.145
                self.pose_goal.position.z = 0.82                          #starting z coordinate
                self.kreni = True               #this variable ensures x and y won't change till z becomes low enough or contact is made at current x-y
            if not self.sensor_data_left.states and not self.sensor_data_right.states and self.pose_goal.position.z > 0.77 and self.brojac > 0:     #if franka didn't make any touch with the object or it hasn't gotten low enough to maybe make contact on current x-y
                #print('2')
                self.pose_goal.position.z = self.pose_goal.position.z - 0.01
                self.move_arm.set_pose_target(self.pose_goal)
                self.plan = self.move_arm.go()
                self.move_arm.stop()
            if (self.sensor_data_left.states or self.sensor_data_right.states or self.pose_goal.position.z <= 0.77) and self.brojac > 0:   #if franka got low enough and has or hasn't made contact
                #print('3')
                self.move_arm.set_named_target("new_searching_position")                     #return to the starting position, next turn franka tries with a new x-y
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.position.z = self.current_pose.pose.position.z
                self.kreni = False                                                       #self.kreni is now false, so franka moves to a new x and y next loop
                if self.sensor_data_left.states:                                         #if contact was made, the position where the contact was made is published as a Pointcloud message
                    print(self.sensor_data_left.states[0].contact_positions[0].x)
                    self.tocka.x = self.sensor_data_left.states[0].contact_positions[0].x
                    self.tocka.y = self.sensor_data_left.states[0].contact_positions[0].y
                    self.tocka.z = self.sensor_data_left.states[0].contact_positions[0].z
                    if self.tocka.x >= self.x_min and self.tocka.x <= self.x_max and self.tocka.y >= self.y_min and self.tocka.y <= self.y_max:  #if contact was made inside the franka workspace
                        self.x_koord = int(round(copy.copy(self.tocka.x)*100.0))                   #turn the x-y coordinates into a pixel on the picture 
                        self.y_koord = int(round(copy.copy(self.tocka.y)*100.0))
                        self.x_piksel = -(self.x_koord - 10 - 80)
                        self.y_piksel = -(self.y_koord + 40 - 80)
                        self.img [copy.copy(self.x_piksel),copy.copy(self.y_piksel)] = [255, 255, 255]
                        self.broj_tocaka = self.broj_tocaka + 1
                if self.sensor_data_right.states:                                         #same as above, only with right sensor
                    print(self.sensor_data_right.states[0].contact_positions[0].x)
                    self.tocka.x = self.sensor_data_right.states[0].contact_positions[0].x
                    self.tocka.y = self.sensor_data_right.states[0].contact_positions[0].y
                    self.tocka.z = self.sensor_data_right.states[0].contact_positions[0].z
                    if self.tocka.x >= self.x_min and self.tocka.x <= self.x_max and self.tocka.y >= self.y_min and self.tocka.y <= self.y_max:
                        self.x_koord = int(round(copy.copy(self.tocka.x)*100.0))
                        self.y_koord = int(round(copy.copy(self.tocka.y)*100.0))
                        self.x_piksel = -(self.x_koord - 10 - 80)
                        self.y_piksel = -(self.y_koord + 40 - 80)
                        self.img [copy.copy(self.x_piksel),copy.copy(self.y_piksel)] = [255, 255, 255]
                        self.broj_tocaka = self.broj_tocaka + 1
        
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)        #setting up the picture to use for the already trained deep learning model
        self.img = cv2.resize(self.img, (self.IMG_SIZE, self.IMG_SIZE))
        self.testing_data.append(self.img)

        self.testing_data = np.array(self.testing_data).reshape(-1, self.IMG_SIZE,self.IMG_SIZE,1)
        self.testing_data = self.testing_data/255.0

        self.model = tf.keras.models.load_model("/home/karlo/franka/src/franka_gazebo/scripts/treci_strojno",compile=False)  #loading in the model

        self.model_optimizer = tf.keras.optimizers.Adam(learning_rate=0.00007)    #making sure the model has some of the needed parameters set correctly
        self.model.compile(optimizer=self.model_optimizer, loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True), metrics=['accuracy'])

        self.predictions = self.model.predict_classes([self.testing_data])    #predict what you see on the picture
        self.dio_biljke = self.CATEGORIES[self.predictions[0]]
        #print(self.dio_biljke)
        if self.dio_biljke == 'racvanje':
            print(self.dio_biljke)
            #radi nesto
        if self.dio_biljke == 'ravno':
            print(self.dio_biljke)
            #radi nesto drugo
        if self.dio_biljke == 'plod':
            print(self.dio_biljke)
            #radi nesto trece
                       
    

if __name__ == "__main__":
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('novo_gibanje_robota', anonymous=True)
    
    try:
        ne = Kretanje()
        ne.run()
    except rospy.ROSInterruptException: pass