#!/usr/bin/env python

import sys
import copy
import rospy
import numpy
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
class Kretanje():
    def __init__(self):
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        #self.p_cloud = rospy.Publisher('/franka_pozicije_kontakata',sensor_msgs.msg.PointCloud,queue_size=1)                                        #potrebni subscriberi i publisheri
        #self.left_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState,self.left_sensor_callback,queue_size=1)   
        #self.right_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState,self.right_sensor_callback,queue_size=1) #queue_size je 1 kako bi uvijek 
                                                                                                                                                             #najnoviju poruku dobili
        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_hand = moveit_commander.MoveGroupCommander("hand")

        self.pose_goal = geometry_msgs.msg.Pose()
        self.current_pose = geometry_msgs.msg.PoseStamped()
        self.sensor_data_left = gazebo_msgs.msg.ContactsState()                    #sve varijable koje koristim
        self.sensor_data_right = gazebo_msgs.msg.ContactsState()
        #self.pozicije = sensor_msgs.msg.PointCloud()
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
        self.kreni_x = False
        self.kreni_y_desno = False
        #self.kreni_x_lijevo = False
        self.kreni_y_lijevo = False
        self.pocetni = False
        self.novi_brojac = 0
        #self.nova_lista = []
        #self.pomocna_lista =['1']
        self.x_koord = 0
        self.y_koord = 0
        self.x_piksel = 0
        self.y_piksel = 0
        self.broj_tocaka = 0
        self.y_min = -0.4
        self.y_max = 0.4
        self.x_min = 0.1
        self.x_max = 0.9

        self.IMG_SIZE = 80
        self.testing_data = []
        self.CATEGORIES = ["ravno", "racvanje", "plod"]

     
    #def left_sensor_callback(self,data):
        #print("1")
        #self.sensor_data_left = data
        #print(self.sensor_data_left)                 
        
        
    #def right_sensor_callback(self,data):
        #print("4")
        #self.sensor_data_right = data                   #posto je gripper spojeni, i lijevi i desni finger daju otprilike istu poziciju, pa nije potrebno koristiti oba fingera

    
    def run(self):
         
        self.img = cv2.imread('/home/karlo/pocetna_slika.jpg')
        self.img = cv2.resize(self.img,(80,80))   
         
        while not rospy.is_shutdown() and self.broj_tocaka <= 200:
            self.sensor_data_left = rospy.wait_for_message('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState)
            self.sensor_data_right = rospy.wait_for_message('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState)
            if self.brojac == 0:                                       #izvrsenje samo kod prvog pokretanja
                self.move_arm.set_named_target("new_searching_position")          #prije definirani polozaj
                self.plan = self.move_arm.go(wait=True)
                self.move_arm.stop()
                self.current_pose = self.move_arm.get_current_pose()
                self.pose_goal.orientation = self.current_pose.pose.orientation           #kopiram orijentaciju end effectora u ovom polozaju koju poslije cijelo vrijeme koristim
                self.pose_goal.position.z = self.current_pose.pose.position.z
                self.pose_goal.position.x = 0.31 
                self.pose_goal.position.y = -0.145 
                self.brojac = self.brojac + 1
                #self.pocetni = True
            if self.kreni == False and self.brojac > 0:                      #ovo radimo na pocetku kad biramo  x,y u prostoru u nekom range-u
                if not self.kreni_x and not self.kreni_y_desno and not self.kreni_y_lijevo:
                    self.pose_goal.position.x = self.pose_goal.position.x + 0.01
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
                self.pose_goal.position.z = 0.82
                self.kreni = True
            if not self.sensor_data_left.states and not self.sensor_data_right.states and self.pose_goal.position.z > 0.77 and self.brojac > 0:     #ako nismo ostvarili dodir ili nismo se jos dovoljno spustili, nastavi se spustati
                #print("2")
                self.pose_goal.position.z = self.pose_goal.position.z - 0.01
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
                    self.tocka.x = self.sensor_data_left.states[0].contact_positions[0].x
                    self.tocka.y = self.sensor_data_left.states[0].contact_positions[0].y
                    self.tocka.z = self.sensor_data_left.states[0].contact_positions[0].z
                    if self.tocka.x >= self.x_min and self.tocka.x <= self.x_max and self.tocka.y >= self.y_min and self.tocka.y <= self.y_max:
                        self.x_koord = int(round(copy.copy(self.tocka.x)*100.0))
                        self.y_koord = int(round(copy.copy(self.tocka.y)*100.0))
                        self.x_piksel = -(self.x_koord - 10 - 80)
                        self.y_piksel = -(self.y_koord + 40 - 80)
                        self.img [copy.copy(self.x_piksel),copy.copy(self.y_piksel)] = [255, 255, 255]
                        self.broj_tocaka = self.broj_tocaka + 1
                if self.sensor_data_right.states:                                         #ako je bio ostvareni kontakt publishamo poziciju kao PointCloud poruku
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
        #cv2.imwrite('objekt.jpg',self.img)
        #print("gotovo")
        #rospy.spin()
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.img = cv2.resize(self.img, (self.IMG_SIZE, self.IMG_SIZE))
        self.testing_data.append(self.img)

        #plt.imshow(testing_data[0],cmap='gray')
        #plt.show()

        self.testing_data = np.array(self.testing_data).reshape(-1, self.IMG_SIZE,self.IMG_SIZE,1)
        self.testing_data = self.testing_data/255.0

        self.model = tf.keras.models.load_model("treci_strojno",compile=False)  #ValueError: Unknown entries in loss dictionary: [u'class_name', u'config']. Only expected following keys: [u'dense_1']

        self.model_optimizer = tf.keras.optimizers.Adam(learning_rate=0.00007)
        self.model.compile(optimizer=self.model_optimizer, loss=tf.keras.losses.CategoricalCrossentropy(from_logits=True), metrics=['accuracy'])

        self.predictions = self.model.predict_classes([self.testing_data])
        self.dio_biljke = CATEGORIES[self.predictions[0]]
        print(self.dio_biljke)
        if self.dio_biljke == 'racvanje':
            #radi nesto
        if self.dio_biljke == 'ravno':
            #radi nesto drugo
        if self.dio_biljke == 'plod':
            #radi nesto trece
                       
    

if __name__ == "__main__":
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('novo_gibanje_robota', anonymous=True)
    
    try:
        ne = Kretanje()
        ne.run()
    except rospy.ROSInterruptException: pass