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
from std_msgs.msg import Bool

class Kretanje():
    def __init__(self):
        
        self.left_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_left',gazebo_msgs.msg.ContactsState,self.left_sensor_callback)
        self.right_sensor_subscriber = rospy.Subscriber('/franka/robot_contact_right',gazebo_msgs.msg.ContactsState,self.right_sensor_callback)
        self.pub = rospy.Publisher('/ostvarenje_kontakta',Bool,queue_size=5)

        self.sensor_data_left = gazebo_msgs.msg.ContactsState
        self.sensor_data_right = gazebo_msgs.msg.ContactsState
        self.pocni = False
        self.kontakt = False
        self.brojac = 0
        self.pocni_pocni = False
    
    def left_sensor_callback(self,data):
        #print("1")
        self.sensor_data_left = data
        self.pocni = True

    def right_sensor_callback(self,data):
        #print("2")
        self.sensor_data_right = data
        self.pocni = True
    
    def run(self):
        
        while not rospy.is_shutdown():

            if (self.sensor_data_left.states or self.sensor_data_right.states) and self.pocni_pocni:
                print("kontakt uspostavljen")
                self.kontakt = True
            if self.pocni:
                self.pocni_pocni = True
                rospy.sleep(1)
            self.pub.publish(self.kontakt)



if __name__ == "__main__":
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('citanje_topica', anonymous=True)
    
    try:
        ne = Kretanje()
        ne.run()
    except rospy.ROSInterruptException: pass
