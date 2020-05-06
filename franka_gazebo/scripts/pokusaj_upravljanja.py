#!/usr/bin/env python

import rospy
import sys
import copy
import math
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
import trajectory_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64

class Poza():
    def __init__(self):
        
        self.moveit_goal = rospy.Subscriber('trajectory_execution/goal',RobotTrajectory,self.goal_callback)
        self.moveit_pocetak = rospy.Subscriber('/joint_states',JointState,self.pocetak_callback)
        self.polozaj = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1) for i in range(1, 8)]
        self.hand_polozaj = rospy.Publisher('/franka/hand_controller/command',Float64MultiArray, queue_size=1)
        self.hand_msg = Float64MultiArray()
        self.hand_msg.layout.dim = [MultiArrayDimension('', 2, 1)]
        self.goal = RobotTrajectory()
        self.pocetak = JointState()
        self.duljina =0

    def goal_callback(self,data):
        self.goal = data
    
    def pocetak_callback(self,data):
        self.pocetak = data

    def run(self):
        while not rospy.is_shutdown():
            if len(self.pocetak.position) == 9:
                for i in range(7):
                    self.polozaj[i].publish(self.pocetak.position[i]) 
                self.hand_msg.data = [self.pocetak.position[7], self.pocetak.position[8]]
                self.hand_polozaj.publish(self.hand_msg)           

if __name__ == "__main__":

    rospy.init_node('upravljanje_polozajem')

    try:
        ne = Poza()
        ne.run()
    except rospy.ROSInterruptException: pass
            



        



