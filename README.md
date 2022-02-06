# Final BSc Project
This repository contains all the code used for making the Final BSc Project for the Bachelor programme at the Faculty of Electrical Engineering and Computing at the University of Zagreb.
The main code is located in /franka_gazebo/scripts in the franka_final.py file. The script allows Franka Emika robot to move through its workspace, collect information about the object in front of it by making contact with it, use the information to edit a jpeg image that a previously trained machine learning model can use to guess which part of the plant is in front of the robot - a flat part, fruit or branching.
The code in franka_final.py contains multiple comments which explain what each part of the code does.
This project uses ROS in combination with Gazebo. MOre info about ROS can be found [here](https://www.ros.org/) and about Gazebo [here](http://gazebosim.org/).
For controlling the movement of the Franka Emika robot, MoveIt! ROS package is used. More info on the pacakge can be found [here](https://moveit.ros.org/).
To test the main part of the project follow these steps:
1. Build the packages int an existing catkin workspace. More information on creating a catkin workspace available [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
2. Before starting anything in the terminal, some of the code in the packages needs to be changed for a specific case:
2. a) 
