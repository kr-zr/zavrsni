# Final BSc Project
This repository contains all the code used for making the Final BSc Project for the Bachelor programme at the Faculty of Electrical Engineering and Computing at the University of Zagreb.

The main code is located in /franka_gazebo/scripts in the _franka_final.py_ file. The script allows Franka Emika robot to move through its workspace, collect information about the object in front of it by making contact with it, use the information to edit a jpg image that a previously trained machine learning model can use to guess which part of the plant is in front of the robot - a flat part, fruit or branching.

The code in _franka_final.py_ contains multiple comments which explain what each part of the code does.

This project uses ROS in combination with Gazebo. More info about ROS can be found [here](https://www.ros.org/) and about Gazebo [here](http://gazebosim.org/).
For controlling the movement of the Franka Emika robot, MoveIt! ROS package is used. More info on the package can be found [here](https://moveit.ros.org/).

To test the main part of the project follow these steps:
1. Build the packages int an existing catkin workspace. More information on creating a catkin workspace available [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
2. Before starting anything in the terminal, some of the code in the packages needs to be changed for a specific case:

   2a) In the /franka_gazebo/worlds/franka_pokusaj.world file, in the part about the model 'panda', 23 instances of the uri need to be changed. The uri that needs    to be changed is /home/karlo/franka/src/franka_gazebo/meshes/... Edit the /home/karlo/franka/src part to correspond to where franka_gazebo package is located      in your case. There were attempts to make the uri more universal, but none worked so for now it has to be changed by hand.

   2b) franka_pokusaj.world file is also the file where you can precisely put the object in front of Franka. To make sure the object is in the workspace of the      robot set the pose element of the object to "<pose frame=''>0.45 -0 0.46 0 -0 0</pose>" or something similar to that, and change the pose or remove any other      object that is set in this or close to this position.

   2c) A few edits are needed in the _franka_final.py_ file. In line 70 of the code (self.img = cv2.imread('/home/karlo/pocetna_slika.jpg')) edit the full path to    match the path where pocetna_slika.jpg is in your case. The picture can be named differently, it can be of any size, it is just improtant that it is completely    black. In the line 166 of the code (self.model = tf.keras.models.load_model("/home/karlo/franka/src/franka_gazebo/scripts/treci_strojno",compile=False)) edit      the full path to treci_strojno file to match the path where the file is located in your case. 

After all of this changes to the files are made, everything should be able to run by following next steps:

3. In the first terminal run _roscore_
4. Open another terminal and run _roslaunch moveit_franka demo.launch_
5. Open another terminal and run _roslaunch franka_gazebo model.launch_
6. Open another terminal and run _rosrun franka_gazebo franka_final.py_

Franka will begin moving through its workspace and try to make contact with the object. This part of the program runs for quite long, as it is set that Franka needs to make at least 200 contacts with the object before it can make a guess about what the object is. After Franka makes enough contacts with the object, a machine learning model will use the picture, which now has white pixels where contacts with the object were made, to guess which part of the stem the object coresponds to. 
