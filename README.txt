# welding_robot
# Robot Kinematics and Dynamics Capstone: Welding Robot Arm

This code controls the HEBI Actuators on a five degree of freedom serial robot mechanism. Matlab and HEBI Actuators are necessary for this code to run.

The function RUN takes in a robot of class Robot3D and an array of waypoints in the workspace in the form [(x,y,z,),...]. The waypoints represent the position of the end effector. The goal of the code is to command the position, velocity and torque of each elastic actuator so that there is minimal error of the path of the robot. Running the RUN function will result in the robot completing the calculated trajectory. 

A semesters worth of information in Robot Kinematics and Dynamics was necessary to develop the code infrasturcture. To convert form a workspace trajectory to configuration trajectory I used Denavit-Hartenberg Transformations, jacobians and inverse kinematics. In order to compesate for the dynamic motion of the arm I had to implement gravity compensation using analtyical jacobians and PID control. 

The functions in the 'visualization' folder help with visualizing the ouput path the algorithims find for a given set of waypoints. 
