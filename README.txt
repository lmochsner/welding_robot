# welding_robot
# Robot Kinematics and Dynamics Capstone: Welding Robot Arm

This code controls the HEBI Actuators on a five degree of freedom serial robot mechanism. Matlab and HEBI Actuators are necessary for this code to run.

The function RUN takes in a robot of class Robot3D and an array of waypoints in the workspace in the form [(x,y,z,),...]. The waypoints represent the position of the end effector. The goal of the code is to command the position, velocity and torque of each elastic actuator so that there is minimal error in the path of the robot. The output of the RUN funciton, is that the robot arm will move in the commanded trajectory. 

A semesters worth of information in Robot Kinematics and Dynamics was necessary to develop the code infrasturcture. To find the path the robot must take I used Denavit-Hartenberg Transformations, jacobians, inverse kinematics. In order to compesate for the dynamic motion of the arm I had to implement PID control and gravity compensation using analtyical jacobians. 

The functions in the 'visualization' folder help with determining the ouput path the algorithims find for a given set of waypoints. 
