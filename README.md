# VSDS_Quat
Copyright (C) 2023, by Youssef Michel.
The project provides the C++ implementation of the VSDS approach for the orientation case (in Unit quaternions) which aims to generate a torque field
based on a first order DS and a desired stiffness profile in a closed loop configuration. For more details, please check:



The main scripts in this folder are:

VSDS_robot_main.cpp: The node responsible for executing the control algorithm on the robot. The node communicates
with the Kuka via the FRI receiving information related to the robot position, velocities etc.. and sends torque commands to the
robot. The script is structured in the form of cases, with case 'w' being the one that executes VSDS on the robot. The other cases implement
convenient functionalities such as gravity compensation and homing.

To launch the node use:
```
roslaunch vsds_orient VSDS.launch VSDS_name:=Worm
```


MotionGeneration.h: The class definition for VSDS.
MotionGeneration.cpp: The script where the main VSDS functions are implemented.



