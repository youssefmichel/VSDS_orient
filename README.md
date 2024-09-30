# VSDS_orient

Copyright (C) 2023, by Youssef Michel.

The project provides the C++ implementation of the Variable Stiffness Dynamical System (VSDS) approach for the orientation case (in Unit quaternions) which aims to generate a torque field
based on a first order DS and a desired stiffness profile in a closed loop configuration. For more details, please check:

> [1] Y. Michel, M. Saveriano, F. J. Abu-Dakka and D. Lee, "Orientation Control with Variable Stiffness Dynamical Systems," 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2023.


The main scripts in this folder are:

- `MotionGeneration.cpp`: contains the implementation of the VSDS algorithm for the orientation case. The `/config` folder contains the parameters used by VSDS_quat, such as the via-points definition, initial and goal orientation. 

- `task_planner.cpp` : virtual class containing functionalities that interface VSDS with the Fast Research Interface (FRI) library.This is responsible for executing the control algorithm on the Kuka robot i.e receiving information related to the robot position, velocities etc.. and sending torque commands to the robot. Override the implementation based on your speficic robot. 


- `lasa_task_planner.cpp`: overrides `task_planner.cpp` to implement several shapes from the lasa handwritting dataset on Riemenan manifolds (https://www.sciencedirect.com/science/article/pii/S0921889023001495). 
- `cutting_task_planner.cpp`: implements the cutting task, shown in the paper.

- `lasa_node.cpp` and `lasa_node.cpp` implement the ros nodes for the aforementioned tasks. 



To launch the nodes, use the `VSDS.launch`, for instance:
```
roslaunch vsds_orient VSDS.launch VSDS_name:=Worm
```





