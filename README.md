## ROBOTICS_2_project - GROUP 11
Repository for the MATLAB project for the course Robotics and Control 2

Teams members: Ambrosin Gioele, Andreoli Jacopo, Pastorello Edoardo

## The problem: 
A formation of UAVs has to track a UGV evader, that is following its own trajectory.

Each UAV is provided with a camera to retrieve its relative position wrt the UGV

Consensus used for:
Formation control
UGV motion estimation



## To run the project
For the complete simulation, please run  ```project_outline.mlx```

Set the variable traj_sim in the first section as:
```
-true: if you wanto to simulate with Simulink the trajectory of the UGV evader and the searching trajectory of the UAVs
-false: if you want to skip this simulation and load the data from our previous simulation
```
