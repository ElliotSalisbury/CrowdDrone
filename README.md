#CrowdDrone
CrowdDrone is a tool for performing crowd robotics research. It enables users to stream live video from a robotic agent to an online crowd such that the live annotations from the crowd can be used to influence the control of the robot. Crowd robotics could be used to directly control the robot, with simple directional commands, or imagery from the robot could be rapidly analysed and the analysis used to inform the robots path.
An example simulation environment is provided and built on top of Gazebo. Simulations enables rapid research in this area, but this package could be easily deployed on real robots too.

![alt text](https://raw.githubusercontent.com/ElliotSalisbury/CrowdDrone/master/interface.png "Example Interface")
![alt text](https://raw.githubusercontent.com/ElliotSalisbury/CrowdDrone/master/topdown.png "A view of the environment")

This has been test on Ubuntu 12.04, with ROS Hydro, and Gazebo 3.

Requirements:
* ROS Hydro: [http://wiki.ros.org/hydro]
* Gazebo v3: [http://gazebosim.org/]
* ARDrone Autonomy: [https://github.com/AutonomyLab/ardrone_autonomy]
* Tum Simulator: [https://github.com/ElliotSalisbury/tum_simulator/tree/gazebo3]
