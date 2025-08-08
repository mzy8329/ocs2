# OCS2 Toolbox
## Installation

Create a folder:
> mkdir -p ~/ocs2_ws/src && cd ~/ocs2_ws/src

Clone necessary packages:
> git clone git@github.com:leggedrobotics/ocs2.git
> 
> git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
> 
> git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
>
> git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git
>
> git clone https://github.com/ANYbotics/grid_map.git
>
> git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

Install dependencies:
> sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
>
> sudo apt install -y ros-noetic-pybind11-catkin ros-noetic-grid-map-core ros-noetic-grid-map-msgs libopencv-dev libeigen3-dev libgmp-dev libmpfr-dev libboost-all-dev
>
> sudo apt install libeigen3-dev
>
> sudo apt-get install ros-noetic-rqt-multiplot
>

Build:
> cd ~/ocs2_ws
>
> rosdep install --from-path src --ignore-src -ry 
>
> catkin build -DCMAKE_BUILD_TYPE=Release


## Summary
OCS2 is a C++ toolbox tailored for Optimal Control for Switched Systems (OCS2). The toolbox provides an efficient implementation of the following algorith

* SLQ: Continuous-time domin DDP
* iLQR: Discrete-time domain DDP
* SQP: Multiple-shooting algorithm based on HPIPM
* PISOC: Path integral stochatic optimal control

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides the user with additional tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate its deployment on robotic platforms, the OCS2 provides tools for ROS interfaces. The toolbox’s efficient and numerically stable implementations in conjunction with its user-friendly interface have paved the way for employing it on numerous robotic applications with limited onboard computation power.

For more information refer to the project's [Documentation Page](https://leggedrobotics.github.io/ocs2/) 
