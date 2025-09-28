# TASC-Recruitment-Package

My submission for TASC Communications and Controls team

## How to run:

1. Make sure you have ROS2 Humble installed and sourced.
2. Clone this repository.
3. Go into the cloned directory and the utils folder and build the workspace using `colcon build`.
4. Go back to the main directory and source the install folder using `source utils/install/setup.bash`.
5. Open a new terminal and source the install folder again using `source utils/install/setup.bash`.
6. Then in the first terminal, run `python3 base.py` to start the base node.
7. In the second terminal, run `python3 rover.py` to start the rover node.

## ROS2 RQT Graph:

![alt text](https://github.com/ryan1le/TASC-Recruitment-Package/blob/main/rosgraph.png?raw=true)
