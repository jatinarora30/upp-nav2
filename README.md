# Unified Path Planner with Adaptive Safety and Optimality

This repository provides the **Unified Path Planner with Adaptive Safety and OptimalitY** integrated into the **ROS2 Navigation2 stack** as a modified global planner. 
UPP balances **path optimality** and **safety** through a tunable trade-off. 


## Overview
- Implemented by modifying **Nav2’s StraightLine planner**. 
- Parameters **α, β, R** are **hard-coded** in C++ for reproducibility. 
- Validated in simulation (5000 trials) and on TurtleBot3 hardware.

## Installation
```bash
git clone https://github.com/jatinarora30/upp-nav2.git
cd upp-nav2
colcon build
source install/setup.bash
# Running nav2 using this after mapping using this
ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=$HOME/upp-nav2/nav2_straightline_planner/nav2_params.yaml
```


