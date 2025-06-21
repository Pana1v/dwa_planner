DWA Path Planner for TurtleBot3
===============================

A custom Dynamic Window Approach (DWA) implementation for autonomous navigation in ROS2 Humble. This project implements a local path planner that generates safe and efficient trajectories for obstacle avoidance while navigating towards a goal.
  

Prerequisites
-------------

*   ROS2 Humble
    
*   TurtleBot3 packages
    
*   Gazebo simulation environment
    
*   RViz2 for visualization
    

Installation
------------

1.  Clone this repository into your ROS2 workspace:
    

Plain `bashcd ~/ros2_ws/src  git clone` 

1.  Install dependencies:
    

Plain `   bashcd ~/ros2_ws  rosdep install -i --from-path src --rosdistro humble -y   `

1.  Build the package:
    

Plain `   bashcolcon build --packages-select dwa_planner  source install/setup.bash   `

Usage
-----

Running the Simulation
----------------------

1.  **Launch TurtleBot3 World:**
    

Plain `   bashexport TURTLEBOT3_MODEL=burger  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py   `

1.  **Start the DWA planner:**
    

Plain `   bashros2 run dwa_planner dwa_node   `

1.  **Launch RViz for visualization:**
    

Plain `   bashros2 launch turtlebot3_bringup rviz2.launch.py   `

Setting Goal Position
---------------------

You can set the goal position by modifying the parameters in the launch file or by setting them directly:

Plain `   bashros2 run dwa_planner dwa_node --ros-args -p goal_x:=3.0 -p goal_y:=2.0   `

Parameter Tuning
----------------

Key parameters that can be adjusted:

Plain `   bashros2 run dwa_planner dwa_node --ros-args \    -p max_speed:=0.3 \    -p max_rotation_speed:=2.0 \    -p tolerance:=0.5 \    -p trajectory_length:=100   `

Visualization
-------------

Added a Marker topic in RViz with topic name /trajectory\_marker to see the visualization.

Development Issues & Solutions
------------------------------

Index of some issues and solution approach
-----------------------------

1.  **Collisions**
    
    *   _Issue_: Robots corners or edges would collide or graze the obstacle.
        
    *   _Solution_: Added directional checks for corner points around the robot body with tolerance.
        
2.  **Robot Driving Into Obstacles**
    
    *   _Issue_: In some cases where all scores were negative due to collison_scores being triggered, robot would run into an object anyways.
        
    *   _Solution_: Used continue; to actually skip collision trajectories
        
3.  **Circular Motion Patterns**
    
    *   _Issue_: Robot getting stuck in loops instead of progressing
        
    *   _Solution_: Also added 2 Trajectory Lengths, full length and full length/3 which enabled the robot to break out of large circular paths,Added oscillation detection and adjusted scoring weights
        
4.  **Speed Distribution Problems**
    
    *   _Issue_: Oscillation between forward and backward movement, especially when forming two semicircular arcs(paths) for 2 ways of movement
        
    *   _Solution_: Added speed_bias for negative velocities
        
6.  **Side Collisions During Turns**
    
    *   _Issue_: Robot's corners hitting obstacles during rotation
        
    *   _Solution_: Implemented robot footprint checking with multiple angle offsets
        
9.  **Robot Unable to Navigate Around Obstacles**
    
    *   _Issue_: Scoring weights heavily favored goal over obstacle avoidance
        
    *   _Solution_: Rebalanced scoring factors and increased obstacle avoidance weight, also using larger trajectory_length helped the bot to find paths around the obstacle.
        
10.  **All Trajectories Being Rejected**
    
    *   _Issue_: Overly sensitive collision detection rejecting valid paths
        
    *   _Solution_: Implemented movement-direction-based collision checking instead of checking all angles from the footprint.
        

Algorithm Overview
------------------

The DWA implementation follows these steps:

1.  Sample velocity commands within dynamic constraints
    
2.  Predict trajectories for each velocity sample
    
3.  Evaluate trajectories using multi-criteria scoring
    
4.  Select optimal trajectory and execute corresponding velocity
    
5.  Visualize results for debugging and analysis