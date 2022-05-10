#  Trjaectory Optimization for Segway

This is the final project for MIT 6.832, Spring 2022.

The file structure is:

- trajectory_opt: the scripts for the simulation
  - world.py: construct the segway, obstalces, and reference lines for the tasks
  - visualizer.py: plot the optimal trajectory, states, and torque
  - trajectory_optimization.py: find the optimal trajectory for the segway
  - figs: the plots for the optimal results
    - Task_1: segway passing a narrow space
    - Task_2: segway follow a sharp circle turning
- model: ROS package for visualize the results
- videos: recorded videos for tests (2 videos from different views for each test)

