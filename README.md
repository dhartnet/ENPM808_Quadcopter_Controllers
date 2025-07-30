# ENPM808_Quadcopter_Controllers
Final code and report for UMD ENPM808 Independent Study Course. Project covers design and implementation of PID, LQ, LQI, and MPC controllers for trajectory tracking on a linearized quadcopter model.

## Overview
This project uses a six degree of freedom quadcopter model linearized around a hover point and multiple 3D trajectories for testing controller performance. The eight MATLAB files in this repository include four files for each controller (PID, LQ, LQI, MPC) and four files for tuning each controller type with a Genetic Algorithm (GA). The GA files package the controllers as a cost function and run them though a GA optimization problem to optimize the tuning parameters. The optimized tuning parameters will be printed to the terinal and can be entered into the controller files.

For all eight files, there are different trajectories that can be simulated: 3D Line, Upwards Spiral, Rose Petal Curve. Simulated wind demonstrating a realistic unmeasured disturbance can be added if uncommented. Be sure to pay attention to the comments I left to make sure you comment/uncomment the necessary sections to run each controller properly. Specifically, to run the MPC with added wind, there are a few sections that need to be commented/uncommented as the wind needed to be modeled as an extra set of observable states due to the nature of the MPC Toolbox. 

The GA files have options for tuning on either the 3D Line or Upwards Spiral trajectories. Initial gains, seed gains, GA parameters, and more can be customized as needed.

## Files
***Quadcopter_PID_Trajectory_Tracking.m*** - Simulate quadcopter trajectory tracking with a PID controller on a straight line, spiral, or rose petal curve, with or without wind.
***Quadcopter_LQ_Trajectory_Tracking.m*** - Simulate quadcopter trajectory tracking with an LQ controller on a straight line, spiral, or rose petal curve, with or without wind.
***Quadcopter_LQI_Trajectory_Tracking.m*** - Simulate quadcopter trajectory tracking with an LQI controller on a straight line, spiral, or rose petal curve, with or without wind.
***Quadcopter_MPC_Trajectory_Tracking.m*** - Simulate quadcopter trajectory tracking with an MPC controller on a straight line, spiral, or rose petal curve, with or without wind.

***GA_Tune_PID.m*** - Leverage MATLAB's *ga* function from the Genetic Algorithm Toolbox to optimize tuning for the PID controller via a minimized RMSE cost function on either a line or spiral trajectory.
***GA_Tune_LQ.m*** - Leverage MATLAB's *ga* function from the Genetic Algorithm Toolbox to optimize tuning for the LQ controller via a minimized RMSE cost function on either a line or spiral trajectory.
***GA_Tune_LQI.m*** - Leverage MATLAB's *ga* function from the Genetic Algorithm Toolbox to optimize tuning for the LQI controller via a minimized RMSE cost function on either a line or spiral trajectory.
***GA_Tune_MPC.m*** - Leverage MATLAB's *ga* function from the Genetic Algorithm Toolbox to optimize tuning for the MPC controller via a minimized RMSE cost function on either a line or spiral trajectory.

## Point of Contact
ADD
