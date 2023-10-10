# trajectory_generation
ROS package that contains implementation of trajectory generation methods such as Model Predictive Control methods.

# Dependencies
This is tested with Ubuntu 22.04, and ROS2 `humble`.


* [custom_trajectory_msgs](https://github.com/mzahana/custom_trajectory_msgs/tree/ros2_humble) `ros2_humble` branch

* Eigen3

* [OSQP](https://github.com/osqp/osqp.git), commit `25b6b39`

* [osqp-eigen](https://github.com/robotology/osqp-eigen.git), version `v0.8.0`

# Setup
A sample [setup.sh](setup.sh) script is availble for convenience in the main directory of this repo.

# Libraries
Core libraries are implementations of MPC with specific model with no dependence on ROS.

* `mpc_6dof.cpp` implements MPC problem for a 3D discrete linear model with position and velocities as states, and acceleration as control inputs.

# Nodes

## mpc_ros
* Uses `mpc_6dof.cpp`

### Subscribers
* `px4_ros/in/odom` : Message type `nav_msgs::msg::Odometry`. This is to get current state (e.g. position, velocity)
* `traj_predictor/in/ref_traj`: Message type [custom_trajectory_msgs::msg::StateTrajectory](https://github.com/mzahana/custom_trajectory_msgs/blob/ros2_humble/msg/StateTrajectory.msg). This subscribes to the desired (reference) trajectory (position, velocity).
### Publishers
* `mpc_tracker/out/path` : Message type `nav_msgs::msg::Path`. To publish the poses of the solution state trajectory to visualize in RViz2
* `mpc_tracker/out/trajectory`: Message type [custom_trajectory_msgs::msg::StateTrajectory](https://github.com/mzahana/custom_trajectory_msgs/blob/ros2_humble/msg/StateTrajectory.msg). Publishes the detailed trajectory (position, velocity, acceleration, ...)


### config file
* [mpc.yaml](config/mpc.yaml)

# Run

```bash
ros2 launch trajectory_generation mpc.launch.py 
```

# Test
The core algorithm can be tested using test cases implemented in the test directory, no dependency on ROS.

* `test_mpc_6dof`
  ```bash
  ros2 run trajectory_generation test_mpc_6dof
  ```
