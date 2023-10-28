/*
BSD 3-Clause License


Copyright (c) 2023, Mohamed Abdelkader Zahana

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef MPC_12STATE_ROS_H
#define MPC_12STATE_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "custom_trajectory_msgs/msg/state_trajectory.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include "trajectory_generation/mpc_12state.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MPCROS : public rclcpp::Node
{
public:
    MPCROS();
    ~MPCROS();
private:

   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    _odom_sub; /** Drone's odometry subscriber, to get position/velocity */
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr      _imu_sub; /** Drone's IMU subscriber, to get acceleration, "mavros/imu/data" */
   rclcpp::Subscription<custom_trajectory_msgs::msg::StateTrajectory>::SharedPtr   _referenceTraj_sub; /** Subscriber to the target predicted trajectory. Referene trajectory of the MPC */
   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   _referencePoses_sub; /** Subscriber to the target predicted poses. Referene trajectory of the MPC */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _referencePath_sub; /** Subscriber to the target predicted Path. Referene trajectory of the MPC */

   rclcpp::Publisher<custom_trajectory_msgs::msg::StateTrajectory>::SharedPtr _desired_traj_pub; /** Desired trajectory sent to the trajectory planner/sampler */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       _poseHistory_pub; /** ROS Publisher for _posehistory_vector */
   rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr _multiDofTraj_pub; /** To publish first MPC control solution to the geometric controller */
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _yaw_marker_pub;

   bool                          _debug;
   double                        _dt;                    /** Prediction time step in seconds */
   std::string                   _reference_frame_id;    /** Name of the map (inertial) frame, where the robot localizes */

   int                           _mpcWindow;

   Eigen::MatrixXd               _referenceTraj;         /** Target's predicted trajectory, over _mpcWindow. Received from the target predictor node*/
   MatX_12STATE                  _current_state;   /** Current state (position, velocity, acceleration) */
   Eigen::Vector3d               _current_accel;   /** Latest acceleration measurements. Will be added to _current_drone_state */
   bool                          _state_received;  /** True if a first measurment is received. Used for initialization*/
   builtin_interfaces::msg::Time _state_last_t;    /** Last time stamp of _current_state */
   builtin_interfaces::msg::Time _state_current_t; /** Current time stamp of _current_state */
   bool                          _target_traj_received;  /** Flag to indicate whether the first target trajectory is received */

   custom_trajectory_msgs::msg::StateTrajectory _solution_traj_msg; /** ROS message for the optimal trajectory, position, velocity, acceleration, max velocity, max acceleration */
   builtin_interfaces::msg::Time _ref_traj_last_t;       /** Time stamp of the last reference trajectory */

   bool                          _pub_pose_path;         /** Whether to publish MPC predicted path for visualization in RViz */
   std::vector<geometry_msgs::msg::PoseStamped> _posehistory_vector; /** Holds the predicted positions of the MPC, for visualization */

   trajectory_msgs::msg::MultiDOFJointTrajectory _multidof_msg;

   MPC12STATE *_mpc; /** MPC object */

   /**
    * @brief Odometry callback
    * @param msg nav_msgs::msg::Odometry
   */
   void odomCallback(const nav_msgs::msg::Odometry & msg);
   /**
    * @brief IMU callback to get current velocity and acceleration
    * @param msg sensor_msgs::msg::Imu
   */
   void imuCallback(const sensor_msgs::msg::Imu & msg);
   /**
    * @brief Callback for the reference trajectory as PoseArray
    * @param msg geometry_msgs::msg::PoseArray
   */
   void refPosesCallback(const geometry_msgs::msg::PoseArray & msg);
   /**
    * @brief Callback for the reference trajectory as Path
    * @param msg geometry_msgs::msg::PoseArray
   */
   void refPathCallback(const nav_msgs::msg::Path & msg);
   /**
    * @brief Callback of the MPC reference trajectory, which is expected to be published by traj_predictor node.
    * Updates _referenceTraj
    * @param msg custom_trajectory_msgs::msg::StateTrajectory
    */
   void refTrajCallback(const custom_trajectory_msgs::msg::StateTrajectory & msg);

   void extractSolution(void);

   bool mpcROSLoop(void);

   void pubPoseHistory(void);
   void pubMultiDofTraj(void); /** @todo implement */

};

#endif // MPC_12STATE_ROS_H