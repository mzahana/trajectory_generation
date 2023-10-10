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
#include "trajectory_generation/mpc_ros.hpp"

MPCROS::MPCROS(): Node("mpc_trajectory_generator")
{
    _mpc = new MPC();

    this->declare_parameter("dt_pred", 0.05);
    _mpc->setDt(this->get_parameter("dt_pred").get_parameter_value().get<double>());

    this->declare_parameter("debug", false);
    _debug = this->get_parameter("debug").get_parameter_value().get<bool>();
    _mpc->setDebug(_debug);

    this->declare_parameter("pub_pose_path", false);
    _pub_pose_path = this->get_parameter("pub_pose_path").get_parameter_value().get<bool>();

    this->declare_parameter("mpc_window", 10);
    _mpcWindow = this->get_parameter("mpc_window").get_parameter_value().get<int>();
    _mpc->setMPCWindow(_mpcWindow);

    this->declare_parameter("state_weight", 1.0);
    _mpc->setStateWeight(this->get_parameter("state_weight").get_parameter_value().get<double>());

    this->declare_parameter("input_weight", 0.1);
    _mpc->setInputWeight(this->get_parameter("input_weight").get_parameter_value().get<double>());

    this->declare_parameter("smooth_input_weight", 0.1);
    _mpc->setSmoothInputWeight(this->get_parameter("smooth_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("enable_control_smoothing", true);
    _mpc->enableControlSmoothing(this->get_parameter("enable_control_smoothing").get_parameter_value().get<bool>());

    this->declare_parameter("alt_above_target", 1.0);
    _mpc->setAltAboveTarget(this->get_parameter("alt_above_target").get_parameter_value().get<double>());

    this->declare_parameter("minimum_altitude", 1.0);
    _mpc->setMinimumAltitude(this->get_parameter("minimum_altitude").get_parameter_value().get<double>());

    std::vector<double> maxVelVec {10.0, 10.0, 10.0};
   this->declare_parameter("max_velocity", maxVelVec);
   _mpc->setMaxVel( this->get_parameter("max_velocity").get_parameter_value().get<std::vector<double>>());

   std::vector<double> maxAccVec {3.0, 3.0, 3.0};
   this->declare_parameter("max_acceleration", maxAccVec);
   _mpc->setMaxAccel( this->get_parameter("max_acceleration").get_parameter_value().get<std::vector<double>>());

   // This is only used with 9dof model, needs mpc_9dof.hpp
   // std::vector<double> maxJerkVec {2.0, 2.0, 2.0};
   // this->declare_parameter("max_jerk", maxJerkVec);
   // _mpc->set_maxJerk( this->get_parameter("max_jerk").get_parameter_value().get<std::vector<double>>());

   if(! _mpc->initMPCProblem())
   {
        RCLCPP_INFO(this->get_logger(),"[MPCROS] Could not initialize MPC problem");
        return;
   }

   RCLCPP_INFO(this->get_logger(),"[MPCROS] will execute once reference trajectory is published...");


   _droneOdom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "px4_ros/in/odom", rclcpp::SensorDataQoS(), std::bind(&MPCROS::odomCallback, this, _1));

   _droneImu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "px4_ros/in/imu", rclcpp::SensorDataQoS(), std::bind(&MPCROS::imuCallback, this, _1));    

   _referenceTraj_sub = this->create_subscription<custom_trajectory_msgs::msg::StateTrajectory>(
      "traj_predictor/in/ref_traj", 10, std::bind(&MPCROS::refTrajCallback, this, _1));


   _poseHistory_pub = this->create_publisher<nav_msgs::msg::Path>("mpc_tracker/out/path", 10);
   _desired_traj_pub = this->create_publisher<custom_trajectory_msgs::msg::StateTrajectory>("mpc_tracker/out/trajectory", 10);
   _multiDofTraj_pub = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("mpc_tracker/command/trajectory", 10);
    
}

MPCROS::~MPCROS()
{
    delete _mpc;
}

void
MPCROS::odomCallback(const nav_msgs::msg::Odometry & msg)
{
    if(!_drone_state_received)
      _drone_state_received = true;

   _drone_state_current_t = msg.header.stamp;
   _current_drone_state.setZero();
   // TODO Sync time stamps of _current_drone_accel with pose, before adding it to _current_drone_state
   //  state order: [px, py, pz, vx, vy, vz]
      _current_drone_state << msg.pose.pose.position.x,
                              msg.pose.pose.position.y, 
                              msg.pose.pose.position.z,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z;
}

void
MPCROS::imuCallback(const sensor_msgs::msg::Imu & msg)
{
    // WARNING The following is WRONG!!!!
   // TODO: Need to transform IMU from body frame to local frame, and remove gravity magnitude from z axis
   // Get acceleration values
   _current_drone_accel.setZero();
   _current_drone_accel << msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z;
}

void
MPCROS::refTrajCallback(const custom_trajectory_msgs::msg::StateTrajectory & msg)
{
    // WARNING The rate of MPC is affected by
   // the rate of Odom (drone state) (default 30Hz from mavros/local_position/odom),
   // and _referenceTraj
   // and the size of MPC problem
   // The MPC rate will be close to the max(odom, _referenceTraj, MPC execution time)

   if(!_drone_state_received)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Drone state is not received. Check Odom. Returning");
      return;
   }
   // Make sure we have a new reference trajectory
   auto dt = rclcpp::Time(msg.header.stamp).seconds() - rclcpp::Time(_ref_traj_last_t).seconds();
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Received an old reference trajectory");
      return;
   }
   _ref_traj_last_t = msg.header.stamp;

   // Make sure we have a new drone state measurement
   dt = rclcpp::Time(_drone_state_current_t).seconds() - rclcpp::Time(_drone_state_last_t).seconds();
//    dt = (_drone_state_current_t - _drone_state_last_t).toSec();
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Received an old drone state. Return");
      return;
   }
   _drone_state_last_t = _drone_state_current_t;
   if(!_mpc->setCurrentState(_current_drone_state)) return;

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.states.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refTrajCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.states.size(), _mpcWindow+1);
      return;
   }

//    // Update _referenceTraj
   _referenceTraj.setZero();
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _referenceTraj(i*NUM_OF_STATES+0,0) = msg.states[i].position.x;
      _referenceTraj(i*NUM_OF_STATES+1,0) = msg.states[i].velocity.x;

      _referenceTraj(i*NUM_OF_STATES+2,0) = msg.states[i].position.y;
      _referenceTraj(i*NUM_OF_STATES+3,0) = msg.states[i].velocity.y;

      _referenceTraj(i*NUM_OF_STATES+4,0) = msg.states[i].position.z;
      _referenceTraj(i*NUM_OF_STATES+5,0) = msg.states[i].velocity.z;
   }
   if(!_mpc->setReferenceTraj(_referenceTraj)) return;
   

//    /* Solve MPC problem ! */
   if(!_mpc->mpcLoop())
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Error in mpcLooop()");
      return;
   }

   // Extract solutions, updates _optimal_state_traj, _optimal_control_traj, _mpc_ctrl_sol
   extractSolution();

   // Publish desired trajectory, visualization, ... etc
   if(_pub_pose_path)
   {
      pubPoseHistory();
   }

   // Publish optimal trajectory
   _desired_traj_pub->publish(_solution_traj_msg);
   // Publish first control solution u[0] to a lower level controller
   // pubMultiDofTraj();
}

void
MPCROS::extractSolution(void)
{
   auto nx = NUM_OF_STATES;
   auto nu = NUM_OF_INPUTS;

   auto optimal_state_traj = _mpc->getOptimalStateTraj();
   auto optimal_control_traj = _mpc->getOptimalControlTraj();

   _solution_traj_msg.states.resize(_mpcWindow);
   // Update _posehistory_vector for visualiztion
   _posehistory_vector.resize(_mpcWindow+1);
   geometry_msgs::msg::PoseStamped pose_msg;
   double start_t = this->now().seconds();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      int64_t t = static_cast<int64_t>((start_t + (i*_dt))*1e9);
      pose_msg.header.frame_id=_reference_frame_id;
      pose_msg.header.stamp = rclcpp::Time(t);
      pose_msg.pose.position.x = optimal_state_traj(i*nx+0);
      pose_msg.pose.position.y = optimal_state_traj(i*nx+1);
      pose_msg.pose.position.z = optimal_state_traj(i*nx+2);
      pose_msg.pose.orientation.w=1.0; // Keep 0 rotation, for now
      // _posehistory_vector.insert(_posehistory_vector.begin(), pose_msg);
      _posehistory_vector[i] = pose_msg;

      if(i<_mpcWindow)
      {
         // Fill ROS msg
         _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         _solution_traj_msg.states[i].position.x = optimal_state_traj( (i+1)*nx+0 );
         _solution_traj_msg.states[i].position.y = optimal_state_traj( (i+1)*nx+1 );
         _solution_traj_msg.states[i].position.z = optimal_state_traj( (i+1)*nx+2 );
         _solution_traj_msg.states[i].velocity.x = optimal_state_traj( (i+1)*nx+3 );
         _solution_traj_msg.states[i].velocity.y = optimal_state_traj( (i+1)*nx+4 );
         _solution_traj_msg.states[i].velocity.z = optimal_state_traj( (i+1)*nx+5 );
         _solution_traj_msg.states[i].acceleration.x = optimal_control_traj(i*nu+0);
         _solution_traj_msg.states[i].acceleration.y = optimal_control_traj(i*nu+1);
         _solution_traj_msg.states[i].acceleration.z = optimal_control_traj(i*nu+2);
      }
   }

   
   _solution_traj_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_t*1e9));
   _solution_traj_msg.header.frame_id = _reference_frame_id;
   _solution_traj_msg.max_acceleration = _mpc->getMaxAccel().maxCoeff();
   _solution_traj_msg.max_velocity = _mpc->getMaxVel().maxCoeff();

   return;

}


// void MPCROS::testCases(void)
// {
//     if (_debug)
//    {
//       RCLCPP_INFO(this->get_logger(), "[MPCROS::testCase] Starting test Case. Target is hovering at 1.0m altitude. Follower is at rest on the ground.");
//    }
//    if(_debug)
//    {
//       RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Setting the reference trajectory");
//    }
   
//    // For timing this function
//    // Get the start time.
//     auto start = std::chrono::high_resolution_clock::now();

//    // Create a _referenceTraj of a target hovering at 1.0m altitude from ground
//    _referenceTraj.resize(_mpc->get_num_of_states()*(_mpcWindow+1),1);
//    _referenceTraj.setZero();
//    for (int i=0; i<_mpcWindow+1; i++)
//    {
//       if (_use_6dof_model)
//       {
//          // states order: [px, vx, py, vy, pz, vz]
//          _referenceTraj(i*_mpc->get_num_of_states()+4,0) = 1.0; // z coordinate at all times
//          _referenceTraj(i*_mpc->get_num_of_states()+0,0) = 0.1; // x coordinate at all time
//       }
//       else
//       {
//          // states order: [px, vx, ax, py, vy, ay, pz, vz, az]
//          _referenceTraj(i*_mpc->get_num_of_states()+6,0) = 1.0; // z coordinate at all times
//          _referenceTraj(i*_mpc->get_num_of_states()+0,0) = 0.1; // x coordinate at all times  
//       }
//    }

//    if(_debug)
//    {
//       RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Setting the drone state");
//    }
//    // Current drone state
//    _current_drone_state.setZero();
//    if (_use_6dof_model)
//    {
//       // state order: [px, vx, py, vy, pz, vz]
//       _current_drone_state(0,0)=0.1; // px
//       _current_drone_state(2,0)=-0.5; // py
//       _current_drone_state(4,0)=0.1; // pz
//    }
//    else
//    {
//       // state order: [px, vx, ax, py, vy ,ay, pz, vz, az]
//       _current_drone_state(0,0)=0.1; // px
//       _current_drone_state(3,0)=-0.5; // py
//       _current_drone_state(6,0)=0.1; // pz
//    }

//    if(!_mpc->set_current_drone_state(_current_drone_state))
//    {
//       RCLCPP_ERROR(this->get_logger(), "[MPCROS::testCase] Could not set current drone state. Return");
//       return;
//    }


//    if(_debug)
//    {
//       RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Updating MPC reference trajectory");
//    }
   
//    // Update QP
//    if(!_mpc->set_referenceTraj(_referenceTraj))
//    {
//       RCLCPP_ERROR(this->get_logger(), "[MPCROS::testCase] Failed to set reference trajectory. Return");
//       return;
//    }

//    if(!_mpc->updateMPC())
//    {
//       RCLCPP_ERROR(this->get_logger(), "Could not updateMPC. Return");
//       return;
//    }

   
//    // Solve MPC
//    if(_debug)
//    {
//       RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Solving MPC problem");
//    }
//    if(!_mpc->mpcLoop())
//    {
//       RCLCPP_ERROR(this->get_logger(),"[MPCROS::testCase] MPC solution is not found");
//       return;
//    }

//    if(_debug)
//    {
//      RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Extracting solution");
//    }
//    // Extract solutions, updates _optimal_state_traj, _optimal_control_traj, _mpc_ctrl_sol
//    if (_use_6dof_model)
//       extractSolution6Dof();
//    else
//       extractSolution();

//    _desired_traj_pub->publish(_solution_traj_msg);


//    // if(_debug)
//    // {
//    //    std::cout << "[MPCTracker::testCase] Optimal control: " << std::endl << _mpc_ctrl_sol << std::endl;
//    //    std::cout << "[MPCTracker::testCase] Optimal state trajectory: " << std::endl << _optimal_state_traj << std::endl;
//    //    std::cout << "[MPCTracker::testCase] Optimal control trajectory: " << std::endl << _optimal_control_traj << std::endl;
//    // }

//    // Apply the optimal control inputs to the initial condition
//    RCLCPP_INFO(this->get_logger(), "[MPCROS::testCase] Apply the optimal control inputs to the initial condition");
//    auto x0 = _current_drone_state;
//    for (int i=0; i < _mpcWindow; i++)
//    {
//       x0 = _mpc->get_transition_matrix()*x0 + _mpc->get_input_matrix()*_mpc->get_optimal_control_traj().segment(i*_mpc->get_num_of_inputs(), _mpc->get_num_of_inputs());
//    }

//    // Publish desired trajectory, visualization, ... etc
//    if(_pub_pose_path)
//    {
//       if(_debug)
//       {
//          RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Publishing predicted position path");
//       }
//       pubPoseHistory();
//    }

//    if (_debug)
//    {
//       RCLCPP_INFO(this->get_logger(),"[MPCROS::testCase] Test case is Done!");
//    }

//    // Get the end time.
//    auto end = std::chrono::high_resolution_clock::now();
//    // Compute the difference between the end time and the start time.
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//    double time_in_seconds = duration.count() / 1000.0;
//    RCLCPP_INFO(this->get_logger()," Test case took %f seconds.", time_in_seconds);

//    // Compare the simulated satate to the one computed by the optimization
//    auto x_N_opt = _mpc->get_optimal_state_traj().segment(_mpc->get_num_of_states()*_mpcWindow, _mpc->get_num_of_states());
//    std::cout << "[MPCROS::testCase] Simulated state at time step t= " << _mpcWindow << "\n" << x0 << "\n";
//    std::cout << "[MPCROS::testCase] Optimal state at time step t= " << _mpcWindow << "\n" << x_N_opt << "\n";
//    std::cout << "[MPCROS::testCase] Error between simulated and optimal final state at t= " << _mpcWindow << "\n" << (x_N_opt-x0).norm() << "\n";


//    // if(_plot)
//    //    plotSolutions();
//    return;
// }


void MPCROS::pubPoseHistory(void)
{
   nav_msgs::msg::Path msg;

   msg.header.stamp = _posehistory_vector[0].header.stamp;
   msg.header.frame_id = _reference_frame_id;
   msg.poses = _posehistory_vector;

   _poseHistory_pub->publish(msg);
}