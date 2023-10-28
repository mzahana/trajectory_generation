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
#include "trajectory_generation/mpc_12state_ros.hpp"

MPCROS::MPCROS(): Node("mpc_12state_trajectory_generator")
{
    _mpc = new MPC12STATE();

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

    this->declare_parameter("xy_state_weight", 7000.0);
    _mpc->setXYStateWeight(this->get_parameter("xy_state_weight").get_parameter_value().get<double>());

    this->declare_parameter("z_state_weight", 7000.0);
    _mpc->setZStateWeight(this->get_parameter("z_state_weight").get_parameter_value().get<double>());

    this->declare_parameter("yaw_state_weight", 7000.0);
    _mpc->setYawStateWeight(this->get_parameter("yaw_state_weight").get_parameter_value().get<double>());

    this->declare_parameter("xy_input_weight", 10.0);
    _mpc->setXYInputWeight(this->get_parameter("xy_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("z_input_weight", 10.0);
    _mpc->setZInputWeight(this->get_parameter("z_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("yaw_input_weight", 0.1);
    _mpc->setYawInputWeight(this->get_parameter("yaw_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("xy_smooth_input_weight", 10.0);
    _mpc->setXYSmoothInputWeight(this->get_parameter("xy_smooth_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("z_smooth_input_weight", 10.0);
    _mpc->setZSmoothInputWeight(this->get_parameter("z_smooth_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("yaw_smooth_input_weight", 1.0);
    _mpc->setYawSmoothInputWeight(this->get_parameter("yaw_smooth_input_weight").get_parameter_value().get<double>());

    this->declare_parameter("enable_control_smoothing", false);
    _mpc->enableControlSmoothing(this->get_parameter("enable_control_smoothing").get_parameter_value().get<bool>());

    this->declare_parameter("alt_above_target", 1.0);
    _mpc->setAltAboveTarget(this->get_parameter("alt_above_target").get_parameter_value().get<double>());

    this->declare_parameter("minimum_altitude", 1.0);
    _mpc->setMinimumAltitude(this->get_parameter("minimum_altitude").get_parameter_value().get<double>());

   this->declare_parameter("xy_max_velocity", 12.0);
   _mpc->setXYMaxVel( this->get_parameter("xy_max_velocity").get_parameter_value().get<double>());

   this->declare_parameter("z_max_velocity", 6.0);
   _mpc->setZMaxVel( this->get_parameter("z_max_velocity").get_parameter_value().get<double>());

   this->declare_parameter("yaw_max_velocity", 5.0);
   _mpc->setYawMaxVel( this->get_parameter("yaw_max_velocity").get_parameter_value().get<double>());

   this->declare_parameter("xy_max_acceleration", 5.0);
   _mpc->setXYMaxAccel( this->get_parameter("xy_max_acceleration").get_parameter_value().get<double>());

   this->declare_parameter("z_max_acceleration", 5.0);
   _mpc->setZMaxAccel( this->get_parameter("z_max_acceleration").get_parameter_value().get<double>());

   this->declare_parameter("yaw_max_acceleration", 10.0);
   _mpc->setYawMaxAccel( this->get_parameter("yaw_max_acceleration").get_parameter_value().get<double>());

   this->declare_parameter("xy_max_jerk", 10.0);
   _mpc->setXYMaxJerk( this->get_parameter("xy_max_jerk").get_parameter_value().get<double>());

   this->declare_parameter("z_max_jerk", 10.0);
   _mpc->setZMaxJerk( this->get_parameter("z_max_jerk").get_parameter_value().get<double>());

   this->declare_parameter("yaw_max_jerk", 10.0);
   _mpc->setYawMaxJerk( this->get_parameter("yaw_max_jerk").get_parameter_value().get<double>());


   if(! _mpc->initMPCProblem())
   {
        RCLCPP_INFO(this->get_logger(),"[MPCROS] Could not initialize MPC problem");
        return;
   }
   // Resize some variables in the initialization phase
   _referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_STATES*(_mpcWindow+1),1);
   _solution_traj_msg.states.resize(_mpcWindow);
   // Update _posehistory_vector for visualiztion
   _posehistory_vector.resize(_mpcWindow+1);

   RCLCPP_INFO(this->get_logger(), "[MPCROS] Creating subscribers and publishers");
   _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "mpc/in/odom", rclcpp::SensorDataQoS(), std::bind(&MPCROS::odomCallback, this, _1));

   _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "mpc/in/imu", rclcpp::SensorDataQoS(), std::bind(&MPCROS::imuCallback, this, _1));    

   _referenceTraj_sub = this->create_subscription<custom_trajectory_msgs::msg::StateTrajectory>(
      "mpc/in/ref_traj", 10, std::bind(&MPCROS::refTrajCallback, this, _1));

   _referencePoses_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "mpc/in/ref_traj_poses", 10, std::bind(&MPCROS::refPosesCallback, this, _1));

   _referencePath_sub = this->create_subscription<nav_msgs::msg::Path>(
      "mpc/in/ref_traj_path",  rclcpp::SensorDataQoS(), std::bind(&MPCROS::refPathCallback, this, _1));


   _poseHistory_pub = this->create_publisher<nav_msgs::msg::Path>("mpc/out/path", 10);
   _desired_traj_pub = this->create_publisher<custom_trajectory_msgs::msg::StateTrajectory>("mpc/out/trajectory", 10);
   _multiDofTraj_pub = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("mpc/out/trajectory_command", 10);

   RCLCPP_INFO(this->get_logger(), "[MPCROS] Done with creating subscribers and publishers");

   RCLCPP_INFO(this->get_logger(),"[MPCROS] will execute once reference trajectory is published...");

   return;    
}

MPCROS::~MPCROS()
{
    delete _mpc;
}

void
MPCROS::odomCallback(const nav_msgs::msg::Odometry & msg)
{
   // RCLCPP_INFO(this->get_logger(), "[MPCROS::odomCallback] Got odom msg");
    if(!_state_received)
    {
      RCLCPP_INFO(this->get_logger(), "[MPCROS::odomCallback] Received initial state");
      _state_received = true;
    }

   _state_current_t = msg.header.stamp;
   _reference_frame_id = msg.header.frame_id;
   _current_state.setZero();
   // TODO Sync time stamps of _current_drone_accel with pose, before adding it to _current_drone_state
   //  state order: [x, vx, ax, y, vy, ay, z, vz, az, yaw, v_yaw, a_yaw]

   tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );
    // Convert the quaternion to a 3x3 rotation matrix
    tf2::Matrix3x3 m(q);

    // Extract the Euler angles from the rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
      // _current_state << msg.pose.pose.position.x,
      //                   msg.twist.twist.linear.x,
      //                   _current_accel(0),
      //                   msg.pose.pose.position.y,
      //                   msg.twist.twist.linear.y,
      //                   _current_accel(1),
      //                   msg.pose.pose.position.z,
      //                   msg.twist.twist.linear.z,
      //                   _current_accel(2),
      //                   yaw,
      //                   msg.twist.twist.angular.z,
      //                   0.0;

      _current_state << msg.pose.pose.position.x,
                        0,
                        0,
                        msg.pose.pose.position.y,
                        0,
                        0,
                        msg.pose.pose.position.z,
                        0,
                        0,
                        yaw,
                        0,
                        0.0;


}

void
MPCROS::imuCallback(const sensor_msgs::msg::Imu & msg)
{
   // RCLCPP_INFO(this->get_logger(), "[MPCROS::imuCallback] Got IMU msg");
    // WARNING The following is WRONG!!!!
   // TODO: Need to transform IMU from body frame to local frame, and remove gravity magnitude from z axis
   // Get acceleration values
   // _current_accel.setZero();
   _current_accel << msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z;
}

void MPCROS::refPathCallback(const nav_msgs::msg::Path & msg)
{
   // RCLCPP_INFO(this->get_logger(),"Executing refPathCallback");
   // WARNING The rate of MPC is affected by
   // the rate of Odom (drone state) (default 30Hz from mavros/local_position/odom),
   // and _referenceTraj
   // and the size of MPC problem
   // The MPC rate will be close to the max(odom, _referenceTraj, MPC execution time)
   if(!_state_received)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refPathCallback] An initial state is not received. Check Odom. Returning");
      return;
   }
   // Make sure we have a new reference trajectory
   double d2 = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
   double d1 = _ref_traj_last_t.sec + static_cast<double>(_ref_traj_last_t.nanosec) * 1e-9;
   auto dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refPathCallback] Received an old reference trajectory");
      return;
   }
   _ref_traj_last_t = msg.header.stamp;

   // Make sure we have a new drone state measurement
   d2 = _state_current_t.sec + static_cast<double>(_state_current_t.nanosec) * 1e-9;
   d1 = _state_last_t.sec + static_cast<double>(_state_last_t.nanosec) * 1e-9;
   dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refPathCallback] Received an old state. Return");
      return;
   }
   _state_last_t = _state_current_t;

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.poses.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refPathCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.poses.size(), _mpcWindow+1);
      return;
   }

   // RCLCPP_INFO(this->get_logger(), "Setting _referenceTraj");
   // Update _referenceTraj
   _referenceTraj.setZero();
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _referenceTraj(i*NUM_OF_STATES+0,0) = msg.poses[i].pose.position.x;
      _referenceTraj(i*NUM_OF_STATES+3,0) = msg.poses[i].pose.position.y;
      _referenceTraj(i*NUM_OF_STATES+6,0) = msg.poses[i].pose.position.z;
   }

   // set current state, refTraj, solve, extract solution, and publish msgs
   mpcROSLoop();

}

void
MPCROS::refPosesCallback(const geometry_msgs::msg::PoseArray & msg)
{
   // WARNING The rate of MPC is affected by
   // the rate of Odom (drone state) (default 30Hz from mavros/local_position/odom),
   // and _referenceTraj
   // and the size of MPC problem
   // The MPC rate will be close to the max(odom, _referenceTraj, MPC execution time)

   if(!_state_received)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refPosesCallback] Initial state is not received. Check Odom. Returning");
      return;
   }
   // Make sure we have a new reference trajectory
   double d2 = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
   double d1 = _ref_traj_last_t.sec + static_cast<double>(_ref_traj_last_t.nanosec) * 1e-9;
   auto dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refPosesCallback] Received an old reference trajectory");
      return;
   }
   _ref_traj_last_t = msg.header.stamp;

   // Make sure we have a new drone state measurement
   d2 = _state_current_t.sec + static_cast<double>(_state_current_t.nanosec) * 1e-9;
   d1 = _state_last_t.sec + static_cast<double>(_state_last_t.nanosec) * 1e-9;
   dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Received an old state. Return");
      return;
   }
   _state_last_t = _state_current_t;

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.poses.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refTrajCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.poses.size(), _mpcWindow+1);
      return;
   }

//    // Update _referenceTraj
   _referenceTraj.setZero();
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _referenceTraj(i*NUM_OF_STATES+0,0) = msg.poses[i].position.x;
      _referenceTraj(i*NUM_OF_STATES+3,0) = msg.poses[i].position.y;
      _referenceTraj(i*NUM_OF_STATES+6,0) = msg.poses[i].position.z;
   }

   // set current state, refTraj, solve, extract solution, and publish msgs
   mpcROSLoop();
}

void
MPCROS::refTrajCallback(const custom_trajectory_msgs::msg::StateTrajectory & msg)
{
   // WARNING The rate of MPC is affected by
   // the rate of Odom (drone state) (default 30Hz from mavros/local_position/odom),
   // and _referenceTraj
   // and the size of MPC problem
   // The MPC rate will be close to the max(odom, _referenceTraj, MPC execution time)

   if(!_state_received)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Initial state is not received. Check Odom. Returning");
      return;
   }
   // Make sure we have a new reference trajectory
   double d2 = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
   double d1 = _ref_traj_last_t.sec + static_cast<double>(_ref_traj_last_t.nanosec) * 1e-9;
   auto dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Received an old reference trajectory");
      return;
   }
   _ref_traj_last_t = msg.header.stamp;

   // Make sure we have a new drone state measurement
   d2 = _state_current_t.sec + static_cast<double>(_state_current_t.nanosec) * 1e-9;
   d1 = _state_last_t.sec + static_cast<double>(_state_last_t.nanosec) * 1e-9;
   dt = d2 - d1;
   if (dt <= 0.0)
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::refTrajCallback] Received an old state. Return");
      return;
   }
   _state_last_t = _state_current_t;

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
      _referenceTraj(i*NUM_OF_STATES+2,0) = msg.states[i].acceleration.x;
      _referenceTraj(i*NUM_OF_STATES+3,0) = msg.states[i].position.y;
      _referenceTraj(i*NUM_OF_STATES+4,0) = msg.states[i].velocity.y;
      _referenceTraj(i*NUM_OF_STATES+5,0) = msg.states[i].acceleration.y;
      _referenceTraj(i*NUM_OF_STATES+6,0) = msg.states[i].position.z;
      _referenceTraj(i*NUM_OF_STATES+7,0) = msg.states[i].velocity.z;
      _referenceTraj(i*NUM_OF_STATES+8,0) = msg.states[i].acceleration.z;
   }
   
   // set current state, refTraj, solve, extract solution, and publish msgs
   mpcROSLoop();

   return;
}

void
MPCROS::extractSolution(void)
{
   auto nx = NUM_OF_STATES;
   auto nu = NUM_OF_INPUTS;

   auto optimal_state_traj = _mpc->getOptimalStateTraj();
   auto optimal_control_traj = _mpc->getOptimalControlTraj();

   // Used to compute quaternion from yaw
   tf2::Quaternion q_yaw;

   geometry_msgs::msg::PoseStamped pose_msg;
   double start_t = this->now().seconds();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      int64_t t = static_cast<int64_t>((start_t + (i*_dt))*1e9);
      pose_msg.header.frame_id=_reference_frame_id;
      pose_msg.header.stamp = rclcpp::Time(t);
      pose_msg.pose.position.x = optimal_state_traj(i*nx+0);
      pose_msg.pose.position.y = optimal_state_traj(i*nx+3);
      pose_msg.pose.position.z = optimal_state_traj(i*nx+6);

      // compute yaw as quaternion
      q_yaw.setRPY(0, 0, optimal_state_traj( i*nx+9 ));

      pose_msg.pose.orientation.w=q_yaw.w();
      pose_msg.pose.orientation.x=q_yaw.x();
      pose_msg.pose.orientation.y=q_yaw.y();
      pose_msg.pose.orientation.z=q_yaw.z();
      // _posehistory_vector.insert(_posehistory_vector.begin(), pose_msg);
      _posehistory_vector[i] = pose_msg;

      if(i<_mpcWindow)
      {
         // Fill ROS msg
         _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         _solution_traj_msg.states[i].position.x = optimal_state_traj( (i+1)*nx+0 );
         _solution_traj_msg.states[i].velocity.x = optimal_state_traj( (i+1)*nx+1 );
         _solution_traj_msg.states[i].acceleration.x = optimal_state_traj( (i+1)*nx+2 );
         _solution_traj_msg.states[i].position.y = optimal_state_traj( (i+1)*nx+3 );
         _solution_traj_msg.states[i].velocity.y = optimal_state_traj( (i+1)*nx+4 );
         _solution_traj_msg.states[i].acceleration.y = optimal_state_traj( (i+1)*nx+5 );
         _solution_traj_msg.states[i].position.z = optimal_state_traj( (i+1)*nx+6 );               
         _solution_traj_msg.states[i].velocity.z = optimal_state_traj( (i+1)*nx+7 );         
         _solution_traj_msg.states[i].acceleration.z = optimal_state_traj( (i+1)*nx+8 );

         _solution_traj_msg.states[i].yaw = optimal_state_traj( (i+1)*nx+9 );
         _solution_traj_msg.states[i].yaw_speed = optimal_state_traj( (i+1)*nx+10 );
         _solution_traj_msg.states[i].yaw_acceleration = optimal_state_traj( (i+1)*nx+11 );

         _solution_traj_msg.states[i].jerk.x = optimal_control_traj( i*nu+0 );
         _solution_traj_msg.states[i].jerk.y = optimal_control_traj( i*nu+1 );
         _solution_traj_msg.states[i].jerk.z = optimal_control_traj( i*nu+2 );
         _solution_traj_msg.states[i].yaw_jerk = optimal_control_traj( i*nu+3 );
      }
   }

   
   _solution_traj_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_t*1e9));
   _solution_traj_msg.header.frame_id = _reference_frame_id;

   // First control input (and corresponding state)
   // This can be used by a lower level controller

   _multidof_msg.header.frame_id = _reference_frame_id;
   _multidof_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_t*1e9));
   _multidof_msg.points.resize(1);
   _multidof_msg.points[0].transforms.resize(1);
   _multidof_msg.points[0].velocities.resize(1);
   _multidof_msg.points[0].accelerations.resize(1);

   _multidof_msg.points[0].transforms[0].translation.x = _solution_traj_msg.states[0].position.x;
   _multidof_msg.points[0].transforms[0].translation.y = _solution_traj_msg.states[0].position.y;
   _multidof_msg.points[0].transforms[0].translation.z = _solution_traj_msg.states[0].position.z;
   _multidof_msg.points[0].velocities[0].linear.x = _solution_traj_msg.states[0].velocity.x;
   _multidof_msg.points[0].velocities[0].linear.y = _solution_traj_msg.states[0].velocity.y;
   _multidof_msg.points[0].velocities[0].linear.z = _solution_traj_msg.states[0].velocity.z;
   _multidof_msg.points[0].accelerations[0].linear.x = _solution_traj_msg.states[0].acceleration.x;
   _multidof_msg.points[0].accelerations[0].linear.y = _solution_traj_msg.states[0].acceleration.y;
   _multidof_msg.points[0].accelerations[0].linear.z = _solution_traj_msg.states[0].acceleration.z;

   q_yaw.setRPY(0, 0, _solution_traj_msg.states[0].yaw);
   _multidof_msg.points[0].transforms[0].rotation.x = q_yaw.x();
   _multidof_msg.points[0].transforms[0].rotation.y = q_yaw.y();
   _multidof_msg.points[0].transforms[0].rotation.z = q_yaw.z();
   _multidof_msg.points[0].transforms[0].rotation.w = q_yaw.w();

   return;

}


bool MPCROS::mpcROSLoop(void)
{
   if(!_mpc->setCurrentState(_current_state))
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::mpcROSLoop] Could not set _current_state");
      return false;
   }
   
   if(!_mpc->setReferenceTraj(_referenceTraj))
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::mpcROSLoop] Could not set _referenceTraj");
      return false;
   }
   

   //Solve MPC problem
   if(!_mpc->mpcLoop())
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::mpcROSLoop] Error in mpcLooop()");
      // std::cout << "current_state\n" << _current_state << "\n";
      // std::cout << "refTraj[0]:\n" << _referenceTraj.block(0,0, NUM_OF_STATES,1) << "\n";
      return false;
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
   pubMultiDofTraj();

   return true;
}

void MPCROS::pubPoseHistory(void)
{
   nav_msgs::msg::Path msg;

   msg.header.stamp = _posehistory_vector[0].header.stamp;
   msg.header.frame_id = _reference_frame_id;
   msg.poses = _posehistory_vector;

   _poseHistory_pub->publish(msg);

   return;
}

void MPCROS::pubMultiDofTraj(void)
{
   if(_multidof_msg.points.size() > 0)
      _multiDofTraj_pub->publish(_multidof_msg);
}