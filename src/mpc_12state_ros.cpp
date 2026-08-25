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

    // nav_msgs/Odometry twist is expressed in child_frame_id. For mavros
    // local_position/odom that is the body frame, so it must be rotated to world.
    this->declare_parameter("odom_twist_in_body_frame", true);
    _odom_twist_in_body_frame = this->get_parameter("odom_twist_in_body_frame").get_parameter_value().get<bool>();

    // Which sample of the optimal horizon is published as the controller
    // setpoint. This node used to publish the LAST sample (a full horizon
    // ahead), which is a far "carrot" the geometric controller tracks poorly.
    // A near sample (1-3 steps) is the correct receding-horizon command.
    this->declare_parameter("cmd_sample_index", 2);
    _cmd_sample_index = this->get_parameter("cmd_sample_index").get_parameter_value().get<int>();

    // --- Fixed-rate solving & health monitoring (T1.7) ---------------------
    // The MPC used to solve only inside the reference callback, which tied the
    // control rate (and its liveness) to whatever the predictor was doing. The
    // node now owns its own rate; callbacks only store the latest data.
    // mpc_rate <= 0 restores the legacy solve-in-callback behaviour.
    this->declare_parameter("mpc_rate", 20.0);          // Hz
    _mpc_rate = this->get_parameter("mpc_rate").get_parameter_value().get<double>();

    // Reference freshness gate. This is also what hands command authority over
    // to terminal guidance: below terminal_entry_range the guidance node stops
    // publishing the MPC reference, and the MPC must then FALL SILENT, because
    // terminal APN publishes to the same controller topic.
    //
    // At 0.5 s it did not. The guidance reference arrives at 50 Hz, so 0.5 s is
    // 25 missed messages, and for ~10 MPC cycles after handover BOTH sources
    // commanded the controller. Measured in SITL: two setpoints 1.25 ms apart
    // differing by 1.23 rad of yaw at R = 38.6 m, i.e. right at the 40 m entry.
    // The conflict is not limited to yaw -- the position/velocity/accel triples
    // disagree too.
    //
    // 0.15 s is still 7 missed reference messages, generous for jitter, but it
    // silences the MPC within ~3 of its own cycles.
    this->declare_parameter("ref_timeout", 0.15);       // seconds
    _ref_timeout = this->get_parameter("ref_timeout").get_parameter_value().get<double>();

    this->declare_parameter("odom_timeout", 0.3);       // seconds
    _odom_timeout = this->get_parameter("odom_timeout").get_parameter_value().get<double>();

    this->declare_parameter("max_consecutive_failures", 5);
    _max_consecutive_failures = this->get_parameter("max_consecutive_failures").get_parameter_value().get<int>();

    _consecutive_failures = 0;
    _solve_count = 0;
    _fail_count = 0;
    _last_solve_ms = 0.0;
    _max_solve_ms = 0.0;
    _have_ref = false;
    _last_ref_stamp = this->now();
    _last_odom_stamp = this->now();

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

   // Exact-penalty weight on state-constraint violations, as a MULTIPLE of the
   // per-axis state weight. State bounds (velocity, acceleration, altitude
   // floor) are soft so that a measurement outside the flight envelope can
   // never make the QP infeasible; this ratio is what keeps the relaxation at
   // exactly zero the rest of the time. Declared after the state weights so
   // that the per-axis penalties are derived from the values actually in use.
   // Wall-clock bound per QP solve. A late setpoint is worse for the control
   // loop than a dropped one: the controller handles a drop by holding, and
   // handles lateness not at all. 0 disables the bound.
   this->declare_parameter("qp_time_limit", 0.010);
   _mpc->setQPTimeLimit( this->get_parameter("qp_time_limit").get_parameter_value().get<double>());

   this->declare_parameter("soft_state_penalty_ratio", 3.0);
   _mpc->setSoftStatePenalty( this->get_parameter("soft_state_penalty_ratio").get_parameter_value().get<double>());


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
   _health_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("mpc/out/health", 10);

   RCLCPP_INFO(this->get_logger(), "[MPCROS] Done with creating subscribers and publishers");

   if(_mpc_rate > 0.0)
   {
      const auto period = std::chrono::duration<double>(1.0/_mpc_rate);
      _mpc_timer = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::nanoseconds>(period),
         std::bind(&MPCROS::mpcTimerCallback, this));
      RCLCPP_INFO(this->get_logger(),
         "[MPCROS] Solving at a fixed %.1f Hz (ref_timeout %.2fs, odom_timeout %.2fs)",
         _mpc_rate, _ref_timeout, _odom_timeout);
   }
   else
   {
      RCLCPP_WARN(this->get_logger(),
         "[MPCROS] mpc_rate <= 0: falling back to solving inside the reference callback. "
         "The MPC rate will follow the reference publisher.");
   }

   RCLCPP_INFO(this->get_logger(),"[MPCROS] will execute once reference trajectory is published...");

   return;
}

void
MPCROS::publishHealth(uint8_t level, const std::string & message)
{
   if(!_health_pub) return;

   diagnostic_msgs::msg::DiagnosticStatus msg;
   msg.level = level;
   msg.name = "mpc_12state";
   msg.message = message;

   auto kv = [&msg](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; msg.values.push_back(p);
   };
   kv("last_solve_ms", std::to_string(_last_solve_ms));
   kv("max_solve_ms", std::to_string(_max_solve_ms));
   kv("solve_count", std::to_string(_solve_count));
   kv("fail_count", std::to_string(_fail_count));
   kv("consecutive_failures", std::to_string(_consecutive_failures));
   kv("mpc_rate_hz", std::to_string(_mpc_rate));
   // Per-axis attribution: an infeasible Z (thrust budget) and an infeasible XY
   // (reference too aggressive) need opposite fixes, so an aggregate count is
   // not actionable. "inaccurate" counts solves accepted at reduced tolerance.
   kv("fail_z", std::to_string(_mpc->zSolveFailures()));
   kv("fail_xy", std::to_string(_mpc->xySolveFailures()));
   kv("fail_yaw", std::to_string(_mpc->yawSolveFailures()));
   kv("inaccurate_solves", std::to_string(_mpc->inaccurateSolves()));
   kv("last_fail_reason", _mpc->lastFailReason());
   // Soft state constraints: how far outside the planning envelope the plan
   // had to go. Sustained non-zero slack is the "flying outside the envelope"
   // warning that the old infeasibility count used to (accidentally) provide.
   kv("max_slack_z", std::to_string(_mpc->maxSlackZ()));
   kv("max_slack_xy", std::to_string(_mpc->maxSlackXY()));
   kv("max_slack_yaw", std::to_string(_mpc->maxSlackYaw()));
   kv("soft_active_solves", std::to_string(_mpc->softConstraintActiveCount()));

   _health_pub->publish(msg);
}

void
MPCROS::mpcTimerCallback(void)
{
   using diagnostic_msgs::msg::DiagnosticStatus;

   // Freshness gates. Solving on stale data is worse than not solving: it
   // produces a confident-looking setpoint derived from a target that may no
   // longer be there. Skipping lets the controller's hold-on-setpoint-loss
   // failsafe take over, which is the safe behaviour.
   const auto now = this->now();

   if(!_state_received)
   {
      _consecutive_failures++;
      publishHealth(DiagnosticStatus::WARN, "waiting for odometry");
      return;
   }
   if(!_have_ref)
   {
      _consecutive_failures++;
      publishHealth(DiagnosticStatus::WARN, "waiting for reference trajectory");
      return;
   }

   const double odom_age = (now - _last_odom_stamp).seconds();
   if(_odom_timeout > 0.0 && odom_age > _odom_timeout)
   {
      _consecutive_failures++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
         "[MPCROS] Odometry is stale (%.3f s > %.3f s). Skipping solve.", odom_age, _odom_timeout);
      publishHealth(DiagnosticStatus::ERROR, "odometry stale");
      return;
   }

   const double ref_age = (now - _last_ref_stamp).seconds();
   if(_ref_timeout > 0.0 && ref_age > _ref_timeout)
   {
      _consecutive_failures++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
         "[MPCROS] Reference trajectory is stale (%.3f s > %.3f s). Skipping solve.", ref_age, _ref_timeout);
      publishHealth(DiagnosticStatus::ERROR, "reference trajectory stale");
      return;
   }

   const auto t0 = std::chrono::steady_clock::now();
   const bool ok = mpcROSLoop();
   const auto t1 = std::chrono::steady_clock::now();
   _last_solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
   _max_solve_ms = std::max(_max_solve_ms, _last_solve_ms);

   if(ok)
   {
      _solve_count++;
      _consecutive_failures = 0;
      // A solve that overruns its own period cannot hold the commanded rate.
      if(_mpc_rate > 0.0 && _last_solve_ms > 1000.0/_mpc_rate)
      {
         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[MPCROS] Solve took %.2f ms, longer than the %.2f ms control period.",
            _last_solve_ms, 1000.0/_mpc_rate);
         publishHealth(DiagnosticStatus::WARN, "solve overran control period");
         return;
      }
      publishHealth(DiagnosticStatus::OK, "ok");
   }
   else
   {
      _fail_count++;
      _consecutive_failures++;
      if(_consecutive_failures >= _max_consecutive_failures)
      {
         RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[MPCROS] %d consecutive failed solves. Downstream should treat the MPC as unavailable.",
            _consecutive_failures);
         publishHealth(DiagnosticStatus::ERROR, "repeated solver failures");
      }
      else
      {
         publishHealth(DiagnosticStatus::WARN, "solver failure");
      }
   }
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

   // Node-clock arrival time, used by the solve timer's staleness gate. Kept
   // separate from the message stamp so a source with a skewed clock cannot
   // make stale data look fresh (or fresh data look stale).
   _last_odom_stamp = this->now();

   std::lock_guard<std::mutex> lock(_data_mutex);
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
    
   // Velocity: nav_msgs/Odometry twist is expressed in child_frame_id, which for
   // mavros local_position/odom is the BODY frame. The MPC state is in the world
   // (ENU local) frame, so it must be rotated. Feeding body-frame velocity in
   // directly (or zeroing it, as this node used to) makes every plan start from
   // a state that does not match reality: at pursuit speeds the resulting plan
   // has a velocity discontinuity at t=0 that the tracker absorbs as error.
   // Which frame the twist is in depends on the odometry source, so it is a
   // parameter rather than an assumption (default true = mavros behaviour).
   tf2::Vector3 v_in(msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z);
   tf2::Vector3 v_world = _odom_twist_in_body_frame ? (m * v_in) : v_in;

   // Acceleration is left at zero: the only available source is the IMU, which is
   // body-frame and gravity-biased, and feeding it in raw is worse than nothing
   // (see imuCallback). A filtered/derotated accel estimate is Phase-2 work.
   _current_state << msg.pose.pose.position.x,
                     v_world.x(),
                     0.0,
                     msg.pose.pose.position.y,
                     v_world.y(),
                     0.0,
                     msg.pose.pose.position.z,
                     v_world.z(),
                     0.0,
                     yaw,
                     msg.twist.twist.angular.z,
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

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.poses.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refPathCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.poses.size(), _mpcWindow+1);
      return;
   }

   {
      std::lock_guard<std::mutex> lock(_data_mutex);
      _referenceTraj.setZero();
      for (int i=0; i<_mpcWindow+1; i++)
      {
         _referenceTraj(i*NUM_OF_STATES+0,0) = msg.poses[i].pose.position.x;
         _referenceTraj(i*NUM_OF_STATES+3,0) = msg.poses[i].pose.position.y;
         _referenceTraj(i*NUM_OF_STATES+6,0) = msg.poses[i].pose.position.z;
      }
   }
   _last_ref_stamp = this->now();
   _have_ref = true;

   // Legacy path: when no fixed rate is configured, solve here as before.
   if(_mpc_rate <= 0.0)
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

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.poses.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refPosesCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.poses.size(), _mpcWindow+1);
      return;
   }

   {
      std::lock_guard<std::mutex> lock(_data_mutex);
      _referenceTraj.setZero();
      for (int i=0; i<_mpcWindow+1; i++)
      {
         _referenceTraj(i*NUM_OF_STATES+0,0) = msg.poses[i].position.x;
         _referenceTraj(i*NUM_OF_STATES+3,0) = msg.poses[i].position.y;
         _referenceTraj(i*NUM_OF_STATES+6,0) = msg.poses[i].position.z;
      }
   }
   _last_ref_stamp = this->now();
   _have_ref = true;

   if(_mpc_rate <= 0.0)
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

   // Make sure we have enough state predictions of the target (reference trajectory)
   if (msg.states.size() < (long unsigned int)(_mpcWindow+1) )
   {
      RCLCPP_ERROR(this->get_logger(), "[MPCROS::refTrajCallback] Not enough reference states to consume. Size of reference states %d < MPC steps+1 %d", (int)msg.states.size(), _mpcWindow+1);
      return;
   }

   std::unique_lock<std::mutex> lock(_data_mutex);
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
   lock.unlock();

   _last_ref_stamp = this->now();
   _have_ref = true;

   if(_mpc_rate <= 0.0)
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
   // Receding-horizon command: take a NEAR sample of the optimal trajectory,
   // not the horizon end. Clamped to the available solution length.
   int ui = _cmd_sample_index;
   if (ui < 0) { ui = 0; }
   if (ui > (int)_solution_traj_msg.states.size()-1) { ui = (int)_solution_traj_msg.states.size()-1; }
   _multidof_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(start_t*1e9));
   _multidof_msg.points.resize(1);
   _multidof_msg.points[0].transforms.resize(1);
   _multidof_msg.points[0].velocities.resize(1);
   _multidof_msg.points[0].accelerations.resize(1);

   _multidof_msg.points[0].transforms[0].translation.x = _solution_traj_msg.states[ui].position.x;
   _multidof_msg.points[0].transforms[0].translation.y = _solution_traj_msg.states[ui].position.y;
   _multidof_msg.points[0].transforms[0].translation.z = _solution_traj_msg.states[ui].position.z;
   _multidof_msg.points[0].velocities[0].linear.x = _solution_traj_msg.states[ui].velocity.x;
   _multidof_msg.points[0].velocities[0].linear.y = _solution_traj_msg.states[ui].velocity.y;
   _multidof_msg.points[0].velocities[0].linear.z = _solution_traj_msg.states[ui].velocity.z;
   _multidof_msg.points[0].accelerations[0].linear.x = _solution_traj_msg.states[ui].acceleration.x;
   _multidof_msg.points[0].accelerations[0].linear.y = _solution_traj_msg.states[ui].acceleration.y;
   _multidof_msg.points[0].accelerations[0].linear.z = _solution_traj_msg.states[ui].acceleration.z;

   q_yaw.setRPY(0, 0, _solution_traj_msg.states[ui].yaw);
   _multidof_msg.points[0].transforms[0].rotation.x = q_yaw.x();
   _multidof_msg.points[0].transforms[0].rotation.y = q_yaw.y();
   _multidof_msg.points[0].transforms[0].rotation.z = q_yaw.z();
   _multidof_msg.points[0].transforms[0].rotation.w = q_yaw.w();

   return;

}


bool MPCROS::mpcROSLoop(void)
{
   // Take a consistent snapshot of the inputs. Copying under the lock (rather
   // than holding it across the solve) keeps the sensor callbacks responsive.
   MatX_12STATE current_state;
   Eigen::MatrixXd referenceTraj;
   {
      std::lock_guard<std::mutex> lock(_data_mutex);
      current_state = _current_state;
      referenceTraj = _referenceTraj;
   }

   if(!_mpc->setCurrentState(current_state))
   {
      RCLCPP_ERROR(this->get_logger(),"[MPCROS::mpcROSLoop] Could not set _current_state");
      return false;
   }

   if(!_mpc->setReferenceTraj(referenceTraj))
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