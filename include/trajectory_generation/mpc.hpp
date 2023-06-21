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
#ifndef MPC_H
#define MPC_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include<fstream>
#include <Eigen/Dense>
// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>


// Function to print a string in yellow with optional variables
template<typename... Args>
void printWarn(const char* format, const Args&... args) {
    printf("\033[1;33m"); // Set text color to yellow
    printf("[INFO] ");
    printf(format, args...);
    printf("\033[0m"); // Reset text color (back to default)
    printf("\n");
}

template<typename... Args>
void printInfo(const char* format, const Args&... args) {
    printf("\033[1;32m"); // Set text color to green
    printf("[INFO] ");
    printf(format, args...);
    printf("\033[0m"); // Reset text color (back to default)
    printf("\n");
}

template<typename... Args>
void printError(const char* format, const Args&... args) {
    printf("\033[1;31m"); // Set text color to red
    printf("[ERROR] ");
    printf(format, args...);
    printf("\033[0m"); // Reset text color (back to default)
    printf("\n");
}

/** MPC
 * Implements Receding horizon controller for generating optimal trajectory to track a moving object.
 * Assumed model of the drone that is tracking the target is a 3rd order discrete LTI system capturing the translational dynamics.
 */
class MPC
{
   private:
//   ros::NodeHandle       _nh;                    /** ROS node handle */
//   ros::NodeHandle       _nh_private;            /** ROS private node handle */

//   ros::Subscriber       _dronePose_sub;         /** REMOVE Drone's pose subscriber */
//   ros::Subscriber       _droneOdom_sub;         /** Drone's odometry subscriber, to get position/velocity, "mavros/local_position/odom" */
//   ros::Subscriber       _droneImu_sub;          /** Drone's IMU subscriber, to get acceleration, "mavros/imu/data" */
//   ros::Subscriber       _referenceTraj_sub;     /** Subscriber to the target predicted trajectory. Referene trajectory of the MPC */
//   ros::Subscriber       _testCase_sub;          /** Subscriber for running testCases() function */

//   ros::Publisher        _desired_traj_pub;      /** Desired trajectory sent to the trajectory planner/sampler */
//   ros::Publisher        _poseHistory_pub;       /** ROS Publisher for _posehistory_vector */
//   ros::Publisher        _multiDofTraj_pub;          /** To publish first MPC control solution to the geometric controller */
  
//   ros::ServiceServer    _engageCtrl_srv;        /** Engages/disengages MPC controller */

  bool                  _debug;                 /** Enable printing debug messages */
  double                _dt;                    /** Prediction time step in seconds */
  std::string           _reference_frame_id;    /** Name of the map (inertial) frame, where the drone localizes */
  
  bool                  _use_6dof_model;        /** Use 6DoF model instead of 9DoF */
  Eigen::MatrixXd       _current_drone_state;   /** Current drone state (position, velocity, acceleration) */
  Eigen::Matrix3d       _current_drone_accel;   /** Latest drone acceleration measurements. Will be added to _current_drone_state */
  bool                  _drone_state_received;  /** True if a drone's first measurment is received. Used for initialization*/
  double             _drone_state_last_t;    /** Last time stamp of _current_drone_state */
  double             _drone_state_current_t; /** Current time stamp of _current_drone_state */
  bool                  _target_traj_received;  /** Flag to indicate whether the first target trajectory is received */
  
  
  double                _alt_above_target;      /** Desired altitude above target. */
  
  Eigen::MatrixXd       _referenceTraj;         /** Target's predicted trajectory, over _mpcWindow. Received from the target predictor node*/
  Eigen::VectorXd       _ref_traj_px;           /** Reference trajectory, position, x component */
  Eigen::VectorXd       _ref_traj_py;           /** Reference trajectory, position, y component */
  Eigen::VectorXd       _ref_traj_pz;           /** Reference trajectory, position, z component */
  Eigen::VectorXd       _ref_traj_vx;           /** Reference trajectory, velocity, x component */
  Eigen::VectorXd       _ref_traj_vy;           /** Reference trajectory, velocity, y component */
  Eigen::VectorXd       _ref_traj_vz;           /** Reference trajectory, velocity, z component */
  Eigen::VectorXd       _ref_traj_ax;           /** Reference trajectory, acceleration, x component */
  Eigen::VectorXd       _ref_traj_ay;           /** Reference trajectory, acceleration, y component */
  Eigen::VectorXd       _ref_traj_az;           /** Reference trajectory, acceleration, z component */
//   custom_trajectory_msgs::StateTrajectory _solution_traj_msg; /** ROS message for the optimal trajectory, position, velocity, acceleration, max velocity, max acceleration */
  double             _ref_traj_last_t;       /** Time stamp of the last reference trajectory */
  
  int                   _mpcWindow;             /** Number of prediction steps (N) */
  
  Eigen::MatrixXd       _A;                     /** Discrete transition matrix */
  Eigen::MatrixXd       _B;                     /** Discrete input matrix */
  double                _state_weight;          /** State weight. */
  double                _input_weight;          /** Input weight. */
  double                _smooth_input_weight;   /** Weight/penality on input smoothing term */
  bool                  _enable_control_smoothing; /** Affects the Hessian structure */
  Eigen::MatrixXd       _Q;                     /** States wieght matrix of the quadratic MPC objective. x^T Q x. */
  Eigen::MatrixXd       _R;                     /** Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd       _gradient;              /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd       _hessian;               /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _hessian_sparse;   /** sparce version of the hessian */
  Eigen::MatrixXd       _Ac;                    /** Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _Ac_sparse;       /** Sparse version of Ac */ 
  Eigen::VectorXd       _xMin;                  /** Lower bounds on position, velocity, acceleration */
  Eigen::VectorXd       _xMax;                  /** Upper bounds on position, velocity, acceleration*/
  Eigen::VectorXd       _uMin;                  /** Lower bounds on inputs (jerk) */
  Eigen::VectorXd       _uMax;                  /** Upper bounds on inputs (jerk) */
  Eigen::VectorXd       _lowerBounds;            /** Lower bounds vector of the QP problem */
  Eigen::VectorXd       _upperBounds;           /** Upper bounds vector of the QP problem */
  Eigen::Vector3d       _maxVel;                /** Maximum drone's velocity m/s*/
  Eigen::Vector3d       _maxAccel;              /** Maximum drone's acceleration m/s/s */
  Eigen::Vector3d       _maxJerk;               /** Maximum drone's jerk m/s/s/s */

  Eigen::Vector3d       _mpc_ctrl_sol;          /** MPC control solution */
  Eigen::VectorXd       _optimal_state_traj;    /** Entire state trajectory part of the MPC solution */
  Eigen::VectorXd       _optimal_traj_px;       /** Optimal x-position trajectory */
  Eigen::VectorXd       _optimal_traj_py;       /** Optimal y-position trajectory */
  Eigen::VectorXd       _optimal_traj_pz;       /** Optimal z-position trajectory */
  Eigen::VectorXd       _optimal_traj_vx;       /** Optimal x-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_vy;       /** Optimal y-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_vz;       /** Optimal z-velocity trajectory */
  Eigen::VectorXd       _optimal_traj_ax;       /** Optimal x-acceleration trajectory */
  Eigen::VectorXd       _optimal_traj_ay;       /** Optimal y-acceleration trajectory */
  Eigen::VectorXd       _optimal_traj_az;       /** Optimal z-acceleration trajectory */
  Eigen::VectorXd       _optimal_control_traj;  /** Entire control inputs trajectory part of the MPC solution */
  Eigen::VectorXd       _optimal_traj_ux;       /** Optimal control input trajectory in x direction */
  Eigen::VectorXd       _optimal_traj_uy;       /** Optimal control input trajectory in y direction */
  Eigen::VectorXd       _optimal_traj_uz;       /** Optimal control input trajectory in z direction */

  double                _minAltitude;           /** Minimum altitude to commanded by the MPC */

  bool                  _run_test_cases;        /** If true testCase() is executed, no ROS subscribers */
  bool                  _is_MPC_initialized;    /** True if MPC problem is initialized */
  bool                  _engage_controller;     /** **REMOVE** Flag for whether to publish MPC controls as setpoints */
  bool                  _is_mpc_engaged;        /** Flag for whether to publish MPC controls as setpoints */

  // bool                  _pub_pose_path;         /** Whether to publish MPC predicted path for visualization in RViz */
//   std::vector<geometry_msgs::PoseStamped> _posehistory_vector; /** Holds the predicted positions of the MPC, for visualization */

  OsqpEigen::Solver     _qpSolver;              /** Object of the quadratic program solver */

  bool                  _save_mpc_data;         /** Whether to save MPC data to _outputCSVFile */
  std::string           _outputCSVFile;         /** Full path to a CSV output file where MPC is stored */

  bool                  _plot;                  /** Plots solutions, reference trajectory. ONLY if _run_test_cases==True */

  /**
  * @brief Drone's odometry ROS Callback. Updates current_drone_state_ with position and velocity only
  * 
  * @param msg Holds Odometry measurement
  * 
  */
//   void droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
  * @brief Drone's IMU ROS Callback. Updates current_drone_accel_
  * 
  * @param msg Holds sensor_msgs::Imu measurement
  * 
  */
//   void droneImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
  * @brief Callback of the MPC reference trajectory, which is expected to be published by traj_predictor node.
  * Updates _referenceTraj
  * @param msg mpc_tracker::StateTrajectory
  */
//   void refTrajCallback(const custom_trajectory_msgs::StateTrajectory::ConstPtr& msg);

  /**
  * @brief A callback that allows testCses() peroidically by publishing to a topic
  */
//   void testCasesCallback(const std_msgs::Empty::ConstPtr& msg);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x.
   * Uses state_weight_ and updates Q_
   */
  void setQ(void);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x, fror 6DoF model.
   * Uses state_weight_ and updates Q_
   */
  void setQ6DoF(void);

  /**
   * @brief Sets the inputs weight matrix, R in u^T * R * u.
   * Uses input_weight_ and updates R_
   */
  void setR(void);

  /**
   * @brief Sets the transition matix for 3D discrete-time  integrator.
   * Result is a function of _dt and it's saved in _A
   */
  void setTransitionMatrix(void);

  /**
   * @brief Sets the transition matix for discrete time 6DoF model
   * Result is a function of _dt and it's saved in _A
   */
  void setTransitionMatrix6DoF(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time integrator in 3D.
   * Result is a function of dt_ and saved in _B
   */
  void setInputMatrix(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time discrete time 6DoF model.
   * Result is a function of dt_ and saved in _B
   */
  void setInputMatrix6DoF(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax
   * Uses _maxVel, _maxAccel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  void setStateBounds(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax, for 6Dof model.
   * Uses _maxVel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  void setStateBounds6DoF(void);

  /**
   * @brief Defines the control limits, _uMin, _uMax
   * Uses _maxJerk
   * Updates _uMin, _uMax
   */
  void setControlBounds(void);

  /**
   * @brief Computes the Hessain matrix of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * The Hessian matrix of quadratic function x^T*P*X is P
   * Result is saved in _hessian
   */
  void castMPCToQPHessian(void);

  /**
   * @brief Computes the gradient vector (linear term q in q^T*x) of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * Result is saved in _gradient
   */
  void castMPCToQPGradient(void);

  /**
   * @brief Updates the gradient vector, _gradient, using _referenceTraj
   */
  void updateQPGradientVector(void);

  /**
   * @brief Computes the constraint matrix required by the QP solver
   */
  void castMPCToQPConstraintMatrix(void);

  /**
   * @brief Constructs the bounds vectors for the QP problem.
   * Uses _xMin, _xMax, _uMin, _uMax
   * Result is saved in _lowerBounds and _upperBounds vetors.
   */
  void castMPCToQPConstraintBounds(void);

  /**
   * @brief Updates _lowerBounds & _upperBounds vectors, using _current_drone_state
   */
  void updateQPConstraintsBounds(void);

  /**
   * @brief Initialize QP solver
   * 
   * @return True if initialization is successful. False, otherwise.
   */
  bool initQPSolver(void);


  /**
  * @brief Updates _qpSolver bounds & gradient using _current_drone_state, and _referenceTraj
  */
  bool updateQP(void);

  /**
  * @brief Extracts MPC solutions, contorl/state trajectories, from QP
  * Updates _state_traj_sol, _control_traj_sol
  */
  void extractSolution(void);

  /**
  * @brief Extracts MPC solutions, contorl/state trajectories, from QP, for 6DoF model.
  * Updates _state_traj_sol, _control_traj_sol
  */
  void extractSolution6Dof(void);

  /**
   * @brief Published setpoint message to MAVROS.
   */
//   void publishSetpoint(void);

//   bool engageMPCCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
  * @brief Prints information about the QP problem, e.g. number of optimization variables, size of the Hessian matrix, ... etc
  */
  void printProblemInfo(void);

  /**
  * @brief Appends a new pose msg to _posehistory_vector
  */
//   void appendPoseHistory(geometry_msgs::PoseStamped pose_msg);

  /**
  * @brief Publishes _posehistory_vector to a ROS topic
  */
//   void pubPoseHistory(void);

  /**
   * @brief Published first MPC control solution u[0] to the geometric controller
  */ 
//   void pubMultiDofTraj(void);

  /**
  * @brief Runs a test case to test the entire MPC loop and the associated functions
  */
  void testCases(void);

  /**
  * @brief Saves MPC matrices to an CSV file
  *
  * @param fileName Full path to .csv file
  */
  void saveMPCDataToFile(void);

  /**
  * @brief Plots solutions of the MPC problem. Works ONLY if _run_test_cases==True. For realtime plotting use RViz
  */
  void plotSolutions(void);

public:

  static constexpr int      NUM_OF_STATES = 6;             /** Number of UAV states (position, velocity, acceleration) */
  static constexpr int      NUM_OF_INPUTS = 3;      /** Number of control inputs of the UAV model. Jerk \in R^3 */

  /**
   * @brief Constructor
   * 
   */
  MPC();
  
  /** Destructor */
  ~MPC();

  /**
   * @brief Initialize MPC problem.
   * @return Bool True if initialization is successful
   */
  bool initMPCProblem(void);

  /**
   * @brief Sets _dt
   * @param dt double prediction sampling time in seconds.
  */
  bool setDt(double dt);

  /**
   * @brief Sets _debug
   * @param d bool
  */
 void setDebug(bool d);

 bool setMPCWindow(int N);

 /**
  * @brief Sets _state_weight
  * @param w double weight >= 0
 */
 bool set_state_weight(double w);

 /**
  * @brief Sets _input_weight
  * @param w double weight >= 0
 */
 bool set_input_weight(double w);
 
 /**
  * @brief Sets _smooth_input_weight
  * @param w double weight >= 0
 */
 bool set_smooth_input_weight(double w);

 void enable_control_smoothing(bool b);

 /**
  * @brief Sets _alt_above_target
  * @param h double >=0
 */
 bool set_alt_above_target(double h);

 /**
  * @brief Sets use_6dof_model
  * @param b bool
 */
 void use_6dof_model(bool b);

 /**
  * @brief Sets _reference_frame_id
  * @param s std::string
 */
 bool set_reference_frame_id(std::string s);

 /**
  * @brief Sets _minAltitude
  * @param h double minimum altitude in the generated trajectory >=0
 */
 bool set_minimum_altitude(double h);

 /**
  * @brief Sets _current_drone_state
  * @param x Eigen::MatrixXd
 */
 bool set_current_drone_state(Eigen::MatrixXd x);

 /**
  * @brief Sets _current_drone_accel
  * @param a Eigen::Matrix3d
 */
 bool set_current_drone_accel(Eigen::Matrix3d a);

 bool set_maxVel(std::vector<double> v);

 bool set_maxAccel(std::vector<double> a);

 bool set_maxJerk(std::vector<double> j);

 /**
   * @brief This is the main loop, which executes MPC control loop.
   */
  bool mpcLoop(void);

  /**
   * @brief Sets _referenceTraj
   * @param v Eigen::MatrixXd
  */
  bool set_referenceTraj(Eigen::MatrixXd v);

  Eigen::VectorXd get_optimal_state_traj(void);

  Eigen::VectorXd get_optimal_control_traj(void);

  int get_num_of_states(void);
  int get_num_of_inputs(void);

  Eigen::Vector3d get_maxAccel(void);
  Eigen::Vector3d get_maxVel(void);
  Eigen::Vector3d get_maxJerk(void);
};

#endif 