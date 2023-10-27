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

// This implementation is based on the following thesis
// https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiw7enn4oKCAxUzSvEDHbOGAuYQFnoECBQQAQ&url=https%3A%2F%2Fdspace.cvut.cz%2Fbitstream%2Fhandle%2F10467%2F76157%2FF3-DP-2018-Hert-Daniel-thesis_hertdani.pdf%3Fsequence%3D-1%26isAllowed%3Dy&usg=AOvVaw2mzG5-IBezQMNh9vt76JJx&opi=89978449
// Three stage MPC
// 1- XY trajectory generation
// 2- Z trajectory generation
// 3- yaw trajectry generation
// The above steps are done sequentially (3 MPC problems)

#ifndef MPC_12STATE_H
#define MPC_12STATE_H

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

static constexpr int      NUM_OF_STATES = 6+3+3;       /** Number of UAV states (position, velocity, acceleration, alsoo for yaw) */
static constexpr int      NUM_OF_INPUTS = 2+1+1;      /** Number of control inputs of the UAV model. acceleration \in R^3 */
// Indices of each state in the total state vector
static constexpr int      STATE_X_IDX=0;
static constexpr int      STATE_VX_IDX=1;
static constexpr int      STATE_AX_IDX=2;
static constexpr int      STATE_Y_IDX=3;
static constexpr int      STATE_VY_IDX=4;
static constexpr int      STATE_AY_IDX=5;
static constexpr int      STATE_Z_IDX=6;
static constexpr int      STATE_VZ_IDX=7;
static constexpr int      STATE_AZ_IDX=8;
static constexpr int      STATE_Yaw_IDX=9;
static constexpr int      STATE_VYaw_IDX=10;
static constexpr int      STATE_AYaw_IDX=11;

static constexpr int      NUM_OF_XY_STATES = 6;       /** Number of UAV states in XY plane (position, velocity, acceleration) */
static constexpr int      NUM_OF_XY_INPUTS = 2;      /** Number of control inputs of the UAV model in XY plane. jerk \in R^2 */
static constexpr int      NUM_OF_XY_MIXED_VEL_CONST = 4; /** Number of constraints of the 2nd order approximation of xy velocity */
static constexpr int      NUM_OF_XY_MIXED_ACCEL_CONST = 2; /** Number of constraints of the 1st order approximation of xy acceleration */
// Indices of each state in the XY state vector
static constexpr int      XY_X_IDX=0;
static constexpr int      XY_VX_IDX=1;
static constexpr int      XY_AX_IDX=2;
static constexpr int      XY_Y_IDX=3;
static constexpr int      XY_VY_IDX=4;
static constexpr int      XY_AY_IDX=5;

static constexpr int      NUM_OF_Z_STATES = 3;       /** Number of UAV states along Z-axis (position, velocity, acceleration) */
static constexpr int      NUM_OF_Z_INPUTS = 1;      /** Number of control inputs of the UAV model along Z-axis. jerk \in R^1 */
// Indices of each state in the Z state vector
static constexpr int      Z_Z_IDX=0;
static constexpr int      Z_VZ_IDX=1;
static constexpr int      Z_AZ_IDX=2;

static constexpr int      NUM_OF_YAW_STATES = 3;       /** Number of UAV states for the yaw motion (position, velocity, acceleration) */
static constexpr int      NUM_OF_YAW_INPUTS = 1;      /** Number of control inputs of the UAV model for yaw motion. jerk \in R^1 */
// Indices of each state in the Yaw state vector
static constexpr int      YAW_Yaw_IDX=0;
static constexpr int      YAW_VYaw_IDX=1;
static constexpr int      YAW_AYaw_IDX=2;

// Full State
using MatX_12STATE = Eigen::Matrix<double, NUM_OF_XY_STATES+NUM_OF_Z_STATES+NUM_OF_YAW_STATES, 1>;
// Full control input
using MatU_4INPUTS = Eigen::Matrix<double, NUM_OF_XY_INPUTS+NUM_OF_Z_INPUTS+NUM_OF_YAW_INPUTS, 1>;

// for XY dynamics
using MatXbyX_XY = Eigen::Matrix<double, NUM_OF_XY_STATES, NUM_OF_XY_STATES>;
using MatXbyU_XY = Eigen::Matrix<double, NUM_OF_XY_STATES, NUM_OF_XY_INPUTS>;
using MatUbyU_XY = Eigen::Matrix<double, NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS>;
using MatX_XY = Eigen::Matrix<double, NUM_OF_XY_STATES, 1>;
using MatU_XY = Eigen::Matrix<double, NUM_OF_XY_INPUTS, 1>;

// for Z dynamics
using MatXbyX_Z = Eigen::Matrix<double, NUM_OF_Z_STATES, NUM_OF_Z_STATES>;
using MatXbyU_Z = Eigen::Matrix<double, NUM_OF_Z_STATES, NUM_OF_Z_INPUTS>;
using MatUbyU_Z = Eigen::Matrix<double, NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS>;
using MatX_Z = Eigen::Matrix<double, NUM_OF_Z_STATES, 1>;
using MatU_Z = Eigen::Matrix<double, NUM_OF_Z_INPUTS, 1>;

// for yaw dynamics
using MatXbyX_YAW = Eigen::Matrix<double, NUM_OF_YAW_STATES, NUM_OF_YAW_STATES>;
using MatXbyU_YAW = Eigen::Matrix<double, NUM_OF_YAW_STATES, NUM_OF_YAW_INPUTS>;
using MatUbyU_YAW = Eigen::Matrix<double, NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS>;
using MatX_YAW = Eigen::Matrix<double, NUM_OF_YAW_STATES, 1>;
using MatU_YAW = Eigen::Matrix<double, NUM_OF_YAW_INPUTS, 1>;

/** MPC
 * Implements Receding horizon controller for generating optimal trajectory to track a moving object.
 * Assumed model of the drone that is tracking the target is a 3rd order discrete LTI system capturing the translational dynamics.
 */
class MPC12STATE
{
private:

  ///////////////////////////////// Common variables ///////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////

  bool                      _debug;                 /** Enable printing debug messages */
  double                    _dt;                    /** Prediction time step in seconds */
  
  MatX_12STATE              _current_state;   /** Current drone state (position, velocity, acceleration, yaw, yaw speed, yaw acceleration) */

  bool                      _state_received;  /** True if a drone's first measurment is received. Used for initialization*/
  double                    _last_state_time;    /** Last time stamp of _current_drone_state */
  double                    _current_state_time; /** Current time stamp of _current_drone_state */
  bool                      _target_traj_received;  /** Flag to indicate whether the first target trajectory is received */
  double                    _alt_above_target;      /** Desired altitude above target. */
  Eigen::MatrixXd           _referenceTraj;         /** Target's predicted trajectory, over _mpcWindow. Received from the target predictor node*/
  double                    _ref_traj_last_t;       /** Time stamp of the last reference trajectory */
  bool                      _received_refTraj;
  
  int                       _mpcWindow;             /** Number of prediction steps (N) */

  bool                      _enable_control_smoothing; /** Affects the Hessian structure */
  bool                      _is_MPC_initialized;    /** True if MPC problem is initialized */
  bool                      _save_mpc_data;         /** Whether to save MPC data to _outputCSVFile */
  std::string               _outputCSVFile;         /** Full path to a CSV output file where MPC is stored */

  Eigen::VectorXd           _x_opt;               /** Entire optimal state (x, x_dot, x_ddot, yaw, yaw_dot, yaw_ddot)*/
  Eigen::VectorXd           _u_opt;               /** Entire optimal control inputs (jx, jy, jz, jyaw)*/

  //////////////////////// XY variables /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  
  MatXbyX_XY                  _xy_A;                     /** Discrete transition matrix */
  MatXbyU_XY                  _xy_B;                     /** Discrete input matrix */
  MatXbyX_XY                  _xy_Q;                     /** States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_XY                  _xy_R;                     /** Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _xy_gradient;              /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _xy_hessian;               /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _xy_hessian_sparse;   /** sparce version of the hessian */
  Eigen::MatrixXd             _xy_Ac;                    /** Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _xy_Ac_sparse;       /** Sparse version of Ac */ 
  
  Eigen::VectorXd             _xy_Min;                  /** Lower bounds on position/velocity/accel trajectory in XY plane. vel bounds are computed using _z_x_opt */
  Eigen::VectorXd             _xy_Max;                  /** Upper bounds on  position/velocity/accel trajectory in XY plane. vel bounds are computed using _z_x_opt*/
  Eigen::VectorXd             _xy_MixedState_Min;
  Eigen::VectorXd             _xy_MixedState_Max;
  MatU_XY                     _xy_uMin;                  /** Lower bounds on jerk in XY plane */
  MatU_XY                     _xy_uMax;                  /** Upper bounds on jerk in XY plane */
  Eigen::VectorXd             _xy_lowerBounds;            /** Lower bounds vector of the XY QP problem */
  Eigen::VectorXd             _xy_upperBounds;           /** Upper bounds vector of the XY QP problem */
  double                      _xy_MaxVel;            /** Maximum horizontal velocity */
  double                      _xy_MaxAccel;            /** Maximum horizontal acceleration */
  double                      _xy_MaxJerk;            /** Maximum horizontal jerk */


  Eigen::Vector2d             _xy_u0_opt;          /** XY - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _xy_x_opt;    /** XY - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _xy_u_opt;  /** XY - Entire control inputs trajectory part of the MPC solution */

  double                      _xy_state_weight;          /** XY - State weight, used in _Q_XY. */
  double                      _xy_input_weight;          /** XY - Input weight, used in _Q_XY */
  double                      _xy_smooth_input_weight;   /** XY - Weight/penality on input smoothing term */

  Eigen::MatrixXd             _xy_referenceTraj;
  MatX_XY                     _xy_current_state;

  //////////////////////// Z variables /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  MatXbyX_Z                   _z_A;                     /** Z - Discrete transition matrix */
  MatXbyU_Z                   _z_B;                     /** Z - Discrete input matrix */
  MatXbyX_Z                   _z_Q;                     /** Z - States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_Z                   _z_R;                     /** Z - Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _z_gradient;              /** z - Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _z_hessian;               /** Z - Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _z_hessian_sparse;   /** Z - sparce version of the hessian */
  Eigen::MatrixXd             _z_Ac;                    /** Z - Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _z_Ac_sparse;       /** Z - Sparse version of Ac */ 
  
  MatX_Z                      _z_Min;                  /** Z - Lower bounds on position, velocity, acceleration  */
  MatX_Z                      _z_Max;                  /** Z - Upper bounds on position, velocity, acceleration  */
  double                      _z_uMin;                  /** Z - Lower bounds on jerk */
  double                      _z_uMax;                  /** Z - Upper bounds on jerk */
  Eigen::VectorXd             _z_lowerBounds;            /** Z -  Lower bounds vector of the QP problem */
  Eigen::VectorXd             _z_upperBounds;           /** Z - Upper bounds vector of the QP problem */
  double                      _z_MaxVel;            /** Z - Maximum velocity */
  double                      _z_MaxAccel;            /** Z - Maximum  acceleration */
  double                      _z_MaxJerk;            /** Z - Maximum jerk */
  double                      _minAltitude;           /** Minimum altitude to commanded by the MPC */

  double                      _z_u0_opt;          /** Z - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _z_x_opt;    /** Z - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _z_u_opt;  /** Z - Entire control inputs trajectory part of the MPC solution */

  double                      _z_state_weight;          /** Z - State weight, used in _Q_XY. */
  double                      _z_input_weight;          /** Z - Input weight, used in _Q_XY */
  double                      _z_smooth_input_weight;   /** Z - Weight/penality on input smoothing term */

  Eigen::MatrixXd             _z_referenceTraj;
  MatX_Z                      _z_current_state;

  //////////////////////// YAW variables /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  MatXbyX_YAW                 _yaw_A;                     /** YAW - Discrete transition matrix */
  MatXbyU_YAW                 _yaw_B;                     /** YAW - Discrete input matrix */
  MatXbyX_YAW                 _yaw_Q;                     /** YAW - States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_YAW                 _yaw_R;                     /** YAW - Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _yaw_gradient;              /** Yaw - Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _yaw_hessian;               /** Yaw - Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _yaw_hessian_sparse;   /** Yaw - sparce version of the hessian */
  Eigen::MatrixXd             _yaw_Ac;                    /** Yaw - Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _yaw_Ac_sparse;       /** Yaw - Sparse version of Ac */ 
  
  MatX_YAW                    _yaw_Min;                  /** Yaw - Lower bounds on position, velocity, acceleration  */
  MatX_YAW                    _yaw_Max;                  /** Yaw - Upper bounds on position, velocity, acceleration  */
  double                      _yaw_uMin;                  /** Yaw - Lower bounds on jerk */
  double                      _yaw_uMax;                  /** Yaw - Upper bounds on jerk */
  Eigen::VectorXd             _yaw_lowerBounds;            /** Yaw -  Lower bounds vector of the QP problem */
  Eigen::VectorXd             _yaw_upperBounds;           /** Yaw - Upper bounds vector of the QP problem */
  double                      _yaw_MaxVel;            /** Yaw - Maximum velocity */
  double                      _yaw_MaxAccel;            /** Yaw - Maximum  acceleration */
  double                      _yaw_MaxJerk;            /** Yaw - Maximum jerk */

  double                      _yaw_u0_opt;          /** Yaw - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _yaw_x_opt;    /** Yaw - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _yaw_u_opt;  /** Yaw - Entire control inputs trajectory part of the MPC solution */

  double                      _yaw_state_weight;          /** Yaw - State weight, used in _Q_XY. */
  double                      _yaw_input_weight;          /** Yaw - Input weight, used in _Q_XY */
  double                      _yaw_smooth_input_weight;   /** Yaw - Weight/penality on input smoothing term */

  Eigen::MatrixXd             _yaw_referenceTraj;
  MatX_YAW                    _yaw_current_state;

  //////////////////////////////////// Solver objects ////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  OsqpEigen::Solver     _xy_qpSolver;              /** XY - Object of the quadratic program solver */
  OsqpEigen::Solver     _z_qpSolver;              /** Z - Object of the quadratic program solver */
  OsqpEigen::Solver     _yaw_qpSolver;              /** YAW - Object of the quadratic program solver */

  /**
   * @brief Initialize all variables with dynamic size
  */
  void initVariables(void);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x.
   * Uses state_weight_ and updates Q_
   */
  // void setQ(void);

  void setZQ(void);
  void setXYQ(void);
  void setYawQ(void);

  /**
   * To give the user full control on the entire Q matrix values
  */
  // void setQ(const MatXbyX &Q);

  void setXYQ(const MatXbyX_XY &Q);
  void setZQ(const MatXbyX_Z &Q);
  void setYawQ(const MatXbyX_YAW &Q);


  /**
   * @brief Sets the inputs weight matrix, R in u^T * R * u.
   * Uses input_weight_ and updates R_
   */
  void setR(void);

  void setXYR(void);
  void setZR(void);
  void setYawR(void);

  /**
   * To give the user full control on the entire R matrix values
  */
  // void setR(const MatUbyU &R);

  void setXYR(const MatUbyU_XY &R);
  void setZR(const MatUbyU_Z &R);
  void setYawR(const MatUbyU_YAW &R);


  /**
   * @brief Sets the transition matix for 3D discrete-time  integrator.
   * Result is a function of _dt and it's saved in _A
   */
  // void setTransitionMatrix(void);

  void setXYTransitionMatrix(void);
  void setZTransitionMatrix(void);
  void setYawTransitionMatrix(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time integrator in 3D.
   * Result is a function of dt_ and saved in _B
   */
  // void setInputMatrix(void);

  void setXYInputMatrix(void);
  void setZInputMatrix(void);
  void setYawInputMatrix(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax
   * Uses _maxVel, _maxAccel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  // void setStateBounds(void);

  void setXYStateBounds(void);
  void setZStateBounds(void);
  void setYawStateBounds(void);

  /**
   * @brief Defines the control limits, _uMin, _uMax
   * Uses _maxJerk
   * Updates _uMin, _uMax
   */
  // void setControlBounds(void);

  void setXYControlBounds(void);
  void setZControlBounds(void);
  void setYawControlBounds(void);

  /**
   * @brief Computes the Hessain matrix of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * The Hessian matrix of quadratic function x^T*P*X is P
   * Result is saved in _hessian
   */
  // void castMPCToQPHessian(void);

  void castXYMPCToQPHessian(void);
  void castZMPCToQPHessian(void);
  void castYawMPCToQPHessian(void);

  /**
   * @brief Computes the gradient vector (linear term q in q^T*x) of the quadratic objective of MPC over a prediciton window, _mpcWindow.
   * Result is saved in _gradient
   */
  // void castMPCToQPGradient(void);

  void castXYMPCToQPGradient(void);
  void castZMPCToQPGradient(void);
  void castYawMPCToQPGradient(void);

  /**
   * @brief Updates the gradient vector, _gradient, using _referenceTraj
   */
  // void updateQPGradientVector(void);

  void updateXYQPGradientVector(void);
  void updateZQPGradientVector(void);
  void updateYawQPGradientVector(void);

  /**
   * @brief Computes the constraint matrix required by the QP solver
   */
  // void castMPCToQPConstraintMatrix(void);
  void castXYMPCToQPConstraintMatrix(void);
  void castZMPCToQPConstraintMatrix(void);
  void castYawMPCToQPConstraintMatrix(void);

  /**
   * @brief Constructs the bounds vectors for the QP problem.
   * Uses _xMin, _xMax, _uMin, _uMax
   * Result is saved in _lowerBounds and _upperBounds vetors.
   */
  // void castMPCToQPConstraintBounds(void);

  /**
   * @brief computes v_hmax(t) from a_z(t) and a_zmax. Requires solving the Z-axis optimization problem first.  
   * Needed by castXYMPCToQPConstraintBounds()
   * Uses _z_x_opt.
   * Updates _xy_Min, _xy_Max.
   */ 
  bool computeXYBounds(void);

  void castXYMPCToQPConstraintBounds(void);
  void castZMPCToQPConstraintBounds(void);
  void castYawMPCToQPConstraintBounds(void);

  /**
   * @brief Updates _lowerBounds & _upperBounds vectors, using _current_drone_state
   */
  // void updateQPConstraintsBounds(void)

  void updateXYQPConstraintsBounds(void);
  void updateZQPConstraintsBounds(void);
  void updateYawQPConstraintsBounds(void);

  /**
   * @brief Initialize all three QP solver (XY, Z, YAW)
   * 
   * @return True if initialization is successful. False, otherwise.
   */
  bool initQPSolver(void);


  /**
  * @brief Updates _qpSolver bounds & gradient using _current_drone_state, and _referenceTraj
  */
  // bool updateQP(void);

  bool updateXYQP(void);
  bool updateZQP(void);
  bool updateYawQP(void);

  bool computeYawRefTrajectory(void);

  /**
  * @brief Extracts MPC solutions, contorl/state trajectories, from QP
  * Updates _state_traj_sol, _control_traj_sol
  */
  void extractSolution(void);
  void extractXYSolution(void);
  void extractZSolution(void);
  void extractYawSolution(void);

  /**
  * @brief Prints information about the QP problem, e.g. number of optimization variables, size of the Hessian matrix, ... etc
  */
  // void printProblemInfo(void);

public:

  /**
   * @brief Constructor
   * 
   */
  MPC12STATE();
  
  /** Destructor */
  ~MPC12STATE();

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
//  bool setStateWeight(double w);

bool setXYStateWeight(double w);
bool setZStateWeight(double w);
bool setYawStateWeight(double w);

 /**
  * @brief Sets _input_weight
  * @param w double weight >= 0
 */
//  bool setInputWeight(double w);

bool setXYInputWeight(double w);
bool setZInputWeight(double w);
bool setYawInputWeight(double w);
 
 /**
  * @brief Sets _smooth_input_weight
  * @param w double weight >= 0
 */
//  bool setSmoothInputWeight(double w);

bool setXYSmoothInputWeight(double w);
bool setZSmoothInputWeight(double w);
bool setYawSmoothInputWeight(double w);

 void enableControlSmoothing(bool b);

 /**
  * @brief Sets _alt_above_target
  * @param h double >=0
 */
 bool setAltAboveTarget(double h);

 /**
  * @brief Sets _minAltitude
  * @param h double minimum altitude in the generated trajectory >=0
 */
 bool setMinimumAltitude(double h);

 /**
  * @brief Sets current state, position, velocity, acceleration; also for yaw
  * @param x MatX_12STATE
 */
 bool setCurrentState(const MatX_12STATE &x);

  /**
    * @brief Sets ...
    * @param a Eigen::Vector4d
  */
  bool setCurrentAccel(const Eigen::Vector4d &a);

  // bool setMaxVel(const std::vector<double> &v);

  bool setXYMaxVel(const double vhmax);
  bool setZMaxVel(const double vzmax);
  bool setYawMaxVel(const double vyawmax);

  // bool setMaxAccel(const std::vector<double> &a);
  bool setXYMaxAccel(const double ahmax);
  bool setZMaxAccel(const double azmax);
  bool setYawMaxAccel(const double ayawmax);

  bool setXYMaxJerk(const double jxymax);
  bool setZMaxJerk(const double jzmax);
  bool setYawMaxJerk(const double jyawmax);

  // bool updateMPC(void);
  bool updateXYMPC(void);
  bool updateZMPC(void);
  bool updateYawMPC(void);

 /**
   * @brief This is the main loop, which executes MPC control loop.
   */
  bool mpcLoop(void);

  /**
   * @brief Sets the path to the output file, _outputCSVFile
   * @param path std::string absolutep ath to of the output file
  */
  void setOutputFilePath(std::string path);

  /**
  * @brief Saves MPC matrices to an CSV file
  */
  // void saveMPCDataToFile(void);
  void saveMPCSolutionsToFile(void);

  /**
   * @brief Sets _referenceTraj
   * @param v Eigen::MatrixXd
  */
  bool setReferenceTraj(const Eigen::MatrixXd &v);

  Eigen::VectorXd getOptimalStateTraj(void);

  Eigen::VectorXd getOptimalControlTraj(void);

  int getNumOfStates(void);
  int getNumOfInputs(void);

  // double getXYMaxAccel(void);
  // double getZMaxAccel(void);
  // double getYawMaxAccel(void);

  // double getXYMaxVel(void);
  // double getZMaxVel(void);
  // double getYawMaxVel(void);

  Eigen::MatrixXd getTransitionMatrix(void);
  // Eigen::MatrixXd& getXYTransitionMatrix(void);
  // Eigen::MatrixXd& getZTransitionMatrix(void);
  // Eigen::MatrixXd& getYawTransitionMatrix(void);

  Eigen::MatrixXd getInputMatrix(void);
  // Eigen::MatrixXd& getXYInputMatrix(void);
  // Eigen::MatrixXd& getZInputMatrix(void);
  // Eigen::MatrixXd& getYawInputMatrix(void);

  Eigen::VectorXd getGradient(void);
  // Eigen::VectorXd& getXYGradient(void);
  // Eigen::VectorXd& getZGradient(void);
  // Eigen::VectorXd& getYawGradient(void);

  // Eigen::VectorXd& getXYLowerBounds(void);
  // Eigen::VectorXd& getZLowerBounds(void);
  // Eigen::VectorXd& getYawLowerBounds(void);

  // Eigen::VectorXd& getXYUpperBounds(void);
  // Eigen::VectorXd& getZUpperBounds(void);
  // Eigen::VectorXd& geYawUpperBounds(void);

  // Eigen::MatrixXd& getXYContraintsMatrix(void);
  // Eigen::MatrixXd& getZContraintsMatrix(void);
  // Eigen::MatrixXd& getYawContraintsMatrix(void);

  // Eigen::MatrixXd& getXYHessianMatrix(void);
  // Eigen::MatrixXd& getZHessianMatrix(void);
  // Eigen::MatrixXd& getYawHessianMatrix(void);

  Eigen::MatrixXd getQ(void);
  // Eigen::MatrixXd& getXYQ(void);
  // Eigen::MatrixXd& getZQ(void);
  // Eigen::MatrixXd& getYawQ(void);

  Eigen::MatrixXd getR(void);
  // Eigen::MatrixXd& getXYR(void);
  // Eigen::MatrixXd& getZR(void);
  // Eigen::MatrixXd& getYawR(void);
};

#endif // MPC_12STATE_H