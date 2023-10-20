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

static constexpr int      NUM_OF_STATES = 6;       /** Number of UAV states (position, velocity) */
static constexpr int      NUM_OF_INPUTS = 3;      /** Number of control inputs of the UAV model. acceleration \in R^3 */

static constexpr int      NUM_OF_XY_STATES = 6;       /** Number of UAV states in XY plane (position, velocity, acceleration) */
static constexpr int      NUM_OF_XY_INPUTS = 2;      /** Number of control inputs of the UAV model in XY plane. jerk \in R^2 */

static constexpr int      NUM_OF_Z_STATES = 3;       /** Number of UAV states along Z-axis (position, velocity, acceleration) */
static constexpr int      NUM_OF_Z_INPUTS = 1;      /** Number of control inputs of the UAV model along Z-axis. jerk \in R^1 */

static constexpr int      NUM_OF_YAW_STATES = 3;       /** Number of UAV states for the yaw motion (position, velocity, acceleration) */
static constexpr int      NUM_OF_YAW_INPUTS = 1;      /** Number of control inputs of the UAV model for yaw motion. jerk \in R^1 */


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
class MPC
{
private:

  bool                  _debug;                 /** Enable printing debug messages */
  double                _dt;                    /** Prediction time step in seconds */
  
  Eigen::Matrix<double, NUM_OF_XY_STATES+NUM_OF_Z_STATES+NUM_OF_YAW_STATES,1>       _current_state;   /** Current drone state (position, velocity, acceleration, yaw, yaw speed, yaw acceleration) */
  bool                  _state_received;  /** True if a drone's first measurment is received. Used for initialization*/
  double             _last_state_time;    /** Last time stamp of _current_drone_state */
  double             _current_state_time; /** Current time stamp of _current_drone_state */
  bool                  _target_traj_received;  /** Flag to indicate whether the first target trajectory is received */
  double                _alt_above_target;      /** Desired altitude above target. */
  Eigen::MatrixXd       _referenceTraj;         /** Target's predicted trajectory, over _mpcWindow. Received from the target predictor node*/
  double             _ref_traj_last_t;       /** Time stamp of the last reference trajectory */
  
  int                   _mpcWindow;             /** Number of prediction steps (N) */

  //////////////////////// XY variables /////////////////////////////////////
  
  MatXbyX_XY                  _xy_A;                     /** Discrete transition matrix */
  MatXbyU_XY                  _xy_B;                     /** Discrete input matrix */
  MatXbyX_XY                  _xy_Q;                     /** States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_XY                  _xy_R;                     /** Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _xy_gradient;              /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _xy_hessian;               /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _xy_hessian_sparse;   /** sparce version of the hessian */
  Eigen::MatrixXd             _xy_Ac;                    /** Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _xy_Ac_sparse;       /** Sparse version of Ac */ 
  
  MatX_XY                     _xyMin;                  /** Lower bounds on position, velocity, acceleration in XY plane */
  MatX_XY                     _xyMax;                  /** Upper bounds on position, velocity, acceleration in XY plane */
  MatU_XY                     _uxyMin;                  /** Lower bounds on jerk in XY plane */
  MatU_XY                     _uxyMax;                  /** Upper bounds on jerk in XY plane */
  Eigen::VectorXd             _xylowerBounds;            /** Lower bounds vector of the XY QP problem */
  Eigen::VectorXd             _xyupperBounds;           /** Upper bounds vector of the XY QP problem */
  double                      _xyMaxVel;            /** Maximum horizontal velocity */
  double                      _xyMaxAccel;            /** Maximum horizontal acceleration */

  Eigen::Vector2d             _xy_u0_opt;          /** XY - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _xy_x_opt;    /** XY - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _xy_u_opt;  /** XY - Entire control inputs trajectory part of the MPC solution */

  double                      _xy_state_weight;          /** XY - State weight, used in _Q_XY. */
  double                      _xy_input_weight;          /** XY - Input weight, used in _Q_XY */
  double                      _xy_smooth_input_weight;   /** Weight/penality on input smoothing term */

  //////////////////////// Z variables /////////////////////////////////////
  MatXbyX_Z                   _z_A;                     /** Z - Discrete transition matrix */
  MatXbyU_Z                   _z_B;                     /** Z - Discrete input matrix */
  MatXbyX_Z                   _z_Q;                     /** Z - States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_Z                   _z_R;                     /** Z - Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _z_gradient;              /** z - Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _z_hessian;               /** Z - Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _z_hessian_sparse;   /** Z - sparce version of the hessian */
  Eigen::MatrixXd             _z_Ac;                    /** Z - Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _z_Ac_sparse;       /** Z - Sparse version of Ac */ 
  
  MatX_Z                      _zMin;                  /** Z - Lower bounds on position, velocity, acceleration  */
  MatX_Z                      _zMax;                  /** Z - Upper bounds on position, velocity, acceleration  */
  double                      _uzMin;                  /** Z - Lower bounds on jerk */
  double                      _uzMax;                  /** Z - Upper bounds on jerk */
  Eigen::VectorXd             _zlowerBounds;            /** Z -  Lower bounds vector of the QP problem */
  Eigen::VectorXd             _zupperBounds;           /** Z - Upper bounds vector of the QP problem */
  double                      _zMaxVel;            /** Z - Maximum velocity */
  double                      _zMaxAccel;            /** Z - Maximum  acceleration */
  double                      _minAltitude;           /** Minimum altitude to commanded by the MPC */

  double                      _z_u0_opt;          /** Z - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _z_x_opt;    /** Z - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _z_u_opt;  /** Z - Entire control inputs trajectory part of the MPC solution */

  double                      _z_state_weight;          /** Z - State weight, used in _Q_XY. */
  double                      _z_input_weight;          /** Z - Input weight, used in _Q_XY */

  //////////////////////// YAW variables /////////////////////////////////////
  MatXbyX_YAW                 _yaw_A;                     /** YAW - Discrete transition matrix */
  MatXbyU_YAW                 _yaw_B;                     /** YAW - Discrete input matrix */
  MatXbyX_YAW                 _yaw_Q;                     /** YAW - States wieght matrix of the quadratic MPC objective. x^T Q x. */
  MatUbyU_YAW                 _yaw_R;                     /** YAW - Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd             _yaw_gradient;              /** Yaw - Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd             _yaw_hessian;               /** Yaw - Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> _yaw_hessian_sparse;   /** Yaw - sparce version of the hessian */
  Eigen::MatrixXd             _yaw_Ac;                    /** Yaw - Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> _yaw_Ac_sparse;       /** Yaw - Sparse version of Ac */ 
  
  MatX_YAW                    _yawMin;                  /** Yaw - Lower bounds on position, velocity, acceleration  */
  MatX_YAW                    _yawMax;                  /** Yaw - Upper bounds on position, velocity, acceleration  */
  double                      _uyawMin;                  /** Yaw - Lower bounds on jerk */
  double                      _uyawMax;                  /** Yaw - Upper bounds on jerk */
  Eigen::VectorXd             _yawlowerBounds;            /** Yaw -  Lower bounds vector of the QP problem */
  Eigen::VectorXd             _yawupperBounds;           /** Yaw - Upper bounds vector of the QP problem */
  double                      _yawMaxVel;            /** Yaw - Maximum velocity */
  double                      _yawMaxAccel;            /** Yaw - Maximum  acceleration */

  double                      _yaw_u0_opt;          /** Z - MPC first optimal control solution u_opt[0] (jerk)*/
  Eigen::VectorXd             _yaw_x_opt;    /** Z - Entire state trajectory part of the MPC solution */
  Eigen::VectorXd             _yaw_u_opt;  /** Z - Entire control inputs trajectory part of the MPC solution */

  double                      _yaw_state_weight;          /** Z - State weight, used in _Q_XY. */
  double                      _yaw_input_weight;          /** Z - Input weight, used in _Q_XY */


  // Shared variables
  bool                  _enable_control_smoothing; /** Affects the Hessian structure */
  bool                  _is_MPC_initialized;    /** True if MPC problem is initialized */

  // Solver objects
  OsqpEigen::Solver     _xy_qpSolver;              /** XY - Object of the quadratic program solver */
  OsqpEigen::Solver     _z_qpSolver;              /** Z - Object of the quadratic program solver */
  OsqpEigen::Solver     _yaw_qpSolver;              /** YAW - Object of the quadratic program solver */

  bool                  _save_mpc_data;         /** Whether to save MPC data to _outputCSVFile */
  std::string           _outputCSVFile;         /** Full path to a CSV output file where MPC is stored */

  ///////////////////////////// @ todo continue modifications from here /////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x.
   * Uses state_weight_ and updates Q_
   */
  void setQ(void);

  /**
   * To give the user full control on the entire Q matrix values
  */
  void setQ(const MatXbyX &Q);


  /**
   * @brief Sets the inputs weight matrix, R in u^T * R * u.
   * Uses input_weight_ and updates R_
   */
  void setR(void);

  /**
   * To give the user full control on the entire R matrix values
  */
  void setR(const MatUbyU &R);


  /**
   * @brief Sets the transition matix for 3D discrete-time  integrator.
   * Result is a function of _dt and it's saved in _A
   */
  void setTransitionMatrix(void);

  /**
   * @brief Sets the discrete input matrix of a discrete-time integrator in 3D.
   * Result is a function of dt_ and saved in _B
   */
  void setInputMatrix(void);

  /**
   * @brief Defines the states limits, _xMin, _xMax
   * Uses _maxVel, _maxAccel
   * Assumes infinite bounds on the position states
   * Updates _xMin, _xMax
   */
  void setStateBounds(void);

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
  * @brief Prints information about the QP problem, e.g. number of optimization variables, size of the Hessian matrix, ... etc
  */
  void printProblemInfo(void);

public:

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
 bool setStateWeight(double w);

 /**
  * @brief Sets _input_weight
  * @param w double weight >= 0
 */
 bool setInputWeight(double w);
 
 /**
  * @brief Sets _smooth_input_weight
  * @param w double weight >= 0
 */
 bool setSmoothInputWeight(double w);

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
  * @brief Sets _current_drone_state
  * @param x Eigen::MatrixXd
 */
 bool setCurrentState(const MatX &x);

  /**
    * @brief Sets _current_drone_accel
    * @param a Eigen::Matrix3d
  */
  bool setCurrentAccel(const Eigen::Vector3d &a);

  bool setMaxVel(const std::vector<double> &v);

  bool setMaxAccel(const std::vector<double> &a);

  bool updateMPC(void);

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
  void saveMPCDataToFile(void);

  /**
   * @brief Sets _referenceTraj
   * @param v Eigen::MatrixXd
  */
  bool setReferenceTraj(const Eigen::MatrixXd &v);

  Eigen::VectorXd getOptimalStateTraj(void);

  Eigen::VectorXd getOptimalControlTraj(void);

  int getNumOfStates(void);
  int getNumOfInputs(void);

  Eigen::Vector3d getMaxAccel(void);
  Eigen::Vector3d getMaxVel(void);

  Eigen::MatrixXd getTransitionMatrix(void);
  Eigen::MatrixXd getInputMatrix(void);
  Eigen::VectorXd getGradient(void);
  Eigen::VectorXd getLowerBounds(void);
  Eigen::VectorXd getUpperBounds(void);
  Eigen::MatrixXd getContraintsMatrix(void);
  Eigen::MatrixXd getHessianMatrix(void);
  Eigen::MatrixXd getQ(void);
  Eigen::MatrixXd getR(void);
};

#endif // MPC_12STATE_H