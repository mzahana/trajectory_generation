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

#include "trajectory_generation/mpc.hpp"

MPC::MPC():
_debug(true),
_dt(0.05),
_reference_frame_id("map"),
_use_6dof_model(true),
_drone_state_received(false),
_drone_state_last_t(0.0),
_drone_state_current_t(0.0),
_target_traj_received(false),
_alt_above_target(1.0),
_ref_traj_last_t(0.0),
_mpcWindow(20),
_state_weight(1.0),
_input_weight(0.1),
_smooth_input_weight(10),
_enable_control_smoothing(true),
_minAltitude(1.0),
_run_test_cases(false),
_is_MPC_initialized(false),
_engage_controller(false),
_is_mpc_engaged(false),
_save_mpc_data(false),
_plot(false)

{
   return;
}

MPC::~MPC(){return;}

bool
MPC::setDt(double dt)
{
   if(dt >0)
   {
      _dt = dt;
      printInfo("[MPC::setDt] dt = %f", _dt);
      return true;
   }
   else
   {
         printError("[MPC::setDt] dt < 0. Defaulting to 0.1");
         _dt = 0.1;
         return false;
   }
}

void
MPC::setDebug(bool d)
{
   _debug = d;
   printInfo("Debug = %d", (int) _debug);
   return;
}

bool
MPC::setMPCWindow(int N)
{
   if(N>0)
   {
      _mpcWindow = N;
      printInfo("_mpcWindow = %d", _mpcWindow);
      return true;
   }
   else
   {
      printError("_mpcWindow < 0. Default to 10");
      _mpcWindow = 10;
      return false;
   }
}

bool
MPC::set_state_weight(double w)
{
   if (w<0.0)
   {
      printError("state_weight =%f < 0", w);
      return false;
   }

   _state_weight = w;
   return true;
}

bool
MPC::set_input_weight(double w)
{
   if (w<0.0)
   {
      printError("_input_weight = %f < 0", w);
      return false;
   }

   _input_weight = w;
   return true;
}

bool
MPC::set_smooth_input_weight(double w)
{
   if (w<0.0)
   {
      printError("_smooth_input_weight = %f < 0", w);
      return false;
   }

   _smooth_input_weight = w;
   return true;
}

void
MPC::enable_control_smoothing(bool b)
{
   _enable_control_smoothing = b;
   printInfo("_enable_control_smoothing = %d", (int)_enable_control_smoothing);
   return;
}

bool
MPC::set_alt_above_target(double h)
{
   if(h<0)
   {
      printError("_alt_above_target = %f < 0", h);
      return false;
   }
   _alt_above_target = h;
   printInfo("_alt_above_target = %f", _alt_above_target);
   return true;
}

void
MPC::use_6dof_model(bool b)
{
   _use_6dof_model = b;
   printInfo("_use_6dof_model = %d", (int) _use_6dof_model);
}

bool
MPC::set_reference_frame_id(std::string s)
{
   _reference_frame_id = s;
   printInfo("_reference_frame_id = %s", _reference_frame_id.c_str());
   return true;
}


bool
MPC::set_minimum_altitude(double h)
{
   if(h<0)
   {
      printError("_minAltitude = %f <0", h);
      return false;
   }
   _minAltitude = h;
   return true;
}

bool
MPC::set_current_drone_state(Eigen::MatrixXd x)
{
   if(x.size() != NUM_OF_STATES)
   {
      printError("State vector size %d != %u", (int) x.size(), NUM_OF_STATES);
      return false;
   }
   _current_drone_state.resize(NUM_OF_STATES,1);
   _current_drone_state = x;
   return true;
}

bool 
MPC::set_current_drone_accel(Eigen::Matrix3d a)
{
   if(a.size() != 3)
   {
      printError("Inpur accelration vector size %d !=3", (int) a.size());
      return false;
   }
   _current_drone_accel = a;
}

bool
MPC::set_maxVel(std::vector<double> v)
{
   if(v.size() != 3)
   {
      printError("maxVel vector size %d != 3", (int) v.size());
      return false;
   }
   _maxVel.setZero();
   _maxVel(0) = v[0]; _maxVel(1) = v[1]; _maxVel(2) = v[2];
   return true;
}

bool
MPC::set_maxAccel(std::vector<double> a)
{
   if(a.size() != 3)
   {
      printError("maxAccel vector size %d != 3", (int) a.size());
      return false;
   }
   _maxAccel.setZero();
   _maxAccel(0) = a[0]; _maxAccel(1) = a[1]; _maxAccel(2) = a[2];
   return true;
}

bool
MPC::set_maxJerk(std::vector<double> j)
{
   if(j.size() != 3)
   {
      printError("maxJerk vector size %d != 3", (int) j.size());
      return false;
   }
   _maxJerk.setZero();
   _maxJerk(0) = j[0]; _maxJerk(1) = j[1]; _maxJerk(2) = j[2];
   return true;
}

void 
MPC::setQ(void)
{
   _Q.resize(NUM_OF_STATES,NUM_OF_STATES); _Q.setZero();
   _Q(0,0) = _state_weight; // penality on position, x
   _Q(3,3) = _state_weight; // penality on position, y
   _Q(6,6) = _state_weight; // penality on position, z

   if(_debug)
   {
      std::cout<<"[MPC] Q matrix = "<<std::endl<< _Q <<std::endl;
   }

   return;
}

void 
MPC::setQ6DoF(void)
{
   _Q.resize(NUM_OF_STATES,NUM_OF_STATES); _Q.setZero();
   _Q(0,0) = _state_weight; // penality on position, x
   _Q(2,2) = _state_weight; // penality on position, y
   _Q(4,4) = _state_weight; // penality on position, z

   if(_debug)
   {
      std::cout<<"[MPC] Q matrix = "<<std::endl<< _Q <<std::endl;
   }
   
   return;
}

void 
MPC::setR(void)
{
   _R.resize(NUM_OF_INPUTS,NUM_OF_INPUTS);
   _R = _input_weight * Eigen::MatrixXd::Identity(NUM_OF_INPUTS, NUM_OF_INPUTS);

   if(_debug)
   {
      std::cout<<"[MPC] R matrix: "<<std::endl<< _R <<std::endl;
   }

   return;
}

void 
MPC::setTransitionMatrix(void)
{
   // states: [px, vx, ax, py, vy, ay, pz, vz, az]

   Eigen::MatrixXd A_dt;
   A_dt = Eigen::MatrixXd::Identity(3,3); // Construct the smaller A matrix

   A_dt(0,1) = _dt; //A_dt(0,2) = _dt*_dt/2.0;
   A_dt(1,2) = _dt;
   //A_dt(2,2)=0; // ref: Model Predictive Trajectory Tracking and Collision Avoidance for Reliable Outdoor Deployment of Unmanned Aerial Vehicles

   _A.resize(NUM_OF_STATES,NUM_OF_STATES);
   _A = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_STATES);
   
   _A.block(0,0,3, 3) = A_dt;
   _A.block(3,3,3, 3) = A_dt;
   _A.block(6,6,3, 3) = A_dt;

   if(_debug)
   {
      std::cout<< "[MPC] Transition matrix A = " <<std::endl<< _A <<std::endl;
   }
   return;
}

void 
MPC::setTransitionMatrix6DoF(void)
{
   // states: [px, vx, py, vy, pz, vz]

   Eigen::MatrixXd A_dt;
   A_dt = Eigen::MatrixXd::Identity(2,2); // Construct the smaller A matrix

   A_dt(0,1) = _dt;

   _A.resize(NUM_OF_STATES,NUM_OF_STATES);
   _A = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_STATES);
   
   _A.block(0,0, 2, 2) = A_dt;
   _A.block(2,2, 2, 2) = A_dt;
   _A.block(4,4, 2, 2) = A_dt;

   if(_debug)
   {
      std::cout<< "[MPC] Transition matrix A = " <<std::endl<< _A <<std::endl;
   }
   return;
}

void 
MPC::setInputMatrix(void)
{
   _B.resize(NUM_OF_STATES, NUM_OF_INPUTS);
   _B.setZero();

   Eigen::MatrixXd B_dt;
   B_dt = Eigen::MatrixXd::Zero(3,1); // Build the blocks of _B
   // B_dt(0,0) = _dt*_dt/2; // _dt*_dt*_dt/6.0; 
   // B_dt(1,0) = _dt; //_dt*_dt/2.0;
   B_dt(2,0) = _dt;

   // block(start_row_index, start_col_index, number_of_rows, number_of_cols)
   _B.block(0,0,3,1) = B_dt;
   _B.block(3,1,3,1) = B_dt;
   _B.block(6,2,3,1) = B_dt;

   if(_debug)
   {
      std::cout<<"Input matrix B = " << std::endl<< _B <<std::endl;
   }
   return;
}

void 
MPC::setInputMatrix6DoF(void)
{
   _B.resize(NUM_OF_STATES, NUM_OF_INPUTS);
   _B.setZero();

   Eigen::MatrixXd B_dt;
   B_dt = Eigen::MatrixXd::Zero(2,1); // Build the blocks of _B
   B_dt(1,0) = _dt;

   _B.block(0,0, 2,1) = B_dt;
   _B.block(2,1, 2,1) = B_dt;
   _B.block(4,2, 2,1) = B_dt;

   if(_debug)
   {
      std::cout<<"Input matrix B = " << std::endl<< _B <<std::endl;
   }
   return;
}

void 
MPC::setStateBounds(void)
{
   _xMin.resize(NUM_OF_STATES);
   _xMin.setZero();
   _xMax.resize(NUM_OF_STATES);
   _xMax.setZero();

   // TODO Sanity checks on the values of maxVel, maxAccel

   // states order: [px, vx, ax, py, vy, ay, pz, vz, az]
   _xMin(0) = -1.0*OsqpEigen::INFTY;   _xMax(0) = OsqpEigen::INFTY; // px
   _xMin(1) = -1.0*_maxVel(0);         _xMax(1) = _maxVel(0); // vx
   _xMin(2) = -1.0*_maxAccel(0);       _xMax(2) = _maxAccel(0); // ax
   
   _xMin(3) = -1.0*OsqpEigen::INFTY;   _xMax(3) = OsqpEigen::INFTY; // py
   _xMin(4) = -1.0*_maxVel(1);         _xMax(4) = _maxVel(1); // vy
   _xMin(5) = -1.0*_maxAccel(1);       _xMax(5) = _maxAccel(1); // ay

   _xMin(6) = _minAltitude;            _xMax(6) = OsqpEigen::INFTY; // pz
   _xMin(7) = -1.0*_maxVel(2);         _xMax(7) = _maxVel(2); // vz
   _xMin(8) = -1.0*_maxAccel(2);       _xMax(8) = _maxAccel(2); // az

   return;
}

void 
MPC::setStateBounds6DoF(void)
{
   _xMin.resize(NUM_OF_STATES);
   _xMin.setZero();
   _xMax.resize(NUM_OF_STATES);
   _xMax.setZero();

   // TODO Sanity checks on the values of maxVel, maxAccel

   // states order: [px, vx, py, vy, pz, vz]
   _xMin(0) = -1.0*OsqpEigen::INFTY;   _xMax(0) = OsqpEigen::INFTY; // px
   _xMin(1) = -1.0*_maxVel(0);         _xMax(1) = _maxVel(0); // vx
   
   _xMin(2) = -1.0*OsqpEigen::INFTY;   _xMax(2) = OsqpEigen::INFTY; // py
   _xMin(3) = -1.0*_maxVel(1);         _xMax(3) = _maxVel(1); // vy

   _xMin(4) = _minAltitude;            _xMax(4) = OsqpEigen::INFTY; // pz
   _xMin(5) = -1.0*_maxVel(2);         _xMax(5) = _maxVel(2); // vz

   return;
}


void 
MPC::setControlBounds(void)
{
   // _uMin = -1.0*_maxJerk;
   // _uMax = _maxJerk;

   _uMin = -1.0*_maxAccel;
   _uMax = _maxAccel;

   return;
}


void 
MPC::castMPCToQPHessian(void)
{
   int h_size = NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _hessian.resize(h_size, h_size);
   _hessian = Eigen::MatrixXd::Zero(h_size, h_size);

   // Add _Q to _hessian
   for (unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _hessian.block(NUM_OF_STATES*i, NUM_OF_STATES*i, NUM_OF_STATES, NUM_OF_STATES) = _Q;
   }

   // Add _R to _hessian
   unsigned int idx = (_mpcWindow+1)*NUM_OF_STATES; //initial index after adding _Q
   for (unsigned int i=0; i<_mpcWindow; i++)
   {
      _hessian.block(idx+i*NUM_OF_INPUTS, idx+i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = _R;
   }

   if (_enable_control_smoothing)
   {
      Eigen::MatrixXd s = _smooth_input_weight * Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
      Eigen::MatrixXd S = Eigen::MatrixXd::Identity(NUM_OF_INPUTS*(_mpcWindow-1), NUM_OF_INPUTS*(_mpcWindow-1));
      // Input difference matrix
      Eigen::MatrixXd U_diff = Eigen::MatrixXd::Zero(NUM_OF_INPUTS*(_mpcWindow-1), NUM_OF_INPUTS*_mpcWindow);
      for (int i=0; i<(_mpcWindow-1); i++)
      {
         S.block(i*NUM_OF_INPUTS, i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = s;
         U_diff.block(i*NUM_OF_INPUTS, i*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = -1.0*Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
         U_diff.block(i*NUM_OF_INPUTS, (i+1)*NUM_OF_INPUTS, NUM_OF_INPUTS, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_INPUTS,NUM_OF_INPUTS);
      }

      Eigen::MatrixXd product = U_diff.transpose()*S*U_diff;
      auto N_x = NUM_OF_STATES*(_mpcWindow+1);
      auto N_u = NUM_OF_INPUTS*_mpcWindow;
      _hessian.block(N_x,N_x, N_u,N_u) = _hessian.block(N_x,N_x, N_u,N_u) + product;
   }

   return;
}

void 
MPC::castMPCToQPGradient(void)
{
   int g_size = NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow;
   _gradient.resize(g_size);
   _gradient.setZero();

   // Populate the gradient vector
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _gradient.segment(i*NUM_OF_STATES,NUM_OF_STATES) = -1.0*_Q*_referenceTraj.block(i*NUM_OF_STATES,0,NUM_OF_STATES,1);
   }

   if(_debug)
   {
      std::cout<<"QP gradient vector q = "<<std::endl<<_gradient<<std::endl;
   }

   return;
}


void 
MPC::updateQPGradientVector(void)
{
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _gradient.segment(i*NUM_OF_STATES,NUM_OF_STATES) = -1.0*_Q*_referenceTraj.block(i*NUM_OF_STATES,0,NUM_OF_STATES,1);
   }
   if (_debug)
   {
      std::cout << "[MPC::updateQPGradientVector] Updated QP gradient = \n" << _gradient << "\n";
   }

   return;
}


void 
MPC::castMPCToQPConstraintMatrix(void)
{
   // Initialize Ac
   int size_r = 2*NUM_OF_STATES * (_mpcWindow+1) + NUM_OF_INPUTS * _mpcWindow;
   int size_c = NUM_OF_STATES * (_mpcWindow+1) + NUM_OF_INPUTS * _mpcWindow;
   _Ac.resize(size_r, size_c);
   _Ac.setZero();
   //_Ac = Eigen::MatrixXd::Zero(size_r, size_c);

   // length of states/inputs over _mpcWindow
   auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS * _mpcWindow;

   _Ac.block(0, 0, N_x, N_x) = -1.0 * Eigen::MatrixXd::Identity(N_x,N_x);
   for (unsigned int i=1; i<_mpcWindow+1; i++)
   {
      // upper-left block
      _Ac.block(i*NUM_OF_STATES, (i-1)*NUM_OF_STATES, NUM_OF_STATES, NUM_OF_STATES) = _A;
      // upper-right block
      _Ac.block(i*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = _B;
      
      // input constraints: lower-right block
      //_Ac.block(2*N_x+(i-1)*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_INPUTS);
   }

   // Bounds: State block, middle-left block = Identity
   _Ac.block(N_x, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x,N_x);

   // Bounds: Inputs block, lower-right block = Identity
   _Ac.block(2*N_x, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u,N_u);

   if(_debug)
   {
      std::cout<<"Constraint matrix A_c = " <<std::endl<< _Ac <<std::endl;
   }

   return;
}


void 
MPC::castMPCToQPConstraintBounds(void)
{
   // length of states/inputs over _mpcWindow
   // _mpcWindow+1, because x(0) is included
   auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS * _mpcWindow;

   _lowerBounds.resize(2*N_x + N_u,1);
   _lowerBounds.setZero();
   _upperBounds.resize(2*N_x + N_u,1);
   _upperBounds.setZero();

   // Equality bounds
   _lowerBounds.block(0,0, NUM_OF_STATES,1) = -1.0*_current_drone_state;
   _upperBounds.block(0,0, NUM_OF_STATES,1) = -1.0*_current_drone_state;

   // Inequality bounds for the states
   // WARNING Assuming _xMin, _xMax are already set
   for(unsigned int i=0; i<_mpcWindow+1; i++)
   {
      _lowerBounds.block(N_x+i*NUM_OF_STATES,0, NUM_OF_STATES,1) = _xMin;
      _upperBounds.block(N_x+i*NUM_OF_STATES,0, NUM_OF_STATES,1) = _xMax;
   }
   // Inequality bounds for the controls, u
   // WARNING Assuming _uMin, _uMax are already set
   for(unsigned int i=0; i<_mpcWindow; i++)
   {
      _lowerBounds.block(2*N_x+i*NUM_OF_INPUTS, 0, NUM_OF_INPUTS,1) = _uMin;
      _upperBounds.block(2*N_x+i*NUM_OF_INPUTS,0, NUM_OF_INPUTS,1) = _uMax;
   }

   if(_debug)
   {
      std::cout<<"Lower bounds vector l = "<<std::endl<< _lowerBounds <<std::endl;
      std::cout<<"Upper bounds vector u = "<<std::endl<< _upperBounds <<std::endl;
   }

   return;
}

void 
MPC::updateQPConstraintsBounds(void)
{
   // Equality for x(0)
   _lowerBounds.block(0,0,NUM_OF_STATES,1) = -1.0*_current_drone_state;
   _upperBounds.block(0,0,NUM_OF_STATES,1) = -1.0*_current_drone_state;

   // auto N_x = NUM_OF_STATES*(_mpcWindow+1);
   // // Equality (in the form of inequality), for x(0)
   // _lowerBounds.block(N_x,0, NUM_OF_STATES,1) = _current_drone_state;
   // _upperBounds.block(N_x,0, NUM_OF_STATES,1) = _current_drone_state;
   if(_debug)
   {
      printInfo("[MPC::updateQPConstraintsBounds] QP bounds are updated");
      std::cout << "[MPC::updateQPConstraintsBounds] Updated lower bound,l = \n " << _lowerBounds << "\n";
      std::cout << "[MPC::updateQPConstraintsBounds] Updated upper bound,u = \n " << _upperBounds << "\n";
   }

   return;
}


bool 
MPC::initQPSolver(void)
{
   printInfo("Initializing QP solver ...");

   _qpSolver.settings()->setVerbosity(_debug);
   _qpSolver.settings()->setWarmStart(true);
   // set the initial data of the QP solver
   _qpSolver.data()->setNumberOfVariables(NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow);
   _qpSolver.data()->setNumberOfConstraints(2*NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow);
   _hessian_sparse = _hessian.sparseView();
   _Ac_sparse = _Ac.sparseView();
   if(!_qpSolver.data()->setHessianMatrix(_hessian_sparse)) return false;
   if(!_qpSolver.data()->setGradient(_gradient)) return false;
   if(!_qpSolver.data()->setLinearConstraintsMatrix(_Ac_sparse)) return false;
   if(!_qpSolver.data()->setLowerBound(_lowerBounds)) return false;
   if(!_qpSolver.data()->setUpperBound(_upperBounds)) return false;

   if(!_qpSolver.initSolver()) return false;


   if(_debug)
   {
      printInfo("QP solver is initialized.");
   }
   return true;
}


bool 
MPC::initMPCProblem(void)
{
   //  Sanity check
   if (_use_6dof_model)
   {
      if (NUM_OF_STATES != 6)
      {
         printError("[initMPCProblem] NUM_OF_STATES should be equal to 6 for 6-state model. Exiting...");
         return false;
      }
   }
   else
   {
      if (NUM_OF_STATES != 9)
      {
         printError("[initMPCProblem] NUM_OF_STATES should be equal to 9 for 9-state model. Exiting...");
         return false;
      }
   }

   if (_use_6dof_model)
   {
      setTransitionMatrix6DoF(); 
      setInputMatrix6DoF();
      setQ6DoF();
   }
   else
   {
      setTransitionMatrix(); 
      setInputMatrix();
      setQ();
   }
   setR();
   castMPCToQPHessian();
   _current_drone_state.resize(NUM_OF_STATES,1);
   _current_drone_state.setZero();
   _referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_STATES*(_mpcWindow+1),1);
   castMPCToQPGradient();
   castMPCToQPConstraintMatrix();
   if (_use_6dof_model)
      setStateBounds6DoF();
   else
      setStateBounds();
      
   setControlBounds();
   castMPCToQPConstraintBounds();
   if (!initQPSolver())
   {
      printError("[MPC::initMPCProblem] MPC initialization is not successful.");
      return false;
   }

   // Decide on the size of vectors in the initialization instead of doing it repeatidly in the callbacks
  // This is to avoid repeated memory (re)allocation, to save time.
  _referenceTraj.resize(NUM_OF_STATES*(_mpcWindow+1),1);

  printProblemInfo();


   _is_MPC_initialized = true;

   // double total_elapsed = (ros::WallTime::now() - startTime).toSec();
   // ROS_INFO("MPC is initialized in %f second(s).", total_elapsed);

   return true;
}


bool 
MPC::updateQP(void)
{
   // update current drone's position (updates QP linear constraints bounds)
   updateQPConstraintsBounds();
   if(!_qpSolver.updateBounds(_lowerBounds, _upperBounds))
   {
      printInfo("_qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      printInfo("[MPCTracker::updateQP] Bounds are updated.");

   
   // Update the QP gradient vector using  new _referenceTraj
   updateQPGradientVector();
   if(!_qpSolver.updateGradient(_gradient))
   {
      printInfo("_qpSolver.updateGradient failed to update _gradient");
      return false;
   }
   if(_debug)
      printInfo("QP gradient is updated");

   return true;
}

bool 
MPC::mpcLoop(void)
{
   // ros::WallTime startTime = ros::WallTime::now();

   if(!_is_MPC_initialized)
   {
      printWarn("MPC controller is not initialized. Skipping MPC loop.");
      return false;
   }
   if(!_drone_state_received)
   {
      printWarn("[MPC controller] Drone state is not received yet. Skipping MPC loop.");
      return false;
   }

   // Update gradient and bounds
   if(!updateQP())
   {
      printError("[mpcLoop] Failed to update bounds and gradient");
      return false;
   }
   
   // Solve MPC
   if(!_qpSolver.solve())
   {
      printError("MPC solution is not found");
      return false;
   }

   // if(_debug)
   // {
   //    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
   //    ROS_INFO("MPC is solved in %f second(s).", total_elapsed);
   // }

   return true;
}


void 
MPC::extractSolution(void)
{
   Eigen::VectorXd QPSolution;
   QPSolution = _qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _optimal_state_traj = QPSolution.block(0, 0, NUM_OF_STATES * (_mpcWindow+1), 1);

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS*_mpcWindow;
   _optimal_control_traj = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _mpc_ctrl_sol = _optimal_control_traj.segment(0,NUM_OF_INPUTS);

   _optimal_traj_px.resize(_mpcWindow+1);
   _optimal_traj_py.resize(_mpcWindow+1);
   _optimal_traj_pz.resize(_mpcWindow+1);
   _optimal_traj_vx.resize(_mpcWindow+1);
   _optimal_traj_vy.resize(_mpcWindow+1);
   _optimal_traj_vz.resize(_mpcWindow+1);
   _optimal_traj_ax.resize(_mpcWindow+1);
   _optimal_traj_ay.resize(_mpcWindow+1);
   _optimal_traj_az.resize(_mpcWindow+1);

   _optimal_traj_ux.resize(_mpcWindow);
   _optimal_traj_uy.resize(_mpcWindow);
   _optimal_traj_uz.resize(_mpcWindow);

   _ref_traj_px.resize(_mpcWindow+1);
   _ref_traj_py.resize(_mpcWindow+1);
   _ref_traj_pz.resize(_mpcWindow+1);
   _ref_traj_vx.resize(_mpcWindow+1);
   _ref_traj_vy.resize(_mpcWindow+1);
   _ref_traj_vz.resize(_mpcWindow+1);
   _ref_traj_ax.resize(_mpcWindow+1);
   _ref_traj_ay.resize(_mpcWindow+1);
   _ref_traj_az.resize(_mpcWindow+1);

   // _solution_traj_msg.states.resize(_mpcWindow);

   // Update _posehistory_vector for visualiztion
   // _posehistory_vector.resize(_mpcWindow+1);
   // geometry_msgs::PoseStamped pose_msg;
   // auto start_t = ros::Time::now();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      // pose_msg.header.frame_id="map";
      // pose_msg.header.stamp = start_t + ros::Duration(i*_dt);
      // pose_msg.pose.position.x = _optimal_state_traj(i*NUM_OF_STATES+0);
      // pose_msg.pose.position.y = _optimal_state_traj(i*NUM_OF_STATES+3);
      // pose_msg.pose.position.z = _optimal_state_traj(i*NUM_OF_STATES+6);
      // pose_msg.pose.orientation.w=1.0; // Keep 0 rotation
      // _posehistory_vector[i] = pose_msg;

      _optimal_traj_px(i) = _optimal_state_traj(i*NUM_OF_STATES+0);
      _optimal_traj_py(i) = _optimal_state_traj(i*NUM_OF_STATES+3);
      _optimal_traj_pz(i) = _optimal_state_traj(i*NUM_OF_STATES+6);

      _optimal_traj_vx(i) = _optimal_state_traj(i*NUM_OF_STATES+1);
      _optimal_traj_vy(i) = _optimal_state_traj(i*NUM_OF_STATES+4);
      _optimal_traj_vz(i) = _optimal_state_traj(i*NUM_OF_STATES+7);

      _optimal_traj_ax(i) = _optimal_state_traj(i*NUM_OF_STATES+2);
      _optimal_traj_ay(i) = _optimal_state_traj(i*NUM_OF_STATES+5);
      _optimal_traj_az(i) = _optimal_state_traj(i*NUM_OF_STATES+8);

      _ref_traj_px(i) = _referenceTraj(i*NUM_OF_STATES+0, 0);
      _ref_traj_py(i) = _referenceTraj(i*NUM_OF_STATES+3, 0);
      _ref_traj_pz(i) = _referenceTraj(i*NUM_OF_STATES+6, 0);

      _ref_traj_vx(i) = _referenceTraj(i*NUM_OF_STATES+1, 0);
      _ref_traj_vy(i) = _referenceTraj(i*NUM_OF_STATES+4, 0);
      _ref_traj_vz(i) = _referenceTraj(i*NUM_OF_STATES+7, 0);

      _ref_traj_ax(i) = _referenceTraj(i*NUM_OF_STATES+2, 0);
      _ref_traj_ay(i) = _referenceTraj(i*NUM_OF_STATES+5, 0);
      _ref_traj_az(i) = _referenceTraj(i*NUM_OF_STATES+8, 0);

      if(i<_mpcWindow)
      {
         _optimal_traj_ux(i) = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         _optimal_traj_uy(i) = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         _optimal_traj_uz(i) = _optimal_control_traj(i*NUM_OF_INPUTS+2);

         // Fill ROS msg
         // _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         // _solution_traj_msg.states[i].position.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+0 );
         // _solution_traj_msg.states[i].position.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+3 );
         // _solution_traj_msg.states[i].position.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+6 );
         // _solution_traj_msg.states[i].velocity.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+1 );
         // _solution_traj_msg.states[i].velocity.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+4 );
         // _solution_traj_msg.states[i].velocity.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+7 );
         // _solution_traj_msg.states[i].acceleration.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+2 );
         // _solution_traj_msg.states[i].acceleration.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+5 );
         // _solution_traj_msg.states[i].acceleration.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+8 );
      }
   }

   // _solution_traj_msg.header.stamp = start_t;
   // _solution_traj_msg.header.frame_id = _reference_frame_id;
   // _solution_traj_msg.max_acceleration = _maxAccel. maxCoeff();
   // _solution_traj_msg.max_velocity = _maxVel. maxCoeff();

   return;
}

void 
MPC::extractSolution6Dof(void)
{
   Eigen::VectorXd QPSolution;
   QPSolution = _qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _optimal_state_traj = QPSolution.block(0, 0, NUM_OF_STATES * (_mpcWindow+1), 1);

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_INPUTS*_mpcWindow;
   _optimal_control_traj = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _mpc_ctrl_sol = _optimal_control_traj.segment(0,NUM_OF_INPUTS);

   _optimal_traj_px.resize(_mpcWindow+1);
   _optimal_traj_py.resize(_mpcWindow+1);
   _optimal_traj_pz.resize(_mpcWindow+1);
   _optimal_traj_vx.resize(_mpcWindow+1);
   _optimal_traj_vy.resize(_mpcWindow+1);
   _optimal_traj_vz.resize(_mpcWindow+1);

   _optimal_traj_ux.resize(_mpcWindow);
   _optimal_traj_uy.resize(_mpcWindow);
   _optimal_traj_uz.resize(_mpcWindow);

   _ref_traj_px.resize(_mpcWindow+1);
   _ref_traj_py.resize(_mpcWindow+1);
   _ref_traj_pz.resize(_mpcWindow+1);
   _ref_traj_vx.resize(_mpcWindow+1);
   _ref_traj_vy.resize(_mpcWindow+1);
   _ref_traj_vz.resize(_mpcWindow+1);

   // _solution_traj_msg.states.resize(_mpcWindow);

   // Update _posehistory_vector for visualiztion
   // _posehistory_vector.resize(_mpcWindow+1);
   // geometry_msgs::PoseStamped pose_msg;
   // auto start_t = ros::Time::now();
   for (int i=0; i < _mpcWindow+1; i++)
   {
      // pose_msg.header.frame_id=_reference_frame_id;
      // pose_msg.header.stamp = start_t + ros::Duration(i*_dt);
      // pose_msg.pose.position.x = _optimal_state_traj(i*NUM_OF_STATES+0);
      // pose_msg.pose.position.y = _optimal_state_traj(i*NUM_OF_STATES+2);
      // pose_msg.pose.position.z = _optimal_state_traj(i*NUM_OF_STATES+4);
      // pose_msg.pose.orientation.w=1.0; // Keep 0 rotation, for now
      // // _posehistory_vector.insert(_posehistory_vector.begin(), pose_msg);
      // _posehistory_vector[i] = pose_msg;

      _optimal_traj_px(i) = _optimal_state_traj(i*NUM_OF_STATES+0);
      _optimal_traj_py(i) = _optimal_state_traj(i*NUM_OF_STATES+2);
      _optimal_traj_pz(i) = _optimal_state_traj(i*NUM_OF_STATES+4);

      _optimal_traj_vx(i) = _optimal_state_traj(i*NUM_OF_STATES+1);
      _optimal_traj_vy(i) = _optimal_state_traj(i*NUM_OF_STATES+3);
      _optimal_traj_vz(i) = _optimal_state_traj(i*NUM_OF_STATES+5);

      _ref_traj_px(i) = _referenceTraj(i*NUM_OF_STATES+0, 0);
      _ref_traj_py(i) = _referenceTraj(i*NUM_OF_STATES+2, 0);
      _ref_traj_pz(i) = _referenceTraj(i*NUM_OF_STATES+4, 0);

      _ref_traj_vx(i) = _referenceTraj(i*NUM_OF_STATES+1, 0);
      _ref_traj_vy(i) = _referenceTraj(i*NUM_OF_STATES+3, 0);
      _ref_traj_vz(i) = _referenceTraj(i*NUM_OF_STATES+5, 0);

      if(i<_mpcWindow)
      {
         _optimal_traj_ux(i) = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         _optimal_traj_uy(i) = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         _optimal_traj_uz(i) = _optimal_control_traj(i*NUM_OF_INPUTS+2);

         // Fill ROS msg
         // _solution_traj_msg.states[i].time_from_start = (i+1)*_dt;
         // _solution_traj_msg.states[i].position.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+0 );
         // _solution_traj_msg.states[i].position.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+2 );
         // _solution_traj_msg.states[i].position.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+4 );
         // _solution_traj_msg.states[i].velocity.x = _optimal_state_traj( (i+1)*NUM_OF_STATES+1 );
         // _solution_traj_msg.states[i].velocity.y = _optimal_state_traj( (i+1)*NUM_OF_STATES+3 );
         // _solution_traj_msg.states[i].velocity.z = _optimal_state_traj( (i+1)*NUM_OF_STATES+5 );
         // _solution_traj_msg.states[i].acceleration.x = _optimal_control_traj(i*NUM_OF_INPUTS+0);
         // _solution_traj_msg.states[i].acceleration.y = _optimal_control_traj(i*NUM_OF_INPUTS+1);
         // _solution_traj_msg.states[i].acceleration.z = _optimal_control_traj(i*NUM_OF_INPUTS+2);
      }
   }

   // _solution_traj_msg.header.stamp = start_t;
   // _solution_traj_msg.header.frame_id = _reference_frame_id;
   // _solution_traj_msg.max_acceleration = _maxAccel. maxCoeff();
   // _solution_traj_msg.max_velocity = _maxVel. maxCoeff();

   return;
}

void 
MPC::printProblemInfo(void)
{
   auto opt_x_l = NUM_OF_STATES*(_mpcWindow+1)+NUM_OF_INPUTS*_mpcWindow; // length of optimization variable
   auto opt_Ac_l_row = 2*NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow;// number of constraints in matrix Ac

   auto opt_Ac_size = opt_Ac_l_row * opt_x_l; // Number of elements in Ac

   printInfo("[MPC::printProblemInfo]: Number of states = %d",NUM_OF_STATES );
   printInfo("[MPC::printProblemInfo]: Number of inputs = %d",NUM_OF_INPUTS );
   printInfo("[MPC::printProblemInfo]: Number of MPC steps = %d",_mpcWindow );
   printInfo("[MPC::printProblemInfo]: Length of MPC horizon = %f seconds",(double)_mpcWindow*_dt );
   printInfo("[MPC::printProblemInfo]: Number of optimization variables = %d",opt_x_l );
   printInfo("[MPC::printProblemInfo]: Number constraints = %d", opt_Ac_l_row );
   printInfo("[MPC::printProblemInfo]: Number of elements in the constraints matrix Ac = %d",opt_Ac_size );
   return;
}


void 
MPC::testCases(void)
{
   return;
}


void 
MPC::saveMPCDataToFile(void)
{
   return;
}


void 
MPC::plotSolutions(void)
{
   return;
}

bool
MPC::set_referenceTraj(Eigen::MatrixXd v)
{
   if((int)(_referenceTraj.size()) != (int)(NUM_OF_STATES*(_mpcWindow+1)) )
   {
      printError("[MPC::set_referenceTraj] input matrix size %d != %d", (int)(_referenceTraj.size()), (int)(NUM_OF_STATES*(_mpcWindow+1)) );
      return false;
   }
   _referenceTraj = v;
   return true;
}

Eigen::VectorXd
MPC::get_optimal_state_traj(void)
{
   return _optimal_state_traj;
}

Eigen::VectorXd
MPC::get_optimal_control_traj(void)
{
   return _optimal_control_traj;
}

int
MPC::get_num_of_states(void)
{
   return (int)NUM_OF_STATES;
}

int
MPC::get_num_of_inputs(void)
{
   return (int)NUM_OF_INPUTS;
}

Eigen::Vector3d
MPC::get_maxAccel(void)
{
   return _maxAccel;
}

Eigen::Vector3d
MPC::get_maxVel(void)
{
   return _maxVel;
}

Eigen::Vector3d
MPC::get_maxJerk(void)
{
   return _maxJerk;
}