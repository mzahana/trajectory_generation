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

/**
 * @todo resize _xy_Min, _xy_Max, _xy_refTraj, _z_refTraj, _yaw_refTraj, _refTraj
 *    _current_state
 * in the initialization
*/
#include "trajectory_generation/mpc_12state.hpp"

MPC12STATE::MPC12STATE():
_debug(false),
_dt(0.05),
_state_received(false),
_last_state_time(0.0),
_current_state_time(0.0),
_target_traj_received(false),
_alt_above_target(1.0),
_ref_traj_last_t(0.0),
_mpcWindow(20),
_xy_state_weight(1.0),
_z_state_weight(1.0),
_yaw_state_weight(1.0),
_xy_input_weight(0.1),
_z_input_weight(0.1),
_yaw_input_weight(0.1),
_xy_smooth_input_weight(10),
_z_smooth_input_weight(10),
_yaw_smooth_input_weight(10),
_received_refTraj(false),
_enable_control_smoothing(false),
_minAltitude(1.0),
_is_MPC_initialized(false),
_save_mpc_data(false)

{
   return;
}

MPC12STATE::~MPC12STATE(){return;}



void 
MPC12STATE::setXYQ(void)
{
   // state: [x, x_dot, x_ddot, y, y_dot, y_ddot]
   _xy_Q.setZero();
   _xy_Q(0,0) = _xy_state_weight; // penality on position, x
   _xy_Q(3,3) = _xy_state_weight; // penality on position, y

   if(_debug)
   {
      std::cout<<"[MPC] _xy_Q matrix = "<< std::endl<< _xy_Q <<std::endl;
   }
   
   return;
}

void 
MPC12STATE::setZQ(void)
{
   // state: [z, z_dot, z_ddot]
   _z_Q.setZero();
   _z_Q(0,0) = _z_state_weight; // penality on position, z

   if(_debug)
   {
      std::cout<<"[MPC] _z_Q matrix = "<< std::endl<< _z_Q <<std::endl;
   }
   
   return;
}

void 
MPC12STATE::setYawQ(void)
{
   // state: [yaw, yaw_dot, yaw_ddot]
   _yaw_Q.setZero();
   _yaw_Q(0,0) = _yaw_state_weight; // penality on position, yaw

   if(_debug)
   {
      std::cout<<"[MPC] _yaw_Q matrix = "<< std::endl<< _yaw_Q <<std::endl;
   }
   
   return;
}

void 
MPC12STATE::setXYR(void)
{
   _xy_R = _xy_input_weight * _xy_R.setIdentity();

   if(_debug)
   {
      std::cout<<"[MPC] _xy_R matrix: "<<std::endl<< _xy_R <<std::endl;
   }

   return;
}

void 
MPC12STATE::setZR(void)
{
   _z_R = _z_input_weight * _z_R.setIdentity();

   if(_debug)
   {
      std::cout<<"[MPC] _z_R matrix: "<<std::endl<< _z_R <<std::endl;
   }

   return;
}

void 
MPC12STATE::setYawR(void)
{
   _yaw_R = _yaw_input_weight * _yaw_R.setIdentity();

   if(_debug)
   {
      std::cout<<"[MPC] _yaw_R matrix: "<<std::endl<< _yaw_R <<std::endl;
   }

   return;
}


void 
MPC12STATE::setXYTransitionMatrix(void)
{
   // state: [x, x_dot, x_ddot, y, y_dot, y_ddot]
   _xy_A.setIdentity();   
   _xy_A(0,1) = _xy_A(1,2) = _xy_A(3,4) = _xy_A(4,5) = _dt;
   _xy_A(0,2) = _xy_A(3,5) = _dt*_dt/2.0;
   if(_debug)
   {
      std::cout<< "[MPC] Transition matrix _xy_A = " <<std::endl<< _xy_A <<std::endl;
   }
   return;
}

void 
MPC12STATE::setZTransitionMatrix(void)
{
   // state: [z, z_dot, z_ddot]
   _z_A.setIdentity();   
   _z_A(0,1) = _z_A(1,2) = _dt;
   _z_A(0,2) = _dt*_dt/2.0;
   if(_debug)
   {
      std::cout<< "[MPC] Transition matrix _z_A = " <<std::endl<< _z_A <<std::endl;
   }
   return;
}

void 
MPC12STATE::setYawTransitionMatrix(void)
{
   // state: [yaw, yaw_dot, yaw_ddot]
   _yaw_A.setIdentity();   
   _yaw_A(0,1) = _yaw_A(1,2) = _dt;
   _yaw_A(0,2) = _dt*_dt/2.0;
   if(_debug)
   {
      std::cout<< "[MPC] Transition matrix _yaw_A = " <<std::endl<< _yaw_A <<std::endl;
   }
   return;
}

void 
MPC12STATE::setXYInputMatrix(void)
{
   _xy_B.setZero();
   _xy_B(2,0) = _xy_B(5,1) = _dt;
   if(_debug)
   {
      std::cout<< "Input matrix _xy_B = " << std::endl<< _xy_B <<std::endl;
   }
   return;
}

void 
MPC12STATE::setZInputMatrix(void)
{
   _z_B.setZero();
   _z_B(2,0) = _dt;
   if(_debug)
   {
      std::cout<< "Input matrix _z_B = " << std::endl<< _z_B <<std::endl;
   }
   return;
}

void 
MPC12STATE::setYawInputMatrix(void)
{
   _yaw_B.setZero();
   _yaw_B(2,0) = _dt;
   if(_debug)
   {
      std::cout<< "Input matrix _yaw_B = " << std::endl<< _yaw_B <<std::endl;
   }
   return;
}

void 
MPC12STATE::setXYStateBounds(void)
{
   // This function is no useable in this implementation since the xy velocity constraints
   // are functions of the generated solutions of the vel/accel of the z dynamics
   // _xy_Min.setZero();
   // _xy_Max.setZero();


   // // state: [x, x_dot, x_ddot, y, y_dot, y_ddot]
   // _xMin(0) = -1.0*OsqpEigen::INFTY;   _xMax(0) = OsqpEigen::INFTY; // px
   // _xMin(1) = -1.0*OsqpEigen::INFTY;   _xMax(1) = OsqpEigen::INFTY; // py
   // _xMin(2) = _minAltitude;            _xMax(2) = OsqpEigen::INFTY; // pz
   
   // _xMin(3) = -1.0*_maxVel(0);   _xMax(3) = _maxVel(0); // vx
   // _xMin(4) = -1.0*_maxVel(1);   _xMax(4) = _maxVel(1); // vy
   // _xMin(5) = -1.0*_maxVel(2);   _xMax(5) = _maxVel(2); // vz

   return;
}

void 
MPC12STATE::setZStateBounds(void)
{
   _z_Min.setZero();
   _z_Max.setZero();
   // // state: [z, z_dot, z_ddot]
   _z_Min(0,0) = -1.0*OsqpEigen::INFTY;   _z_Max(0,0) = OsqpEigen::INFTY;
   _z_Min(1,0) = -1.0*_z_MaxVel;          _z_Max(1,0) = _z_MaxVel;
   _z_Min(2,0) = -1.0*_z_MaxAccel;        _z_Max(2,0) = _z_MaxAccel;

   return;
}

void 
MPC12STATE::setYawStateBounds(void)
{
   _yaw_Min.setZero();
   _yaw_Max.setZero();
   // // state: [yaw, yaw_dot, yaw_ddot]
   _yaw_Min(0,0) = -2.0*M_PI;          _yaw_Max(0,0) = 2*M_PI;
   _yaw_Min(1,0) = -1.0*_yaw_MaxVel;   _yaw_Max(1,0) = _yaw_MaxVel;
   _yaw_Min(2,0) = -1.0*_yaw_MaxAccel; _yaw_Max(2,0) = _yaw_MaxAccel;

   return;
}

void 
MPC12STATE::setXYControlBounds(void)
{
   // jerk bounds
   _xy_uMin.setOnes(); _xy_uMax.setOnes();
   _xy_uMin = -1.0*_xy_MaxJerk*_xy_uMin;
   _xy_uMax = _xy_MaxJerk*_xy_uMax;
   return;
}

void 
MPC12STATE::setZControlBounds(void)
{
   // jerk bounds
   _z_uMin = -1.0*_z_MaxJerk;
   _z_uMax = _z_MaxJerk;
   return;
}

void 
MPC12STATE::setYawControlBounds(void)
{
   // jerk bounds
   _yaw_uMin = -1.0*_yaw_MaxJerk;
   _yaw_uMax = _yaw_MaxJerk;
   return;
}

void 
MPC12STATE::castXYMPCToQPHessian(void)
{
   auto h_size = NUM_OF_XY_STATES*(_mpcWindow+1) + NUM_OF_XY_INPUTS*_mpcWindow;
   _xy_hessian = Eigen::MatrixXd::Zero(h_size, h_size);

   // Add _Q to _hessian
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _xy_hessian.block(NUM_OF_XY_STATES*i, NUM_OF_XY_STATES*i, NUM_OF_XY_STATES, NUM_OF_XY_STATES) = _xy_Q;
   }

   // Add _R to _hessian
   int idx = (_mpcWindow+1)*NUM_OF_XY_STATES; //initial index after adding _Q
   for (int i=0; i<_mpcWindow; i++)
   {
      _xy_hessian.block(idx+i*NUM_OF_XY_INPUTS, idx+i*NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS) = _xy_R;
   }

   if (_enable_control_smoothing)
   {
      MatUbyU_XY s = _xy_smooth_input_weight * MatUbyU_XY::Identity();
      Eigen::MatrixXd S = Eigen::MatrixXd::Identity(NUM_OF_XY_INPUTS*(_mpcWindow-1), NUM_OF_XY_INPUTS*(_mpcWindow-1));
      // Input difference matrix
      Eigen::MatrixXd U_diff = Eigen::MatrixXd::Zero(NUM_OF_XY_INPUTS*(_mpcWindow-1), NUM_OF_XY_INPUTS*_mpcWindow);
      for (int i=0; i<(_mpcWindow-1); i++)
      {
         S.block(i*NUM_OF_XY_INPUTS, i*NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS) = s;
         U_diff.block(i*NUM_OF_XY_INPUTS, i*NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS) = -1.0*MatUbyU_XY::Identity();
         U_diff.block(i*NUM_OF_XY_INPUTS, (i+1)*NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS) = MatUbyU_XY::Identity();
      }

      Eigen::MatrixXd product = U_diff.transpose()*S*U_diff;
      auto N_x = NUM_OF_XY_STATES*(_mpcWindow+1);
      auto N_u = NUM_OF_XY_INPUTS*_mpcWindow;
      _xy_hessian.block(N_x,N_x, N_u,N_u) = _xy_hessian.block(N_x,N_x, N_u,N_u) + product;
   }

   return;
}

void 
MPC12STATE::castZMPCToQPHessian(void)
{
   int h_size = NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _z_hessian = Eigen::MatrixXd::Zero(h_size, h_size);

   // Add _Q to _hessian
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _z_hessian.block(NUM_OF_Z_STATES*i, NUM_OF_Z_STATES*i, NUM_OF_Z_STATES, NUM_OF_Z_STATES) = _z_Q;
   }

   // Add _R to _hessian
   int idx = (_mpcWindow+1)*NUM_OF_Z_STATES; //initial index after adding _Q
   for (int i=0; i<_mpcWindow; i++)
   {
      _z_hessian.block(idx+i*NUM_OF_Z_INPUTS, idx+i*NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS) = _z_R;
   }

   if (_enable_control_smoothing)
   {
      MatUbyU_Z s = _z_smooth_input_weight * MatUbyU_Z::Identity();
      Eigen::MatrixXd S = Eigen::MatrixXd::Identity(NUM_OF_Z_INPUTS*(_mpcWindow-1), NUM_OF_Z_INPUTS*(_mpcWindow-1));
      // Input difference matrix
      Eigen::MatrixXd U_diff = Eigen::MatrixXd::Zero(NUM_OF_Z_INPUTS*(_mpcWindow-1), NUM_OF_Z_INPUTS*_mpcWindow);
      for (int i=0; i<(_mpcWindow-1); i++)
      {
         S.block(i*NUM_OF_Z_INPUTS, i*NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS) = s;
         U_diff.block(i*NUM_OF_Z_INPUTS, i*NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS) = -1.0*MatUbyU_Z::Identity();
         U_diff.block(i*NUM_OF_Z_INPUTS, (i+1)*NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS) = MatUbyU_Z::Identity();
      }

      Eigen::MatrixXd product = U_diff.transpose()*S*U_diff;
      auto N_x = NUM_OF_Z_STATES*(_mpcWindow+1);
      auto N_u = NUM_OF_Z_INPUTS*_mpcWindow;
      _z_hessian.block(N_x,N_x, N_u,N_u) = _z_hessian.block(N_x,N_x, N_u,N_u) + product;
   }

   return;
}

void 
MPC12STATE::castYawMPCToQPHessian(void)
{
   int h_size = NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _yaw_hessian = Eigen::MatrixXd::Zero(h_size, h_size);

   // Add _Q to _hessian
   for (int i=0; i<_mpcWindow+1; i++)
   {
      _yaw_hessian.block(NUM_OF_YAW_STATES*i, NUM_OF_YAW_STATES*i, NUM_OF_YAW_STATES, NUM_OF_YAW_STATES) = _yaw_Q;
   }

   // Add _R to _hessian
   int idx = (_mpcWindow+1)*NUM_OF_YAW_STATES; //initial index after adding _Q
   for (int i=0; i<_mpcWindow; i++)
   {
      _yaw_hessian.block(idx+i*NUM_OF_YAW_INPUTS, idx+i*NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS) = _yaw_R;
   }

   if (_enable_control_smoothing)
   {
      MatUbyU_Z s = _yaw_smooth_input_weight * MatUbyU_YAW::Identity();
      Eigen::MatrixXd S = Eigen::MatrixXd::Identity(NUM_OF_YAW_INPUTS*(_mpcWindow-1), NUM_OF_YAW_INPUTS*(_mpcWindow-1));
      // Input difference matrix
      Eigen::MatrixXd U_diff = Eigen::MatrixXd::Zero(NUM_OF_YAW_INPUTS*(_mpcWindow-1), NUM_OF_YAW_INPUTS*_mpcWindow);
      for (int i=0; i<(_mpcWindow-1); i++)
      {
         S.block(i*NUM_OF_YAW_INPUTS, i*NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS) = s;
         U_diff.block(i*NUM_OF_YAW_INPUTS, i*NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS) = -1.0*MatUbyU_YAW::Identity();
         U_diff.block(i*NUM_OF_YAW_INPUTS, (i+1)*NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS, NUM_OF_YAW_INPUTS) = MatUbyU_YAW::Identity();
      }

      Eigen::MatrixXd product = U_diff.transpose()*S*U_diff;
      auto N_x = NUM_OF_YAW_STATES*(_mpcWindow+1);
      auto N_u = NUM_OF_YAW_INPUTS*_mpcWindow;
      _yaw_hessian.block(N_x,N_x, N_u,N_u) = _yaw_hessian.block(N_x,N_x, N_u,N_u) + product;
   }

   return;
}

void 
MPC12STATE::castXYMPCToQPGradient(void)
{
   _xy_gradient.setZero();

   // Populate the gradient vector
   for(int i=0; i<_mpcWindow+1; i++)
   {
      _xy_gradient.segment(i*NUM_OF_XY_STATES,NUM_OF_XY_STATES) = -1.0*_xy_Q*_xy_referenceTraj.block(i*NUM_OF_XY_STATES,0,NUM_OF_XY_STATES,1);
   }

   if(_debug)
   {
      std::cout<<"XY QP gradient vector q = "<<std::endl<<_xy_gradient<<std::endl;
   }

   return;
}

void 
MPC12STATE::castZMPCToQPGradient(void)
{
   _z_gradient.setZero();

   // Populate the gradient vector
   for(int i=0; i<_mpcWindow+1; i++)
   {
      _z_gradient.segment(i*NUM_OF_Z_STATES,NUM_OF_Z_STATES) = -1.0*_z_Q*_z_referenceTraj.block(i*NUM_OF_Z_STATES,0,NUM_OF_Z_STATES,1);
   }

   if(_debug)
   {
      std::cout<<"Z QP gradient vector q = "<<std::endl<<_z_gradient<<std::endl;
   }

   return;
}

void 
MPC12STATE::castYawMPCToQPGradient(void)
{
   _yaw_gradient.setZero();

   // Populate the gradient vector
   for(int i=0; i<_mpcWindow+1; i++)
   {
      _yaw_gradient.segment(i*NUM_OF_Z_STATES,NUM_OF_Z_STATES) = -1.0*_yaw_Q*_yaw_referenceTraj.block(i*NUM_OF_YAW_STATES,0,NUM_OF_YAW_STATES,1);
   }

   if(_debug)
   {
      std::cout<<"Yaw QP gradient vector q = "<<std::endl<<_yaw_gradient<<std::endl;
   }

   return;
}


void 
MPC12STATE::updateXYQPGradientVector(void)
{
   for(int i=0; i<_mpcWindow+1; i++)
   {
      _xy_gradient.segment(i*NUM_OF_XY_STATES,NUM_OF_XY_STATES) = -1.0*_xy_Q*_xy_referenceTraj.block(i*NUM_OF_XY_STATES,0,NUM_OF_XY_STATES,1);
   }
   if (_debug)
   {
      std::cout << "[MPC12STATE::updateXYQPGradientVector] Updated XY QP gradient = \n" << _xy_gradient << "\n";
   }

   return;
}

void 
MPC12STATE::updateZQPGradientVector(void)
{
   for(int i=0; i<_mpcWindow+1; i++)
   {
      _z_gradient.segment(i*NUM_OF_Z_STATES,NUM_OF_Z_STATES) = -1.0*_z_Q*_z_referenceTraj.block(i*NUM_OF_Z_STATES,0,NUM_OF_Z_STATES,1);
   }
   if (_debug)
   {
      std::cout << "[MPC12STATE::updateZQPGradientVector] Updated Z QP gradient = \n" << _z_gradient << "\n";
   }

   return;
}

bool 
MPC12STATE::computeYawRefTrajectory(void)
{
   // @NOTE Requires solution  _xy_x_opt

   if(_xy_referenceTraj.size() < 1)
   {
      printError("[computeYawRefTrajectory] _xy_referenceTraj is empty!\n");
      return false;
   }

   if(_xy_x_opt.size() < 1)
   {
      printError("[computeYawRefTrajectory] _xy_x_opt is empty!\n");
      return false;
   }
   
   _yaw_referenceTraj.setZero();
   for (int i=0; i<(_mpcWindow+1); i++)
   {
      auto x_target = _xy_referenceTraj(NUM_OF_XY_STATES*i+XY_X_IDX,0);
      auto x_interceptor = _xy_x_opt(NUM_OF_XY_STATES*i+ XY_X_IDX);
      auto y_target = _xy_referenceTraj(NUM_OF_XY_STATES*i+XY_Y_IDX,0);
      auto y_interceptor = _xy_x_opt(NUM_OF_XY_STATES*i+ XY_Y_IDX);
      _yaw_referenceTraj(NUM_OF_YAW_STATES*i + YAW_Yaw_IDX,0) = std::atan2(y_target - y_interceptor, x_target - x_interceptor);
   }

   return true;
}

void 
MPC12STATE::updateYawQPGradientVector(void)
{

   for(int i=0; i<_mpcWindow+1; i++)
   {
      _yaw_gradient.segment(i*NUM_OF_YAW_STATES,NUM_OF_YAW_STATES) = -1.0*_yaw_Q*_yaw_referenceTraj.block(i*NUM_OF_YAW_STATES,0,NUM_OF_YAW_STATES,1);
   }
   if (_debug)
   {
      std::cout << "[MPC12STATE::updateYawQPGradientVector] Updated Yaw QP gradient = \n" << _yaw_gradient << "\n";
   }

   return;
}

void 
MPC12STATE::castXYMPCToQPConstraintMatrix(void)
{
   _xy_Ac.setZero();
   //_Ac = Eigen::MatrixXd::Zero(size_r, size_c);

   // length of states/inputs over _mpcWindow
   auto N_x = NUM_OF_XY_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_XY_INPUTS * _mpcWindow;

   // Initial condition constraint
   _xy_Ac.block(0, 0, N_x, N_x) = -1.0 * Eigen::MatrixXd::Identity(N_x,N_x);
   // Dynamics constraints
   for (int i=1; i<_mpcWindow+1; i++)
   {
      // upper-left block. Dynamics transition matrix
      _xy_Ac.block(i*NUM_OF_XY_STATES, (i-1)*NUM_OF_XY_STATES, NUM_OF_XY_STATES, NUM_OF_XY_STATES) = _xy_A;
      // upper-right block Dynamics input matrix
      _xy_Ac.block(i*NUM_OF_XY_STATES, N_x+(i-1)*NUM_OF_XY_INPUTS, NUM_OF_XY_STATES, NUM_OF_XY_INPUTS) = _xy_B;
      
      // input constraints: lower-right block
      //_Ac.block(2*N_x+(i-1)*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_INPUTS);
   }

   // Individual state bounds:  middle-left block = Identity
   _xy_Ac.block(N_x, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x,N_x);

   // Inputs bounds: lower-right block = Identity
   _xy_Ac.block(2*N_x, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u,N_u);

   // Mixed vel/accel constraints
   auto N_MIX = NUM_OF_XY_MIXED_ACCEL_CONST + NUM_OF_XY_MIXED_VEL_CONST;
   for (int i=0; i < _mpcWindow; i++)
   {
      // Mixed-velocity contraints (2nd order approximation)
      //sqrt(3)/2 vx + 0.5 vy
      _xy_Ac(2*N_x+N_u + N_MIX*i +0, NUM_OF_XY_STATES *(i+1) + 1) = std::sqrt(3)/2;
      _xy_Ac(2*N_x+N_u + N_MIX*i +0, NUM_OF_XY_STATES *(i+1) + 4) = 0.5;

      // sqrt(3)/2 vx - 0.5 vy
      _xy_Ac(2*N_x+N_u + N_MIX*i +1, NUM_OF_XY_STATES *(i+1) + 1) = std::sqrt(3)/2;
      _xy_Ac(2*N_x+N_u + N_MIX*i +1, NUM_OF_XY_STATES *(i+1) + 4) = -0.5;

      //0.5 vx + sqrt(3)/2 vy
      _xy_Ac(2*N_x+N_u + N_MIX*i +2, NUM_OF_XY_STATES *(i+1) + 1) = 0.5;
      _xy_Ac(2*N_x+N_u + N_MIX*i +2, NUM_OF_XY_STATES *(i+1) + 4) = std::sqrt(3)/2;

      //-0.5 vx + sqrt(3)/2 vy
      _xy_Ac(2*N_x+N_u + N_MIX*i +3, NUM_OF_XY_STATES *(i+1) + 1) = -0.5;
      _xy_Ac(2*N_x+N_u + N_MIX*i +3, NUM_OF_XY_STATES *(i+1) + 4) = std::sqrt(3)/2;

      // Mixed-acceleration contraints (1st order approximation)
      // sqrt(2)/2 ax + sqrt(2)/2 ay
      _xy_Ac(2*N_x+N_u + N_MIX*i +4, NUM_OF_XY_STATES *(i+1) + 2) = std::sqrt(2)/2;
      _xy_Ac(2*N_x+N_u + N_MIX*i +4, NUM_OF_XY_STATES *(i+1) + 5) = std::sqrt(2)/2;

      // sqrt(2)/2 ax - sqrt(2)/2 ay
      _xy_Ac(2*N_x+N_u + N_MIX*i +5, NUM_OF_XY_STATES *(i+1) + 2) = std::sqrt(2)/2;
      _xy_Ac(2*N_x+N_u + N_MIX*i +5, NUM_OF_XY_STATES *(i+1) + 5) = -std::sqrt(2)/2;

   }

   if(_debug)
   {
      std::cout<<"Constraint matrix _xy_Ac = " <<std::endl<< _xy_Ac <<std::endl;
   }

   return;
}

void 
MPC12STATE::castZMPCToQPConstraintMatrix(void)
{
   // Initialize Ac
   int size_r = 2*NUM_OF_Z_STATES * (_mpcWindow+1) + NUM_OF_Z_INPUTS * _mpcWindow;
   int size_c = NUM_OF_Z_STATES * (_mpcWindow+1) + NUM_OF_Z_INPUTS * _mpcWindow;
   _z_Ac.resize(size_r, size_c);
   _z_Ac.setZero();
   //_Ac = Eigen::MatrixXd::Zero(size_r, size_c);

   // length of states/inputs over _mpcWindow
   auto N_x = NUM_OF_Z_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_Z_INPUTS * _mpcWindow;

   // Initial condition constraint
   _z_Ac.block(0, 0, N_x, N_x) = -1.0 * Eigen::MatrixXd::Identity(N_x,N_x);
   // Dynamics constraints
   for (int i=1; i<_mpcWindow+1; i++)
   {
      // upper-left block. Dynamics transition matrix
      _z_Ac.block(i*NUM_OF_Z_STATES, (i-1)*NUM_OF_Z_STATES, NUM_OF_Z_STATES, NUM_OF_Z_STATES) = _z_A;
      // upper-right block Dynamics input matrix
      _z_Ac.block(i*NUM_OF_Z_STATES, N_x+(i-1)*NUM_OF_Z_INPUTS, NUM_OF_Z_STATES, NUM_OF_Z_INPUTS) = _z_B;
      
      // input constraints: lower-right block
      //_Ac.block(2*N_x+(i-1)*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_INPUTS);
   }

   // Individual state bounds:  middle-left block = Identity
   _z_Ac.block(N_x, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x,N_x);

   // Inputs bounds: lower-right block = Identity
   _z_Ac.block(2*N_x, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u,N_u);

   if(_debug)
   {
      std::cout<<"Constraint matrix _z_Ac = " <<std::endl<< _z_Ac <<std::endl;
   }

   return;
}

void 
MPC12STATE::castYawMPCToQPConstraintMatrix(void)
{
   // Initialize Ac
   int size_r = 2*NUM_OF_YAW_STATES * (_mpcWindow+1) + NUM_OF_YAW_INPUTS * _mpcWindow;
   int size_c = NUM_OF_YAW_STATES * (_mpcWindow+1) + NUM_OF_YAW_INPUTS * _mpcWindow;
   _yaw_Ac.resize(size_r, size_c);
   _yaw_Ac.setZero();
   //_Ac = Eigen::MatrixXd::Zero(size_r, size_c);

   // length of states/inputs over _mpcWindow
   auto N_x = NUM_OF_YAW_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_YAW_INPUTS * _mpcWindow;

   // Initial condition constraint
   _yaw_Ac.block(0, 0, N_x, N_x) = -1.0 * Eigen::MatrixXd::Identity(N_x,N_x);
   // Dynamics constraints
   for (int i=1; i<_mpcWindow+1; i++)
   {
      // upper-left block. Dynamics transition matrix
      _yaw_Ac.block(i*NUM_OF_YAW_STATES, (i-1)*NUM_OF_YAW_STATES, NUM_OF_YAW_STATES, NUM_OF_YAW_STATES) = _yaw_A;
      // upper-right block Dynamics input matrix
      _yaw_Ac.block(i*NUM_OF_YAW_STATES, N_x+(i-1)*NUM_OF_YAW_INPUTS, NUM_OF_YAW_STATES, NUM_OF_YAW_INPUTS) = _yaw_B;
      
      // input constraints: lower-right block
      //_Ac.block(2*N_x+(i-1)*NUM_OF_STATES, N_x+(i-1)*NUM_OF_INPUTS, NUM_OF_STATES, NUM_OF_INPUTS) = Eigen::MatrixXd::Identity(NUM_OF_STATES,NUM_OF_INPUTS);
   }

   // Individual state bounds:  middle-left block = Identity
   _yaw_Ac.block(N_x, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x,N_x);

   // Inputs bounds: lower-right block = Identity
   _yaw_Ac.block(2*N_x, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u,N_u);

   if(_debug)
   {
      std::cout<<"Constraint matrix _yaw_Ac = " <<std::endl<< _yaw_Ac <<std::endl;
   }

   return;
}

bool MPC12STATE::computeXYBounds(void)
{
   // sanity check
   if(_z_x_opt.size() < 1)
   {
      printError("[MPC12STATE::computeXYVelMaxFromZAccelMax] _z_x_opt is empty. Cannot compute _xy_VelMin and _xy_VelMax");
      return false;
   }

   // The resize should be done once in the initialization
   //_xy_Min.resize(NUM_OF_XY_STATES*_mpcWindow); _xy_Max.resize(NUM_OF_XY_STATES*_mpcWindow);
   _xy_Min.setZero(); _xy_Max.setZero();
   _xy_MixedState_Min.setZero(); _xy_MixedState_Max.setZero();

   auto s  = NUM_OF_XY_MIXED_VEL_CONST + NUM_OF_XY_MIXED_ACCEL_CONST;

   // state: [x, vx, ax, y, vy, ay]
   for (int i=1; i<(_mpcWindow+1); i++)
   {
      _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_X_IDX, 0) = OsqpEigen::INFTY; // upper bound on x
      _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_Y_IDX, 0) = OsqpEigen::INFTY; // upper bound on y
      _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_AX_IDX, 0) = _xy_MaxAccel; // upper bound on ax
      _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_AY_IDX, 0) = _xy_MaxAccel; // upper bound on ay

      
      // z state: [z, vz, az]
      auto v_zt = _z_x_opt(NUM_OF_Z_STATES*i+ Z_VZ_IDX, 0); // v_z(t)
      if( v_zt < 0) // descending, v_z(t) < 0  keep velocity at max
      {
         _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_VX_IDX, 0) = _xy_MaxVel; // upper bound on vx
         _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_VY_IDX, 0) = _xy_MaxVel; // upper bound on vy

         // Mixed state velocity bounds
         _xy_MixedState_Max.block(s*(i-1),0,NUM_OF_XY_MIXED_VEL_CONST,1) = _xy_MaxVel*Eigen::MatrixXd::Ones(NUM_OF_XY_MIXED_VEL_CONST,1);
      }
      else // ascending, v_z(t) > 0
      {
         double a_zt = _z_x_opt(NUM_OF_Z_STATES*i+ Z_AZ_IDX, 0);
         double d = a_zt /_z_MaxAccel; // Make sure the  _z_MaxVel > 0!!
         double v_hmax_t = _xy_MaxVel*std::sqrt(1 - d*d);

         _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_VX_IDX, 0) = v_hmax_t; // upper bound on vx
         _xy_Max(NUM_OF_XY_STATES*(i-1) + XY_VY_IDX, 0) = v_hmax_t; // upper bound on vy

         // Mixed state velocity bounds
         _xy_MixedState_Max.block(s*(i-1),0,NUM_OF_XY_MIXED_VEL_CONST,1) = v_hmax_t*Eigen::MatrixXd::Ones(NUM_OF_XY_MIXED_VEL_CONST,1);
      }

      // Mixed state acceleration bounds
      _xy_MixedState_Max.block(s*(i-1)+NUM_OF_XY_MIXED_VEL_CONST,0, NUM_OF_XY_MIXED_ACCEL_CONST,1) = _xy_MaxAccel*Eigen::MatrixXd::Ones(NUM_OF_XY_MIXED_ACCEL_CONST,1);

   }

   _xy_Min = -1.0*_xy_Max;
   _xy_MixedState_Min = -1.0* _xy_MixedState_Max;

   if (_debug)
      printInfo("[MPC12STATE::computeXYBounds] DONE computing XY bounds");

   return true;
}

void 
MPC12STATE::castXYMPCToQPConstraintBounds(void)
{
   Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(NUM_OF_XY_STATES*(_mpcWindow+1),1 );
   Eigen::VectorXd upperEquality;
   lowerEquality.block(0,0,NUM_OF_XY_STATES,1) = -_xy_current_state;
   upperEquality = lowerEquality;
   if(_debug)
      printInfo("XY - Calculated lowerEquality and upperEquality");

   // Controls (jerk) bounds
   Eigen::VectorXd uLowerInequality = Eigen::MatrixXd::Zero(NUM_OF_XY_INPUTS*_mpcWindow,1 );
   Eigen::VectorXd uUpperInequality;
   uUpperInequality = uLowerInequality;

   
   for(int i=0; i<_mpcWindow; i++){
      uLowerInequality.block(NUM_OF_XY_INPUTS*i, 0, NUM_OF_XY_INPUTS, 1) = _xy_uMin;
      uUpperInequality.block(NUM_OF_XY_INPUTS*i, 0, NUM_OF_XY_INPUTS, 1) = _xy_uMax;
   }
   if(_debug)
      printInfo("XY - Calculated uLowerInequality and uUpperInequality");

   _xy_upperBounds.setZero();
   _xy_lowerBounds.setZero();
   _xy_lowerBounds << lowerEquality,
                        -OsqpEigen::INFTY,-_xy_MaxVel, -_xy_MaxAccel, -OsqpEigen::INFTY, -_xy_MaxVel, -_xy_MaxAccel,
                        _xy_Min,
                        uLowerInequality,
                        _xy_MixedState_Min;
   _xy_upperBounds << upperEquality,
                        OsqpEigen::INFTY,_xy_MaxVel, _xy_MaxAccel, OsqpEigen::INFTY, _xy_MaxVel, _xy_MaxAccel,
                        _xy_Max,
                        uUpperInequality,
                        _xy_MixedState_Max;
      
   if(_debug)
   {
      std::cout<<"XY - Lower bounds _xy_lowerBounds = "<<std::endl<< _xy_lowerBounds <<std::endl;
      std::cout<<"XY - Upper bounds _xy_upperBounds = "<<std::endl<< _xy_upperBounds <<std::endl;
   }

   return;
}

void 
MPC12STATE::castZMPCToQPConstraintBounds(void)
{
   // length of states/inputs over _mpcWindow
   // _mpcWindow+1, because z(0) is included
   auto N_x = NUM_OF_Z_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_Z_INPUTS * _mpcWindow;

   // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(NUM_OF_Z_STATES*(_mpcWindow+1) +  NUM_OF_Z_INPUTS * _mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(NUM_OF_Z_STATES*(_mpcWindow+1) +  NUM_OF_Z_INPUTS * _mpcWindow, 1);
    for(int i=0; i<_mpcWindow+1; i++){
        lowerInequality.block(NUM_OF_Z_STATES*i,0,NUM_OF_Z_STATES,1) = _z_Min;
        upperInequality.block(NUM_OF_Z_STATES*i,0,NUM_OF_Z_STATES,1) = _z_Max;
    }
    for(int i=0; i<_mpcWindow; i++){
        lowerInequality.block(NUM_OF_Z_INPUTS * i + NUM_OF_Z_STATES * (_mpcWindow + 1), 0, NUM_OF_Z_INPUTS, 1) = _z_uMin*Eigen::VectorXd::Ones(NUM_OF_Z_INPUTS);
        upperInequality.block(NUM_OF_Z_INPUTS * i + NUM_OF_Z_STATES * (_mpcWindow + 1), 0, NUM_OF_Z_INPUTS, 1) = _z_uMax*Eigen::VectorXd::Ones(NUM_OF_Z_INPUTS);
    }
    if(_debug)
      printInfo("Z - Calculated lowerInequality and upperInequality");

    // evaluate the lower and the upper equality vectors      
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(NUM_OF_Z_STATES*(_mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,NUM_OF_Z_STATES,1) = -_z_current_state;
    upperEquality = lowerEquality;

    if(_debug)
      printInfo("Z - Calculated lowerEquality and upperEquality");

   _z_lowerBounds.resize(2*N_x + N_u,1);
   _z_lowerBounds.setZero();
   _z_upperBounds.resize(2*N_x + N_u,1);
   _z_upperBounds.setZero();

   _z_lowerBounds << lowerEquality,
        lowerInequality;
   _z_upperBounds << upperEquality,
        upperInequality;

   if(_debug)
   {
      std::cout<<"Z - Lower bounds _z_lowerBounds = "<<std::endl<< _z_lowerBounds <<std::endl;
      std::cout<<"Z - Upper bounds _z_upperBounds = "<<std::endl<< _z_upperBounds <<std::endl;
   }

   return;
}

void 
MPC12STATE::castYawMPCToQPConstraintBounds(void)
{
   // length of states/inputs over _mpcWindow
   // _mpcWindow+1, because yaw(0) is included
   auto N_x = NUM_OF_YAW_STATES*(_mpcWindow+1);
   auto N_u = NUM_OF_YAW_INPUTS * _mpcWindow;

   // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(NUM_OF_YAW_STATES*(_mpcWindow+1) +  NUM_OF_YAW_INPUTS * _mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(NUM_OF_YAW_STATES*(_mpcWindow+1) +  NUM_OF_YAW_INPUTS * _mpcWindow, 1);
    for(int i=0; i<_mpcWindow+1; i++){
        lowerInequality.block(NUM_OF_YAW_STATES*i,0,NUM_OF_YAW_STATES,1) = _yaw_Min;
        upperInequality.block(NUM_OF_YAW_STATES*i,0,NUM_OF_YAW_STATES,1) = _yaw_Max;
    }
    for(int i=0; i<_mpcWindow; i++){
        lowerInequality.block(NUM_OF_YAW_INPUTS * i + NUM_OF_YAW_STATES * (_mpcWindow + 1), 0, NUM_OF_YAW_INPUTS, 1) = _yaw_uMin*Eigen::VectorXd::Ones(NUM_OF_YAW_INPUTS);
        upperInequality.block(NUM_OF_YAW_INPUTS * i + NUM_OF_YAW_STATES * (_mpcWindow + 1), 0, NUM_OF_YAW_INPUTS, 1) = _yaw_uMax*Eigen::VectorXd::Ones(NUM_OF_YAW_INPUTS);
    }
    if(_debug)
      printInfo("Yaw - Calculated lowerInequality and upperInequality");

    // evaluate the lower and the upper equality vectors      
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(NUM_OF_YAW_STATES*(_mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,NUM_OF_YAW_STATES,1) = -_yaw_current_state;
    upperEquality = lowerEquality;

    if(_debug)
      printInfo("Yaw - Calculated lowerEquality and upperEquality");

   _yaw_lowerBounds.resize(2*N_x + N_u,1);
   _yaw_lowerBounds.setZero();
   _yaw_upperBounds.resize(2*N_x + N_u,1);
   _yaw_upperBounds.setZero();

   _yaw_lowerBounds << lowerEquality,
        lowerInequality;
   _yaw_upperBounds << upperEquality,
        upperInequality;

   if(_debug)
   {
      std::cout<<"Yaw - Lower bounds _yaw_lowerBounds = "<<std::endl<< _yaw_lowerBounds <<std::endl;
      std::cout<<"Yaw - Upper bounds _yaw_upperBounds = "<<std::endl<< _yaw_upperBounds <<std::endl;
   }

   return;
}

void 
MPC12STATE::updateXYQPConstraintsBounds(void)
{
   // @NOTE Needs computeXYVelMaxFromZAccelMax() to be executd first -> need to solve for _z_x_opt traj first

   // _xy_lowerBounds << lowerEquality,
   //                      _xy_current_state,
   //                      _xy_Min,
   //                      uLowerInequality,
   //                      _xy_MixedState_Min;
   // _xy_upperBounds << upperEquality,
   //                      _xy_current_state,
   //                      _xy_Max,
   //                      uUpperInequality,
   //                      _xy_MixedState_Max;
   
   // Equality for x(0)
   _xy_lowerBounds.block(0,0,NUM_OF_XY_STATES,1) = -1.0*_xy_current_state;
   _xy_upperBounds.block(0,0,NUM_OF_XY_STATES,1) = -1.0*_xy_current_state;

   // Inequality for x(0)
   Eigen::VectorXd x0; x0.resize(6); x0.setZero();
   x0 <<  OsqpEigen::INFTY, _xy_MaxVel, _xy_MaxAccel,  OsqpEigen::INFTY, _xy_MaxVel, _xy_MaxAccel;
   _xy_lowerBounds.block(NUM_OF_XY_STATES*(_mpcWindow+1),0,NUM_OF_XY_STATES,1) = -x0;
   _xy_upperBounds.block(NUM_OF_XY_STATES*(_mpcWindow+1),0,NUM_OF_XY_STATES,1) = x0;
   
   // _xy_Min, _xy_Max
   _xy_lowerBounds.block(NUM_OF_XY_STATES*(_mpcWindow+1)+NUM_OF_XY_STATES,0,NUM_OF_XY_STATES*_mpcWindow,1) = _xy_Min;
   _xy_upperBounds.block(NUM_OF_XY_STATES*(_mpcWindow+1)+NUM_OF_XY_STATES,0,NUM_OF_XY_STATES*_mpcWindow,1) = _xy_Max;

   // _xy_MixedState_Min, _xy_MixedState_Max
   _xy_lowerBounds.block(2*NUM_OF_XY_STATES*(_mpcWindow+1)+NUM_OF_XY_INPUTS*_mpcWindow,0,(NUM_OF_XY_MIXED_ACCEL_CONST+NUM_OF_XY_MIXED_VEL_CONST)*_mpcWindow,1) = _xy_MixedState_Min;
   _xy_upperBounds.block(2*NUM_OF_XY_STATES*(_mpcWindow+1)+NUM_OF_XY_INPUTS*_mpcWindow,0,(NUM_OF_XY_MIXED_ACCEL_CONST+NUM_OF_XY_MIXED_VEL_CONST)*_mpcWindow,1) = _xy_MixedState_Max;

   if(_debug)
   {
      printInfo("[MPC12STATE::updateXYQPConstraintsBounds] XY - QP bounds are updated");
      std::cout << "[MPC12STATE::updateXYQPConstraintsBounds] XY - Updated lower bound,l = \n " << _xy_lowerBounds << "\n";
      std::cout << "[MPC12STATE::updateXYQPConstraintsBounds] XY - Updated upper bound,u = \n " << _xy_upperBounds << "\n";
   }

   return;
}

void 
MPC12STATE::updateZQPConstraintsBounds(void)
{
   // Equality for x(0)
   _z_lowerBounds.block(0,0,NUM_OF_Z_STATES,1) = -1.0*_z_current_state;
   _z_upperBounds.block(0,0,NUM_OF_Z_STATES,1) = -1.0*_z_current_state;

   if(_debug)
   {
      printInfo("[MPC12STATE::updateZQPConstraintsBounds] Z- QP bounds are updated");
      std::cout << "[MPC12STATE::updateZQPConstraintsBounds] Updated _z_lowerBounds = \n " << _z_lowerBounds << "\n";
      std::cout << "[MPC12STATE::updateZQPConstraintsBounds] Updated _z_upperBounds = \n " << _z_upperBounds << "\n";
   }

   return;
}

void 
MPC12STATE::updateYawQPConstraintsBounds(void)
{
   // Equality for x(0)
   _yaw_lowerBounds.block(0,0,NUM_OF_YAW_STATES,1) = -1.0*_yaw_current_state;
   _yaw_upperBounds.block(0,0,NUM_OF_YAW_STATES,1) = -1.0*_yaw_current_state;

   if(_debug)
   {
      printInfo("[MPC12STATE::updateYawQPConstraintsBounds] Yaw- QP bounds are updated");
      std::cout << "[MPC12STATE::updateYawQPConstraintsBounds] Updated _yaw_lowerBounds = \n " << _yaw_lowerBounds << "\n";
      std::cout << "[MPC12STATE::updateYawQPConstraintsBounds] Updated _yaw_upperBounds = \n " << _yaw_upperBounds << "\n";
   }

   return;
}

bool 
MPC12STATE::initQPSolver(void)
{
   // @WARNING You must initialize all variables (with unkown size) before executing this function!

   // XY QP solver
   if(_debug)
      printInfo("Initializing XY QP solver ...");
   _xy_qpSolver.settings()->setVerbosity(_debug);
   _xy_qpSolver.settings()->setWarmStart(true);
   _xy_qpSolver.data()->setNumberOfVariables(NUM_OF_XY_STATES*(_mpcWindow+1) + NUM_OF_XY_INPUTS*_mpcWindow);
   _xy_qpSolver.data()->setNumberOfConstraints(2*NUM_OF_XY_STATES*(_mpcWindow+1) + NUM_OF_XY_INPUTS*_mpcWindow + (NUM_OF_XY_MIXED_ACCEL_CONST+NUM_OF_XY_MIXED_VEL_CONST)*_mpcWindow);
   _xy_hessian_sparse = _xy_hessian.sparseView();
   _xy_Ac_sparse = _xy_Ac.sparseView();
   if(!_xy_qpSolver.data()->setHessianMatrix(_xy_hessian_sparse)) return false;
   if(!_xy_qpSolver.data()->setGradient(_xy_gradient)) return false;
   if(!_xy_qpSolver.data()->setLinearConstraintsMatrix(_xy_Ac_sparse)) return false;
   if(!_xy_qpSolver.data()->setLowerBound(_xy_lowerBounds)) return false;
   if(!_xy_qpSolver.data()->setUpperBound(_xy_upperBounds)) return false;
   if(!_xy_qpSolver.initSolver()) return false;
   if(_debug)
   {
      printInfo("XY - QP solver is initialized.");
   }

   // Z QP solver
   if(_debug)
      printInfo("Initializing Z QP solver ...");
   _z_qpSolver.settings()->setVerbosity(_debug);
   _z_qpSolver.settings()->setWarmStart(true);
   _z_qpSolver.data()->setNumberOfVariables(NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow);
   _z_qpSolver.data()->setNumberOfConstraints(2*NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow);
   _z_hessian_sparse = _z_hessian.sparseView();
   _z_Ac_sparse = _z_Ac.sparseView();
   if(!_z_qpSolver.data()->setHessianMatrix(_z_hessian_sparse)) return false;
   if(!_z_qpSolver.data()->setGradient(_z_gradient)) return false;
   if(!_z_qpSolver.data()->setLinearConstraintsMatrix(_z_Ac_sparse)) return false;
   if(!_z_qpSolver.data()->setLowerBound(_z_lowerBounds)) return false;
   if(!_z_qpSolver.data()->setUpperBound(_z_upperBounds)) return false;
   if(!_z_qpSolver.initSolver()) return false;
   if(_debug)
   {
      printInfo("Z - QP solver is initialized.");
   }

   // Yaw QP solver
   if(_debug)
      printInfo("Initializing Yaw QP solver ...");
   _yaw_qpSolver.settings()->setVerbosity(_debug);
   _yaw_qpSolver.settings()->setWarmStart(true);
   _yaw_qpSolver.data()->setNumberOfVariables(NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow);
   _yaw_qpSolver.data()->setNumberOfConstraints(2*NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow);
   _yaw_hessian_sparse = _yaw_hessian.sparseView();
   _yaw_Ac_sparse = _yaw_Ac.sparseView();
   if(!_yaw_qpSolver.data()->setHessianMatrix(_yaw_hessian_sparse)) return false;
   if(!_yaw_qpSolver.data()->setGradient(_yaw_gradient)) return false;
   if(!_yaw_qpSolver.data()->setLinearConstraintsMatrix(_yaw_Ac_sparse)) return false;
   if(!_yaw_qpSolver.data()->setLowerBound(_yaw_lowerBounds)) return false;
   if(!_yaw_qpSolver.data()->setUpperBound(_yaw_upperBounds)) return false;
   if(!_yaw_qpSolver.initSolver()) return false;
   if(_debug)
   {
      printInfo("Yaw - QP solver is initialized.");
   }

   return true;
}

void MPC12STATE::initVariables(void)
{
   if(_debug)
      printInfo("[MPC12STATE::initVariables] Initializing MPC variables");

   _referenceTraj.resize(NUM_OF_STATES*(_mpcWindow+1),1); _referenceTraj.setZero();
   _x_opt.resize(NUM_OF_STATES*(_mpcWindow+1)); _x_opt.setZero();
   _u_opt.resize(NUM_OF_INPUTS*_mpcWindow); _u_opt.setZero();

   if(_debug)
      printInfo("[MPC12STATE::initVariables] DONE Initializing _referenceTraj, _x_opt, _u_opt");
   
   // xy
   int size;
   size = NUM_OF_XY_STATES*(_mpcWindow+1) + NUM_OF_XY_INPUTS*_mpcWindow;
   _xy_gradient.resize(size); _xy_gradient.setZero();

   size = NUM_OF_XY_STATES*(_mpcWindow+1) + NUM_OF_XY_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _xy_hessian.resize(size, size); _xy_hessian.setZero();

   int size_r = 2*NUM_OF_XY_STATES * (_mpcWindow+1) +
               NUM_OF_XY_INPUTS * _mpcWindow +
               (NUM_OF_XY_MIXED_VEL_CONST + NUM_OF_XY_MIXED_ACCEL_CONST) * _mpcWindow;
   int size_c = NUM_OF_XY_STATES * (_mpcWindow+1) + NUM_OF_XY_INPUTS * _mpcWindow;
   _xy_Ac.resize(size_r, size_c); _xy_Ac.setZero();

   _xy_Min.resize(NUM_OF_XY_STATES*_mpcWindow); _xy_Max.resize(NUM_OF_XY_STATES*_mpcWindow);
   _xy_Min.setZero(); _xy_Max.setZero();

   _xy_MixedState_Min.resize((NUM_OF_XY_MIXED_VEL_CONST+NUM_OF_XY_MIXED_ACCEL_CONST)*_mpcWindow);
   _xy_MixedState_Max.resize((NUM_OF_XY_MIXED_VEL_CONST+NUM_OF_XY_MIXED_ACCEL_CONST)*_mpcWindow);
   _xy_MixedState_Min.setZero(); _xy_MixedState_Max.setZero();

   size = 2*NUM_OF_XY_STATES*(_mpcWindow+1) + 
               NUM_OF_XY_INPUTS*_mpcWindow +
               (NUM_OF_XY_MIXED_ACCEL_CONST+NUM_OF_XY_MIXED_VEL_CONST)*_mpcWindow;
   _xy_lowerBounds.resize(size); _xy_lowerBounds.setZero();
   _xy_upperBounds.resize(size); _xy_upperBounds.setZero();

   _xy_x_opt.resize(NUM_OF_XY_STATES*(_mpcWindow+1)); _xy_x_opt.setZero();
   _xy_u_opt.resize(NUM_OF_XY_INPUTS*_mpcWindow); _xy_u_opt.setZero();

   _xy_referenceTraj.resize(NUM_OF_XY_STATES*(_mpcWindow+1), 1);
   _xy_referenceTraj.setZero();

   if(_debug)
      printInfo("[MPC12STATE::initVariables] DONE Initializing XY MPC variables");

   // z
   size = NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow;
   _z_gradient.resize(size); _z_gradient.setZero();

   size = NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _z_hessian.resize(size, size); _z_hessian.setZero();

   size_r = 2*NUM_OF_Z_STATES * (_mpcWindow+1) + NUM_OF_Z_INPUTS * _mpcWindow;
   size_c = NUM_OF_Z_STATES * (_mpcWindow+1) + NUM_OF_Z_INPUTS * _mpcWindow;
   _z_Ac.resize(size_r, size_c); _z_Ac.setZero();

   _z_Min.setZero(); _z_Max.setZero();

   size = 2*NUM_OF_Z_STATES*(_mpcWindow+1) + NUM_OF_Z_INPUTS*_mpcWindow;
   _z_lowerBounds.resize(size); _z_lowerBounds.setZero();
   _z_upperBounds.resize(size); _z_upperBounds.setZero();

   _z_x_opt.resize(NUM_OF_Z_STATES*(_mpcWindow+1)); _z_x_opt.setZero();
   _z_u_opt.resize(NUM_OF_Z_INPUTS*_mpcWindow); _z_u_opt.setZero();

   _z_referenceTraj.resize(NUM_OF_Z_STATES*(_mpcWindow+1), 1);
   _z_referenceTraj.setZero();

   if(_debug)
      printInfo("[MPC12STATE::initVariables] DONE Initializing Z MPC variables");

   // yaw
   size = NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow;
   _yaw_gradient.resize(size); _yaw_gradient.setZero();

   size = NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow; // Length of optimization vector over MPC horizon (_mpcWindow)
   _yaw_hessian.resize(size, size); _yaw_hessian.setZero();

   size_r = 2*NUM_OF_YAW_STATES * (_mpcWindow+1) + NUM_OF_YAW_INPUTS * _mpcWindow;
   size_c = NUM_OF_YAW_STATES * (_mpcWindow+1) + NUM_OF_YAW_INPUTS * _mpcWindow;
   _yaw_Ac.resize(size_r, size_c); _yaw_Ac.setZero();

   _yaw_Min.setZero(); _yaw_Max.setZero();

   size = 2*NUM_OF_YAW_STATES*(_mpcWindow+1) + NUM_OF_YAW_INPUTS*_mpcWindow;
   _yaw_lowerBounds.resize(size); _yaw_lowerBounds.setZero();
   _yaw_upperBounds.resize(size); _yaw_upperBounds.setZero();

   _yaw_x_opt.resize(NUM_OF_YAW_STATES*(_mpcWindow+1)); _yaw_x_opt.setZero();
   _yaw_u_opt.resize(NUM_OF_YAW_INPUTS*_mpcWindow); _yaw_u_opt.setZero();

   _yaw_referenceTraj.resize(NUM_OF_YAW_STATES*(_mpcWindow+1), 1);
   _yaw_referenceTraj.setZero();

   if(_debug)
      printInfo("[MPC12STATE::initVariables] DONE Initializing Yaw MPC variables");

   /// flags
   _received_refTraj = false;
   _state_received = false;

   if(_debug)
      printInfo("[MPC12STATE::initVariables] DONE Initializing MPC variables");
}

bool 
MPC12STATE::updateXYQP(void)
{
   if(!computeXYBounds())
   {
      printError("[updateXYQP] computeXYBounds() returend false");
      return false; // @NOTE this requires solving the Z problem to compute _z_x_opt
   }
   // update current drone's position (updates QP linear constraints bounds)
   updateXYQPConstraintsBounds();
   if(!_xy_qpSolver.updateBounds(_xy_lowerBounds, _xy_upperBounds))
   {
      printError("_xy_qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      printInfo("[MPCTracker::updateXYQP] Bounds are updated.");

   
   // Update the QP gradient vector using  new _xy_referenceTraj
   updateXYQPGradientVector();
   if(!_xy_qpSolver.updateGradient(_xy_gradient))
   {
      printError("_xy_qpSolver.updateGradient failed to update _gradient");
      return false;
   }
   if(_debug)
      printInfo("XY QP gradient is updated");

   return true;
}

bool 
MPC12STATE::updateZQP(void)
{
   // update current drone's position (updates QP linear constraints bounds)
   updateZQPConstraintsBounds();
   if(!_z_qpSolver.updateBounds(_z_lowerBounds, _z_upperBounds))
   {
      printError("_z_qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      printInfo("[MPCTracker::updateZQP] Bounds are updated.");

   
   // Update the QP gradient vector using  new _z_referenceTraj
   updateZQPGradientVector();
   if(!_z_qpSolver.updateGradient(_z_gradient))
   {
      printError("_z_qpSolver.updateGradient failed to update _gradient");
      return false;
   }
   if(_debug)
      printInfo("Z QP gradient is updated");

   return true;
}

bool 
MPC12STATE::updateYawQP(void)
{
   // update current drone's position (updates QP linear constraints bounds)
   updateYawQPConstraintsBounds();
   if(!_yaw_qpSolver.updateBounds(_yaw_lowerBounds, _yaw_upperBounds))
   {
      printError("_yaw_qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      printInfo("[MPCTracker::updateYawQP] Bounds are updated.");

   if(!computeYawRefTrajectory()) return false;
   // Update the QP gradient vector using  new _yaw_referenceTraj
   updateYawQPGradientVector();
   if(!_yaw_qpSolver.updateGradient(_yaw_gradient))
   {
      printError("_yaw_qpSolver.updateGradient failed to update _gradient");
      return false;
   }
   if(_debug)
      printInfo("Yaw QP gradient is updated");

   return true;
}

/////////////////////// public functions /////////////////////
/////////////////////////////////////////////////////////////

bool 
MPC12STATE::initMPCProblem(void)
{
   initVariables();

   setXYTransitionMatrix(); 
   setXYInputMatrix();
   setXYQ();
   setXYR();
   castXYMPCToQPHessian();
   _xy_current_state.setZero();
   _xy_referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_XY_STATES*(_mpcWindow+1),1);
   castXYMPCToQPGradient();
   castXYMPCToQPConstraintMatrix();
   if(!computeXYBounds()) return false; // This requires computing _z_x_opt
   setXYControlBounds();
   castXYMPCToQPConstraintBounds();

   if(_debug)
      printInfo("[MPC12STATE::initMPCProblem] Initialized XY MPC data");

   setZTransitionMatrix(); 
   setZInputMatrix();
   setZQ();
   setZR();
   castZMPCToQPHessian();
   _z_current_state.setZero();
   _z_referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_Z_STATES*(_mpcWindow+1),1);
   castZMPCToQPGradient();
   castZMPCToQPConstraintMatrix();
   setZStateBounds();
   setZControlBounds();
   castZMPCToQPConstraintBounds();

   if(_debug)
      printInfo("[MPC12STATE::initMPCProblem] Initialized Z MPC data");

   setYawTransitionMatrix(); 
   setYawInputMatrix();
   setYawQ();
   setYawR();
   castYawMPCToQPHessian();
   _yaw_current_state.setZero();
   _yaw_referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_YAW_STATES*(_mpcWindow+1),1);
   castYawMPCToQPGradient();
   castYawMPCToQPConstraintMatrix();
   setYawStateBounds();
   setYawControlBounds();
   castYawMPCToQPConstraintBounds();

   if(_debug)
      printInfo("[MPC12STATE::initMPCProblem] Initialized Yaw MPC data");
   
   _current_state.setZero();
   _referenceTraj = Eigen::MatrixXd::Zero(NUM_OF_STATES*(_mpcWindow+1),1);
   
   if (!initQPSolver())
   {
      printError("[MPC12STATE::initMPCProblem] MPC initialization is not successful.");
      return false;
   }

//   printProblemInfo();
  _is_MPC_initialized = true;

   // double total_elapsed = (ros::WallTime::now() - startTime).toSec();
   // ROS_INFO("MPC is initialized in %f second(s).", total_elapsed);

   return true;
}

// bool MPC12STATE::updateMPC(void)
// {
//    return updateQP();
// }

bool MPC12STATE::updateXYMPC(void)
{
   // Requires solving the Z problem first
   return updateXYQP();
}

bool MPC12STATE::updateZMPC(void)
{
   return updateZQP();
}

bool MPC12STATE::updateYawMPC(void)
{
   // Requires solving the XY problem first
   return updateYawQP();
}

bool 
MPC12STATE::mpcLoop(void)
{
   // ros::WallTime startTime = ros::WallTime::now();

   if(!_is_MPC_initialized)
   {
      printWarn("[MPC12STATE::mpcLoop] MPC controller is not initialized. Skipping MPC loop.");
      return false;
   }
   if(!_state_received)
   {
      printWarn("[MPC12STATE::mpcLoop] curent state is not received. Skipping MPC loop.");
      return false;
   }
   if(!_received_refTraj)
   {
      printWarn("[MPC12STATE::mpcLoop] Reference trajectory is not received. Skipping MPC loop.");
      return false;
   }

   // 1- Solve Z problem first
   
   // Update gradient and bounds
   if(!updateZQP())
   {
      printError("[MPC12STATE::mpcLoop] Z - Failed to update bounds and gradient");
      return false;
   }
   
   // Solve MPC, for Z
   if(!_z_qpSolver.solve())
   {
      printError("[MPC12STATE::mpcLoop] Z -  MPC solution is not found");
      return false;
   }

   extractZSolution();

   // 2- Solve XY problem
   // Update gradient and bounds
   if(!updateXYQP())
   {
      printError("[MPC12STATE::mpcLoop] XY - Failed to update bounds and gradient");
      return false;
   }
   
   // Solve MPC, for XY
   if(!_xy_qpSolver.solve())
   {
      printError("[MPC12STATE::mpcLoop] XY -  MPC solution is not found");
      return false;
   }

   extractXYSolution();

   // 3- Solve Yaw problem
   // Update gradient and bounds
   if(!updateYawQP())
   {
      printError("[MPC12STATE::mpcLoop] Yaw - Failed to update bounds and gradient");
      return false;
   }
   
   // Solve MPC, for Yaw
   if(!_yaw_qpSolver.solve())
   {
      printError("[MPC12STATE::mpcLoop] Yaw -  MPC solution is not found");
      return false;
   }

   extractYawSolution();

   // Merge all solutions into
   // _x_opt, _u_opt
   extractSolution();

   // @todo count time ?
   return true;
}

void 
MPC12STATE::extractXYSolution(void)
{
   Eigen::VectorXd QPSolution;
   if(_debug)
      printInfo("[MPC12STATE::extractXYSolution] Getting optimal solution from _xy_qpSolver");
   QPSolution = _xy_qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _xy_x_opt = QPSolution.block(0, 0, NUM_OF_XY_STATES * (_mpcWindow+1), 1);
   if(_debug)
      printInfo("[MPC12STATE::extractXYSolution] Extracted _xy_x_opt");

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_XY_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_XY_INPUTS*_mpcWindow;
   _xy_u_opt = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _xy_u0_opt = _xy_u_opt.segment(0,NUM_OF_XY_INPUTS);

   return;
}

void 
MPC12STATE::extractZSolution(void)
{
   Eigen::VectorXd QPSolution;
   if(_debug)
      printInfo("[MPC12STATE::extractZSolution] Getting optimal solution from _z_qpSolver");
   QPSolution = _z_qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _z_x_opt = QPSolution.block(0, 0, NUM_OF_Z_STATES * (_mpcWindow+1), 1);
   if(_debug)
      printInfo("[MPC12STATE::extractZSolution] Extracted _z_x_opt");

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_Z_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_Z_INPUTS*_mpcWindow;
   _z_u_opt = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _z_u0_opt = _z_u_opt(0);

   return;
}

void 
MPC12STATE::extractYawSolution(void)
{
   Eigen::VectorXd QPSolution;
   if(_debug)
      printInfo("[MPC12STATE::extractYawSolution] Getting optimal solution from _yaw_qpSolver");
   QPSolution = _yaw_qpSolver.getSolution();

   // State trajectory, [x(0), x(1), ... , x(N)]
   _yaw_x_opt = QPSolution.block(0, 0, NUM_OF_YAW_STATES * (_mpcWindow+1), 1);
   if(_debug)
      printInfo("[MPC12STATE::extractZSolution] Extracted _yaw_x_opt");

   // Control trajectory, [u(0), u(1), ... , u(N-1)]
   auto N_x = NUM_OF_YAW_STATES * (_mpcWindow+1);
   auto N_u = NUM_OF_YAW_INPUTS*_mpcWindow;
   _yaw_u_opt = QPSolution.block(N_x, 0, N_u, 1);

   // Control solution at t=0, u(0)
   _yaw_u0_opt = _yaw_u_opt(0);

   return;
}

void 
MPC12STATE::extractSolution(void)
{
   for(int i=0; i < (_mpcWindow+1); i++)
   {
      MatX_12STATE x_t; x_t.setZero();
      x_t << _xy_x_opt.segment(i*NUM_OF_XY_STATES, NUM_OF_XY_STATES),
               _z_x_opt.segment(i*NUM_OF_Z_STATES, NUM_OF_Z_STATES),
               _yaw_x_opt.segment(i*NUM_OF_YAW_STATES, NUM_OF_Z_STATES);
      _x_opt.segment(i*NUM_OF_STATES, NUM_OF_STATES) = x_t;

      if (i<_mpcWindow)
      {
         MatU_4INPUTS u_t; u_t.setZero();
         u_t << _xy_u_opt.segment(i*NUM_OF_XY_INPUTS, NUM_OF_XY_INPUTS),
                  _z_u_opt.segment(i*NUM_OF_Z_INPUTS, NUM_OF_Z_INPUTS),
                  _yaw_u_opt.segment(i*NUM_OF_YAW_INPUTS, NUM_OF_Z_INPUTS);
         _u_opt.segment(i*NUM_OF_INPUTS, NUM_OF_INPUTS) = u_t;
      }
   }
   if(_debug)
      printInfo("[MPC12STATE::extractSolution] Computed _x_opt and _u_opt");

   return;
}

// void 
// MPC12STATE::printProblemInfo(void)
// {
//    auto opt_x_l = NUM_OF_STATES*(_mpcWindow+1)+NUM_OF_INPUTS*_mpcWindow; // length of optimization variable
//    auto opt_Ac_l_row = 2*NUM_OF_STATES*(_mpcWindow+1) + NUM_OF_INPUTS*_mpcWindow;// number of constraints in matrix Ac

//    auto opt_Ac_size = opt_Ac_l_row * opt_x_l; // Number of elements in Ac

//    printInfo("[MPC12STATE::printProblemInfo]: Number of states = %d",NUM_OF_STATES );
//    printInfo("[MPC12STATE::printProblemInfo]: Number of inputs = %d",NUM_OF_INPUTS );
//    printInfo("[MPC12STATE::printProblemInfo]: Number of MPC steps = %d",_mpcWindow );
//    printInfo("[MPC12STATE::printProblemInfo]: Length of MPC horizon = %f seconds",(double)_mpcWindow*_dt );
//    printInfo("[MPC12STATE::printProblemInfo]: Number of optimization variables = %d",opt_x_l );
//    printInfo("[MPC12STATE::printProblemInfo]: Number constraints = %d", opt_Ac_l_row );
//    printInfo("[MPC12STATE::printProblemInfo]: Number of elements in the constraints matrix Ac = %d",opt_Ac_size );
//    return;
// }

bool
MPC12STATE::setDt(double dt)
{
   if(dt >0)
   {
      _dt = dt;
      if(_debug)
         printInfo("[MPC12STATE::setDt] dt = %f", _dt);
      return true;
   }
   else
   {
      printError("[MPC12STATE::setDt] dt < 0. Defaulting to 0.1");
      _dt = 0.1;
      return false;
   }
}

void
MPC12STATE::setDebug(bool d)
{
   _debug = d;
   if(_debug)
      printInfo("Debug = %d", (int) _debug);
   return;
}

bool
MPC12STATE::setMPCWindow(int N)
{
   if(N>0)
   {
      _mpcWindow = N;
      if(_debug)
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
MPC12STATE::setXYStateWeight(double w)
{
   if (w<0.0)
   {
      printError("xy_state_weight =%f < 0", w);
      return false;
   }

   _xy_state_weight = w;
   if(_debug)
      printInfo("_xy_state_weight = %d", w);
   return true;
}

bool
MPC12STATE::setZStateWeight(double w)
{
   if (w<0.0)
   {
      printError("z_state_weight =%f < 0", w);
      return false;
   }

   _z_state_weight = w;
   if(_debug)
      printInfo("_z_state_weight = %d", w);
   return true;
}

bool
MPC12STATE::setYawStateWeight(double w)
{
   if (w<0.0)
   {
      printError("yaw_state_weight =%f < 0", w);
      return false;
   }

   _yaw_state_weight = w;
   if(_debug)
      printInfo("_yaw_state_weight = %d", w);
   return true;
}


bool
MPC12STATE::setXYInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_xy_input_weight = %f < 0", w);
      return false;
   }
   if(_debug)
      printInfo("_xy_input_weight = %d", w);
   _xy_input_weight = w;
   return true;
}

bool
MPC12STATE::setZInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_z_input_weight = %f < 0", w);
      return false;
   }

   if(_debug)
      printInfo("_z_input_weight = %d", w);
   _z_input_weight = w;
   return true;
}

bool
MPC12STATE::setYawInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_yaw_input_weight = %f < 0", w);
      return false;
   }
   _yaw_input_weight = w;
   if(_debug)
      printInfo("_yaw_input_weight = %d", w);
   return true;
}

bool
MPC12STATE::setXYSmoothInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_xy_smooth_input_weight = %f < 0", w);
      return false;
   }

   _xy_smooth_input_weight = w;
   if(_debug)
      printInfo("_xy_smooth_input_weight = %d", w);
   return true;
}

bool
MPC12STATE::setZSmoothInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_z_smooth_input_weight = %f < 0", w);
      return false;
   }

   _z_smooth_input_weight = w;
   if(_debug)
      printInfo("_z_smooth_input_weight = %d", w);
   return true;
}

bool
MPC12STATE::setYawSmoothInputWeight(double w)
{
   if (w<0.0)
   {
      printError("_yaw_smooth_input_weight = %f < 0", w);
      return false;
   }

   _yaw_smooth_input_weight = w;
   if(_debug)
      printInfo("_yaw_smooth_input_weight = %d", w);
   return true;
}

void
MPC12STATE::enableControlSmoothing(bool b)
{
   _enable_control_smoothing = b;
   if(_debug)
      printInfo("_enable_control_smoothing = %d", (int)_enable_control_smoothing);
   return;
}

bool
MPC12STATE::setAltAboveTarget(double h)
{
   if(h<0)
   {
      printError("_alt_above_target = %f < 0", h);
      return false;
   }
   _alt_above_target = h;
   if(_debug)
      printInfo("_alt_above_target = %f", _alt_above_target);
   return true;
}

bool
MPC12STATE::setMinimumAltitude(double h)
{
   // if(h<0)
   // {
   //    printError("_minAltitude = %f <0", h);
   //    return false;
   // }
   _minAltitude = h;
   return true;
}

bool
MPC12STATE::setCurrentState(const MatX_12STATE &x)
{
   if(x.size() != NUM_OF_STATES)
   {
      printError("State vector size %d != %u", (int) x.size(), NUM_OF_STATES);
      return false;
   }
   // state: [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, yaw, yaw_dot, yaw_ddot]
   // state: [0,   1,      2,    3,   4,    5,    6,    7,    8,     9,   10,        11]

   _current_state.setZero();
   _current_state <<  x;
   _xy_current_state.setZero();
   _xy_current_state = x.head(NUM_OF_XY_STATES);
   _z_current_state.setZero();
   _z_current_state = x.segment(NUM_OF_XY_STATES, NUM_OF_Z_STATES);
   _yaw_current_state.setZero();
   _yaw_current_state = x.tail(NUM_OF_YAW_STATES);

   if(_debug)
   {
      std::cout << "[MPC12STATE::setCurrentState] _current_state" << std::endl << _current_state << std::endl;
      std::cout << "[MPC12STATE::setCurrentState] _xy_current_state" << std::endl << _xy_current_state << std::endl;
      std::cout << "[MPC12STATE::setCurrentState] _z_current_state" << std::endl << _z_current_state << std::endl;
      std::cout << "[MPC12STATE::setCurrentState] _yaw_current_state" << std::endl << _yaw_current_state << std::endl;  
   }
   _state_received = true;
   return true;
}


bool
MPC12STATE::setXYMaxVel(const double v)
{
   if(v < 0)
   {
      printError("maxXYVel %0.3f < 0. Setting default of 10.0 m/s", v);
      _xy_MaxVel = 10.0;
      return false;
   }
   _xy_MaxVel = v;
   return true;
}

bool
MPC12STATE::setZMaxVel(const double v)
{
   if(v < 0)
   {
      printError("maxZVel %0.3f < 0. Setting default of 10.0 m/s", v);
      _z_MaxVel = 10.0;
      return false;
   }
   _z_MaxVel = v;
   return true;
}

bool
MPC12STATE::setYawMaxVel(const double v)
{
   if(v < 0)
   {
      printError("maxYawVel %0.3f < 0. Setting default of 100.0 rad/s", v);
      _yaw_MaxVel = 100.0;
      return false;
   }
   _yaw_MaxVel = v;
   return true;
}

bool
MPC12STATE::setXYMaxAccel(const double a)
{
   if(a < 0)
   {
      printError("maxXYAccel %0.3f < 0. Setting default of 5.0 m/s/s", a);
      _xy_MaxAccel = 5.0;
      return false;
   }
   _xy_MaxAccel = a;
   return true;
}

bool
MPC12STATE::setZMaxAccel(const double a)
{
   if(a < 0)
   {
      printError("maxZAccel %0.3f < 0. Setting default of 5.0 m/s/s", a);
      _z_MaxAccel = 5.0;
      return false;
   }
   _z_MaxAccel = a;
   return true;
}

bool
MPC12STATE::setYawMaxAccel(const double a)
{
   if(a < 0)
   {
      printError("maxYawAccel %0.3f < 0. Setting default of 10.0 rad/s/s", a);
      _yaw_MaxAccel = 100.0;
      return false;
   }
   _yaw_MaxAccel = a;
   return true;
}

bool 
MPC12STATE::setXYMaxJerk(const double j)
{
   if(j < 0)
   {
      printError("_xy_MaxJerk %0.3f < 0. Setting default of 5.0 m/s/s/s", j);
      _xy_MaxJerk = 5.0;
      return false;
   }
   _xy_MaxJerk = j;
   return true;
}

bool 
MPC12STATE::setZMaxJerk(const double j)
{
   if(j < 0)
   {
      printError("_z_MaxJerk %0.3f < 0. Setting default of 5.0 m/s/s/s", j);
      _z_MaxJerk = 5.0;
      return false;
   }
   _z_MaxJerk = j;
   return true;
}

bool 
MPC12STATE::setYawMaxJerk(const double j)
{
   if(j < 0)
   {
      printError("_yaw_MaxJerk %0.3f < 0. Setting default of 100.0 rad/s/s/s", j);
      _yaw_MaxJerk = 100.0;
      return false;
   }
   _yaw_MaxJerk = j;
   return true;
}

bool
MPC12STATE::setReferenceTraj(const Eigen::MatrixXd &v)
{
   if((int)(_referenceTraj.size()) != (int)(NUM_OF_STATES*(_mpcWindow+1)) )
   {
      printError("[MPC12STATE::setReferenceTraj] input matrix size %d != %d", (int)(_referenceTraj.size()), (int)(NUM_OF_STATES*(_mpcWindow+1)) );
      return false;
   }
   _referenceTraj = v;

   for (int i=0; i<(_mpcWindow+1); i++)
   {
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_X_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 0, 0);
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_VX_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 1, 0);
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_AX_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 2, 0);
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_Y_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 3, 0);
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_VY_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 4, 0);
      _xy_referenceTraj(NUM_OF_XY_STATES*i + XY_AY_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 5, 0);

      _z_referenceTraj(NUM_OF_Z_STATES*i + Z_Z_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 6, 0);
      _z_referenceTraj(NUM_OF_Z_STATES*i + Z_VZ_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 7, 0);
      _z_referenceTraj(NUM_OF_Z_STATES*i + Z_AZ_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 8, 0);

      _yaw_referenceTraj(NUM_OF_YAW_STATES*i + YAW_Yaw_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 9, 0);
      _yaw_referenceTraj(NUM_OF_YAW_STATES*i + YAW_VYaw_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 10, 0);
      _yaw_referenceTraj(NUM_OF_YAW_STATES*i + YAW_AYaw_IDX, 0) = _referenceTraj(NUM_OF_STATES*i + 11, 0);
   }
   _received_refTraj = true;
   return true;
}

Eigen::VectorXd
MPC12STATE::getOptimalStateTraj(void)
{
   return _x_opt;
}

Eigen::VectorXd
MPC12STATE::getOptimalControlTraj(void)
{
   return _u_opt;
}

int
MPC12STATE::getNumOfStates(void)
{
   return (int)NUM_OF_STATES;
}

int
MPC12STATE::getNumOfInputs(void)
{
   return (int)NUM_OF_INPUTS;
}

// Eigen::Vector3d
// MPC12STATE::getMaxAccel(void)
// {
//    return _maxAccel;
// }

// Eigen::Vector3d
// MPC12STATE::getMaxVel(void)
// {
//    return _maxVel;
// }

Eigen::MatrixXd MPC12STATE::getTransitionMatrix(void)
{
   // Create full state transition matrix
    Eigen::MatrixXd A(_xy_A.rows() + _z_A.rows() + _yaw_A.rows(), _xy_A.cols() + _z_A.cols() + _yaw_A.cols());
    A.setZero();

    // Place matrices on the diagonal
    A.block(0, 0, _xy_A.rows(), _xy_A.cols()) = _xy_A;
    A.block(_xy_A.rows(), _xy_A.cols(), _z_A.rows(), _z_A.cols()) = _z_A;
    A.block(_xy_A.rows() + _z_A.rows(), _xy_A.cols() + _z_A.cols(), _yaw_A.rows(), _yaw_A.cols()) = _yaw_A;
   return A;
}

Eigen::MatrixXd MPC12STATE::getInputMatrix(void)
{
   // Create full state input matrix
    Eigen::MatrixXd B(_xy_B.rows() + _z_B.rows() + _yaw_B.rows(), _xy_B.cols() + _z_B.cols() + _yaw_B.cols());
    B.setZero();

    // Place matrices on the diagonal
    B.block(0, 0, _xy_B.rows(), _xy_B.cols()) = _xy_B;
    B.block(_xy_B.rows(), _xy_B.cols(), _z_B.rows(), _z_B.cols()) = _z_B;
    B.block(_xy_B.rows() + _z_B.rows(), _xy_B.cols() + _z_B.cols(), _yaw_B.rows(), _yaw_B.cols()) = _yaw_B;
   return B;
}


Eigen::VectorXd MPC12STATE::getGradient(void)
{
   Eigen::VectorXd g(_xy_gradient.size() + _z_gradient.size() + _yaw_gradient.size());
   g << _xy_gradient, _z_gradient, _yaw_gradient;
   return  g;
}

Eigen::MatrixXd MPC12STATE::getQ(void)
{
   // Create full state Q matrix
    Eigen::MatrixXd Q(_xy_Q.rows() + _z_Q.rows() + _yaw_Q.rows(), _xy_Q.cols() + _z_Q.cols() + _yaw_Q.cols());
    Q.setZero();

    // Place matrices on the diagonal
    Q.block(0, 0, _xy_Q.rows(), _xy_Q.cols()) = _xy_Q;
    Q.block(_xy_Q.rows(), _xy_Q.cols(), _z_Q.rows(), _z_Q.cols()) = _z_Q;
    Q.block(_xy_Q.rows() + _z_Q.rows(), _xy_Q.cols() + _z_Q.cols(), _yaw_Q.rows(), _yaw_Q.cols()) = _yaw_Q;
   return Q;
}

Eigen::MatrixXd MPC12STATE::getR(void)
{
   // Create full state R matrix
    Eigen::MatrixXd R(_xy_R.rows() + _z_Q.rows() + _yaw_Q.rows(), _xy_Q.cols() + _z_Q.cols() + _yaw_Q.cols());
    R.setZero();

    // Place matrices on the diagonal
    R.block(0, 0, _xy_R.rows(), _xy_R.cols()) = _xy_R;
    R.block(_xy_R.rows(), _xy_R.cols(), _z_R.rows(), _z_R.cols()) = _z_R;
    R.block(_xy_R.rows() + _z_R.rows(), _xy_R.cols() + _z_R.cols(), _yaw_R.rows(), _yaw_R.cols()) = _yaw_R;
   return R;
}
//////////// Getters for XY /////////////////////
Eigen::MatrixXd MPC12STATE::getXYTransitionMatrix(void) {return _xy_A;}
Eigen::MatrixXd MPC12STATE::getXYInputMatrix(void) {return _xy_B;}
Eigen::VectorXd MPC12STATE::getXYLowerBounds(void) {return _xy_lowerBounds;}
Eigen::VectorXd MPC12STATE::getXYUpperBounds(void) {return _xy_upperBounds;}
Eigen::MatrixXd MPC12STATE::getXYContraintsMatrix(void) {return _xy_Ac;}
Eigen::MatrixXd MPC12STATE::getXYHessianMatrix(void) {return _xy_hessian;}
Eigen::VectorXd MPC12STATE::getXYGradient(void) {return _xy_gradient;}
Eigen::MatrixXd MPC12STATE::getXYQ() {return _xy_Q;}
Eigen::MatrixXd MPC12STATE::getXYR() {return _xy_R;}
double MPC12STATE::getXYMaxVel(void){return _xy_MaxVel;}
double MPC12STATE::getXYMaxAccel(void){return _xy_MaxAccel;}
double MPC12STATE::getXYMaxJerk(void){return _xy_MaxJerk;}
//////////// Getters for Z /////////////////////
Eigen::MatrixXd MPC12STATE::getZTransitionMatrix(void) {return _z_A;}
Eigen::MatrixXd MPC12STATE::getZInputMatrix(void) {return _z_B;}
Eigen::VectorXd MPC12STATE::getZLowerBounds(void) {return _z_lowerBounds;}
Eigen::VectorXd MPC12STATE::getZUpperBounds(void) {return _z_upperBounds;}
Eigen::MatrixXd MPC12STATE::getZContraintsMatrix(void) {return _z_Ac;}
Eigen::MatrixXd MPC12STATE::getZHessianMatrix(void) {return _z_hessian;}
Eigen::VectorXd MPC12STATE::getZGradient(void) {return _z_gradient;}
Eigen::MatrixXd MPC12STATE::getZQ() {return _z_Q;}
Eigen::MatrixXd MPC12STATE::getZR() {return _z_R;}
double MPC12STATE::getZMaxVel(void){return _z_MaxVel;}
double MPC12STATE::getZMaxAccel(void){return _z_MaxAccel;}
double MPC12STATE::getZMaxJerk(void){return _z_MaxJerk;}
//////////// Getters for Yaw /////////////////////
Eigen::MatrixXd MPC12STATE::getYawTransitionMatrix(void) {return _yaw_A;}
Eigen::MatrixXd MPC12STATE::getYawInputMatrix(void) {return _yaw_B;}
Eigen::VectorXd MPC12STATE::getYawLowerBounds(void) {return _yaw_lowerBounds;}
Eigen::VectorXd MPC12STATE::getYawUpperBounds(void) {return _yaw_upperBounds;}
Eigen::MatrixXd MPC12STATE::getYawContraintsMatrix(void) {return _yaw_Ac;}
Eigen::MatrixXd MPC12STATE::getYawHessianMatrix(void) { return _yaw_hessian; }
Eigen::VectorXd MPC12STATE::getYawGradient(void) {return _yaw_gradient;}
Eigen::MatrixXd MPC12STATE::getYawQ() {return _yaw_Q;}
Eigen::MatrixXd MPC12STATE::getYawR() {return _yaw_R;}
double MPC12STATE::getYawMaxVel(void){return _yaw_MaxVel;}
double MPC12STATE::getYawMaxAccel(void){return _yaw_MaxAccel;}
double MPC12STATE::getYawMaxJerk(void){return _yaw_MaxJerk;}

/// @brief //////////////
/// @param path 
void MPC12STATE::setOutputFilePath(std::string path)
{
   _outputCSVFile = path;
}

void
MPC12STATE::saveMPCSolutionsToFile(void)
{
   std::ofstream file(_outputCSVFile);
   if (file.is_open())
   {
      file << "time,x,v_x,a_x,y,v_y,a_y,z,v_z,a_z,yaw,v_yaw,a_yaw,j_x,j_y,j_z,j_yaw,des_x,des_vx,des_ax,des_y,des_vy,des_ay,des_z,des_vz,des_az,des_yaw,des_v_yaw,des_a_yaw,xy_VelMin,xy_VelMax,xy_AccelMin,xy_AccelMax,xy_JerkMin,xy_JerkMax,z_VelMin,z_VelMax,z_AccelMin,z_AccelMax,z_JerkMin,z_JerkMax,yaw_VelMin,yaw_VelMax,yaw_AccelMin,yaw_AccelMax,yaw_JerkMin,yaw_JerkMax\n";
      file << "0.0";// first time stamp
      // Initial state
      for (int i=0; i<NUM_OF_STATES; i++)
      {
         file << "," <<_current_state(i);
      }
      // dummy numbers for input at time=0
      for (int i=0; i<NUM_OF_INPUTS; i++)
      {
         file << "," << "0.0";
      }
      for (int i=0; i<NUM_OF_STATES; i++)
      {
         file << "," <<_referenceTraj(i);
      }
      file << "\n";
      
      // optimal states/inputs & reference trajectory
      for (int i=0; i<(_mpcWindow); i++)
      {
         file << (i+1)*_dt << "," // time
              << _x_opt(NUM_OF_STATES*i + 0) << "," // x
              << _x_opt(NUM_OF_STATES*i + 1) << "," // vx
              << _x_opt(NUM_OF_STATES*i + 2) << "," // ax
              << _x_opt(NUM_OF_STATES*i + 3) << "," // y
              << _x_opt(NUM_OF_STATES*i + 4) << "," // vy
              << _x_opt(NUM_OF_STATES*i + 5) << "," // ay
              << _x_opt(NUM_OF_STATES*i + 6) << "," // z
              << _x_opt(NUM_OF_STATES*i + 7) << "," // vz
              << _x_opt(NUM_OF_STATES*i + 8) << "," // az
              << _x_opt(NUM_OF_STATES*i + 9) << "," // yaw
              << _x_opt(NUM_OF_STATES*i + 10) << "," // vyaw
              << _x_opt(NUM_OF_STATES*i + 11) << "," // ayaw
              << _u_opt(NUM_OF_INPUTS*i + 0) << "," // jx
              << _u_opt(NUM_OF_INPUTS*i + 1) << "," // jy
              << _u_opt(NUM_OF_INPUTS*i + 2) << "," // jz
              << _u_opt(NUM_OF_INPUTS*i + 3) << "," // jyaw
              << _referenceTraj(NUM_OF_STATES*(i+1) + 0) << "," // des_x
              << _referenceTraj(NUM_OF_STATES*(i+1) + 1) << "," // des_vx
              << _referenceTraj(NUM_OF_STATES*(i+1) + 2) << "," // des_ax
              << _referenceTraj(NUM_OF_STATES*(i+1) + 3) << "," // des_y
              << _referenceTraj(NUM_OF_STATES*(i+1) + 4) << "," // des_vy
              << _referenceTraj(NUM_OF_STATES*(i+1) + 5) << "," // des_ay
              << _referenceTraj(NUM_OF_STATES*(i+1) + 6) << "," // des_z
              << _referenceTraj(NUM_OF_STATES*(i+1) + 7) << "," // des_vz
              << _referenceTraj(NUM_OF_STATES*(i+1) + 8) << "," // des_az
              << _yaw_referenceTraj(NUM_OF_YAW_STATES*(i+1) + YAW_Yaw_IDX) << "," // des_yaw
              << _yaw_referenceTraj(NUM_OF_YAW_STATES*(i+1) + YAW_VYaw_IDX) << "," // des_v_yaw
              << _yaw_referenceTraj(NUM_OF_YAW_STATES*(i+1) + YAW_AYaw_IDX) << "," // des_a_yaw
              << -_xy_MaxVel << ","
              << _xy_MaxVel << ","
              << -_xy_MaxAccel << ","
              << _xy_MaxAccel << ","
              << -_xy_MaxJerk << ","
              << _xy_MaxJerk << ","
              << -_z_MaxVel << ","
              << _z_MaxVel << ","
              << -_z_MaxAccel << ","
              << _z_MaxAccel << ","
              << -_z_MaxJerk << ","
              << _z_MaxJerk << ","
              << -_yaw_MaxVel << ","
              << _yaw_MaxVel << ","
              << -_yaw_MaxAccel << ","
              << _yaw_MaxAccel << ","
              << -_yaw_MaxJerk << ","
              << _yaw_MaxJerk << "\n";
      }
      file.close();
      printInfo("[MPC12STATE::saveMPCSolutionsToFile] Saved MPC solutions to file: %s", _outputCSVFile.c_str());
   }
   else
      printError("[MPCROS::saveMPCSolutionsToFile] Coudl not open file %s", _outputCSVFile.c_str());
}

void 
MPC12STATE::saveMPCDataToFile(void)
{
   //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
   const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
   const static Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");
   std::ofstream file(_outputCSVFile);
   if (file.is_open())
   {
      std::string sep= "\n------------------------------------------\n";

      file << "Initial xy state, xy(0): \n";
      file << _xy_current_state.format(CSVFormat) << sep ;

      file << " _xy_A : \n";
      file << _xy_A.format(CleanFmt) << sep;
      
      file << "_xy_B : \n";
      file << _xy_B.format(CleanFmt) << sep;

      file << "_xy_Q : \n";
      file << _xy_Q.format(CleanFmt) << sep;

      file << "_xy_R : \n";
      file << _xy_R.format(CleanFmt) << sep;

      file << "xy Hessian matrix, xy_P: \n";
      file << _xy_hessian.format(CleanFmt) << sep;

      file << "xy Constarints matrix, xy_Ac: \n";
      file << _xy_Ac.format(CleanFmt) << sep;

      file << "xy Lower bounds: \n";
      file << _xy_lowerBounds.format(CleanFmt) << sep;

      file << "xy Upper bounds: \n";
      file << _xy_upperBounds.format(CleanFmt) << sep;

      file << "xy gradient: \n";
      file << _xy_gradient.format(CleanFmt) << sep;

      file << "Optimal xy state trajectory: \n";
      file << _xy_x_opt.format(CleanFmt) << sep;

      file << "Optimal xy control trajectory: \n";
      file << _xy_u_opt.format(CleanFmt) << sep;

      ///////////////// Z data //////////////////
      file << "Initial z state, z(0): \n";
      file << _z_current_state.format(CSVFormat) << sep ;

      file << " _z_A : \n";
      file << _z_A.format(CleanFmt) << sep;
      
      file << "_z_B : \n";
      file << _z_B.format(CleanFmt) << sep;

      file << "_z_Q : \n";
      file << _z_Q.format(CleanFmt) << sep;

      file << "_z_R : \n";
      file << _z_R.format(CleanFmt) << sep;

      file << "z Hessian matrix, z_P: \n";
      file << _z_hessian.format(CleanFmt) << sep;

      file << "z Constarints matrix, z_Ac: \n";
      file << _z_Ac.format(CleanFmt) << sep;

      file << "z Lower bounds: \n";
      file << _z_lowerBounds.format(CleanFmt) << sep;

      file << "z Upper bounds: \n";
      file << _z_upperBounds.format(CleanFmt) << sep;

      file << "z gradient: \n";
      file << _z_gradient.format(CleanFmt) << sep;

      file << "Optimal z state trajectory: \n";
      file << _z_x_opt.format(CleanFmt) << sep;

      file << "Optimal z control trajectory: \n";
      file << _z_u_opt.format(CleanFmt) << sep;

      ///////////////////////////// Yaw data ////////////////////
      file << "Initial yaw state, yaw(0): \n";
      file << _z_current_state.format(CSVFormat) << sep ;

      file << " _yaw_A : \n";
      file << _yaw_A.format(CleanFmt) << sep;
      
      file << "_yaw_B : \n";
      file << _yaw_B.format(CleanFmt) << sep;

      file << "_yaw_Q : \n";
      file << _yaw_Q.format(CleanFmt) << sep;

      file << "_yaw_R : \n";
      file << _yaw_R.format(CleanFmt) << sep;

      file << "yaw Hessian matrix, z_P: \n";
      file << _yaw_hessian.format(CleanFmt) << sep;

      file << "yaw Constarints matrix, z_Ac: \n";
      file << _yaw_Ac.format(CleanFmt) << sep;

      file << "yaw Lower bounds: \n";
      file << _yaw_lowerBounds.format(CleanFmt) << sep;

      file << "yaw Upper bounds: \n";
      file << _yaw_upperBounds.format(CleanFmt) << sep;

      file << "yaw gradient: \n";
      file << _yaw_gradient.format(CleanFmt) << sep;

      file << "Optimal yaw state trajectory: \n";
      file << _yaw_x_opt.format(CleanFmt) << sep;

      file << "Optimal yaw control trajectory: \n";
      file << _yaw_u_opt.format(CleanFmt) << sep;

      file.close();
      printInfo("[MPC12STATE::saveMPCDataToFile] Saved MPC data to file: %s", _outputCSVFile.c_str());
   }
   else
      printError("[MPCROS::saveMPCDataToFile] Coudl not open file %s", _outputCSVFile.c_str());
}