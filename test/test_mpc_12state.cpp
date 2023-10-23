#include <trajectory_generation/mpc_12state.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char * argv[])
{
  MPC *mpc; /** MPC object */
  mpc = new MPC();

  // Get the start time.
  auto start = std::chrono::high_resolution_clock::now();
  mpc->setDebug(false);

  mpc->setOutputFilePath("/home/user/shared_volume/test_mpc_12state.txt");

  double dt = 0.1;
  printInfo("Setting prediction time sample to %f second (s)", dt);
  mpc->setDt(dt);

  int mpcWindow = 20;
  printInfo("Setting MPC window to %d steps.", mpcWindow);
  if(!mpc->setMPCWindow(mpcWindow))
  {
    printError("Could not set MPC window");
    return 1;
  }

  double state_w = 7000.0;
  printInfo("Setting state weight (for xy_Q matrix) to %f", state_w);
  if(!mpc->setXYStateWeight(state_w))
  {
    printError("Could not set XY state weight");
    return 1;
  }

  state_w = 7000.0;
  printInfo("Setting state weight (for z_Q matrix) to %f", state_w);
  if(!mpc->setZStateWeight(state_w))
  {
    printError("Could not set Z state weight");
    return 1;
  }

  state_w = 7000.0;
  printInfo("Setting state weight (for yaw_Q matrix) to %f", state_w);
  if(!mpc->setYawStateWeight(state_w))
  {
    printError("Could not set Yaw state weight");
    return 1;
  }

  double input_w = 500.0;
  printInfo("Setting XY input weight (for xy_R matrix) to %f", input_w);
  if(!mpc->setXYInputWeight(input_w))
  {
    printError("Could not set XY input weight");
    return 1;
  }

  input_w = 500.0;
  printInfo("Setting Z input weight (for z_R matrix) to %f", input_w);
  if(!mpc->setZInputWeight(input_w))
  {
    printError("Could not set Z input weight");
    return 1;
  }

  input_w = 500.0;
  printInfo("Setting Yaw input weight (for yaw_R matrix) to %f", input_w);
  if(!mpc->setYawInputWeight(input_w))
  {
    printError("Could not set Yaw input weight");
    return 1;
  }
  
  double maxVel=10.0;
  printInfo(" Setting max. horizontal velocity to %0.2f m/s", maxVel);
  if(!mpc->setXYMaxVel(maxVel))
  {
    printError("Could not set horizontal max. velocity");
  }

  maxVel=10.0;
  printInfo(" Setting max. vertical velocity to %0.2f m/s", maxVel);
  if(!mpc->setZMaxVel(maxVel))
  {
    printError("Could not set vertical max. velocity");
  }

  maxVel=100.0;
  printInfo(" Setting max. yaw speed to %0.2f rad/s", maxVel);
  if(!mpc->setYawMaxVel(maxVel))
  {
    printError("Could not set max. yaw speed");
  }
  
  double maxAccel=5.0;
  printInfo(" Setting max. horizontal acceleration to %0.2f m/s", maxAccel);
  if(!mpc->setXYMaxAccel(maxAccel))
  {
    printError("Could not set max. horizontal acceleration");
  }

  maxAccel=5.0;
  printInfo(" Setting max. vertical acceleration to %0.2f m/s", maxAccel);
  if(!mpc->setZMaxAccel(maxAccel))
  {
    printError("Could not set max. vertical acceleration");
  }

  maxAccel=50.0;
  printInfo(" Setting max. yaw acceleration to %0.2f m/s", maxAccel);
  if(!mpc->setYawMaxAccel(maxAccel))
  {
    printError("Could not set max. yaw acceleration");
  }
  
  double maxJerk=5.0;
  printInfo(" Setting max. horizontal jerk to %0.2f m/s", maxJerk);
  if(!mpc->setXYMaxJerk(maxJerk))
  {
    printError("Could not set max. horizontal jerk");
  }

  maxJerk=5.0;
  printInfo(" Setting max. vertical jerk to %0.2f m/s", maxJerk);
  if(!mpc->setZMaxJerk(maxJerk))
  {
    printError("Could not set max. vertical jerk");
  }

  maxJerk=100.0;
  printInfo(" Setting max. yaw jerk to %0.2f m/s", maxJerk);
  if(!mpc->setYawMaxJerk(maxJerk))
  {
    printError("Could not set max. yaw jerk");
  }


  printInfo("Initializing MPC.");
  if(!mpc->initMPCProblem())
  {
    printError("Could not initialize MPC problem");
    return 1;
  }

  printInfo("Settin current state to [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].");
  MatX_12STATE current_state; // [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, yaw, yaw_dot, yaw_ddot ]
  current_state << 0.0, 0.0, 0.0, // x, x_dot, x_ddot
                  0.0, 0.0, 0.0, // y, y_dot, y_ddot
                  0.0, 0.0, 0.0, // z, z_dot, z_ddot
                  0.0, 0.0, 0.0; // yaw, yaw_dot, yaw_ddot
  if(!mpc->setCurrentState(current_state))
  {
    printError("Could not set current state");
    return 1;
  }

  printInfo("Setting the reference trajectory to an altitude of 1.0 and 0.0 for all other states.");
  int ref_size = NUM_OF_STATES*(mpcWindow+1);
  Eigen::MatrixXd ref_traj;
  ref_traj.resize(ref_size,1);
  ref_traj.setZero();
  for (int i=0; i<mpcWindow+1; i++)
  {
    // states order: [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, yaw, yaw_dot, yaw_ddot ]
    ref_traj(i*NUM_OF_STATES+6,0) = 1.0; // z coordinate at all times (constant height)
    ref_traj(i*NUM_OF_STATES+0,0) = 0.1; // x coordinate at all time  
  }
  if(!mpc->setReferenceTraj(ref_traj))
  {
    printError("Could not set reference trajectory");
    return 1;
  }

  printInfo("Solving 12-state 3-stage MPC problem");
  if(!mpc->mpcLoop())
  {
    printError("Could not solve MPC problem");
    return 1;
  }

  printInfo("Etracting solution");
  auto optimal_state_traj = mpc->getOptimalStateTraj();
  auto optimal_control_traj = mpc->getOptimalControlTraj();

  //    // Apply the optimal control inputs to the initial condition
   printInfo("Apply the optimal control inputs to the initial condition");
   Eigen::MatrixXd simulted_state_traj;
   simulted_state_traj.resize(NUM_OF_STATES* (mpcWindow+1), 1);
   simulted_state_traj.setZero();
   auto x0 = current_state;
  //  simulted_state_traj.block(0,0, NUM_OF_STATES,1) = x0;
   for (int i=0; i < mpcWindow; i++)
   {
      x0 = mpc->getTransitionMatrix()*x0 + mpc->getInputMatrix()*mpc->getOptimalControlTraj().segment(i*NUM_OF_INPUTS, NUM_OF_INPUTS);
      simulted_state_traj.block(i*NUM_OF_STATES,0, NUM_OF_STATES,1) = x0;
   }

  printInfo("Calculating total execution time");
  //  Get the end time.
   auto end = std::chrono::high_resolution_clock::now();
   // Compute the difference between the end time and the start time.
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
   double time_in_seconds = duration.count() / 1000.0;
   printInfo(" Test case took %f seconds.", time_in_seconds);

  // Compare the simulated satate to the one computed by the optimization
   auto x_N_opt = mpc->getOptimalStateTraj().segment(NUM_OF_STATES*mpcWindow, NUM_OF_STATES);
   printInfo("Simulated state at time step=%d", mpcWindow);
   std::cout << x0 << "\n";
   printInfo("Optimal state at step=%d", mpcWindow);
   std::cout << x_N_opt << "\n";
   printInfo("At step = %d, Error between simulated and optimal final state = %f", mpcWindow, (x_N_opt-x0).norm());

   mpc->saveMPCSolutionsToFile();


  return 0;
}