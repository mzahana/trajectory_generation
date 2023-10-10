#include <trajectory_generation/mpc_6dof.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char * argv[])
{
  MPC *mpc; /** MPC object */
  mpc = new MPC();

  // Get the start time.
  auto start = std::chrono::high_resolution_clock::now();
  mpc->setDebug(false);

  mpc->setOutputFilePath("/home/user/shared_volume/test_mpc_6dof.txt");

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
  printInfo("Setting state weight (fopr Q matrix) to %f", state_w);
  if(!mpc->setStateWeight(state_w))
  {
    printError("Could not set state weight");
    return 1;
  }

  double input_w = 500.0;
  printInfo("Setting input weight (for R matrix) to %f", input_w);
  if(!mpc->setInputWeight(input_w))
  {
    printError("Could not set input weight");
    return 1;
  }
  printInfo(" Setting max. velocity to 10m/s for xyz");
  std::vector<double> maxVel;
  maxVel.push_back(10.0); maxVel.push_back(10.0); maxVel.push_back(10.0);
  mpc->setMaxVel(maxVel);

  printInfo("Setting max. acceleration to 5m/s/s for xyz");
  std::vector<double> maxAcc;
  maxAcc.push_back(5.0); maxAcc.push_back(5.0); maxAcc.push_back(5.0);
  mpc->setMaxAccel(maxAcc);

  double minAlt = -1.0;
  printInfo("Setting minimum altitude to %f meter(s)", minAlt);
  mpc->setMinimumAltitude(minAlt);

  printInfo("Initializing MPC.");
  if(!mpc->initMPCProblem())
  {
    printError("Could not initialize MPC problem");
    return 1;
  }

  printInfo("Settin current state to [0.0, 0.0, 0.0, 0.0, 0.0, 0.0].");
  MatX current_state; // [px, py, pz, vx, vy, vz]
  current_state << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
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
    // states order: [px, py, pz, vx, vy, vz]
    ref_traj(i*NUM_OF_STATES+2,0) = 1.0; // z coordinate at all times (constant height)
    ref_traj(i*NUM_OF_STATES+0,0) = 0.1; // x coordinate at all time  
  }
  if(!mpc->setReferenceTraj(ref_traj))
  {
    printError("Could not set reference trajectory");
    return 1;
  }

  printInfo("Updating MPC problem");
  if(!mpc->updateMPC())
  {
    printError("Could not update MPC problem");
    return 1;
  }

  printInfo("Solving MPC problem");
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

   mpc->saveMPCDataToFile();


  return 0;
}