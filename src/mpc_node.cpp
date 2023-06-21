#include "trajectory_generation/mpc_ros.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCROS>());
  rclcpp::shutdown();
  return 0;
}