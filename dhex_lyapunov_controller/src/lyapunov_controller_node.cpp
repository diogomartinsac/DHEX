#include <dhex_lyapunov_controller/LyapunovController.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lyapunov_controller");
  ros::NodeHandle param_nh("~");
  LyapunovController lyapunov_controller("lyapunov_controller", param_nh);
  ros::spin();

  return 0;
}