#include <rei_robot_base/rei_robot_base.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "rei_robot_base");
  ros::NodeHandle nh;
  reinovo_base::ReiBaseRos rei_base_ros(nh);
  if (!rei_base_ros.Init()) {
    ROS_ERROR("Failed to initialize rei_robot_base");
    return -1;
  }
  rei_base_ros.Run();
  return 0;
}