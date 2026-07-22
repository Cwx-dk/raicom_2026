#include <auto_charging/auto_charging.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "auto_charging");
  std::shared_ptr<rei_auto_charging::AutoCharging> auto_charging =
      std::make_shared<rei_auto_charging::AutoCharging>(ros::NodeHandle());
  ros::spin();
  return 0;
}
