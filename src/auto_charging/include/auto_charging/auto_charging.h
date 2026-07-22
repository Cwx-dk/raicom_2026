#ifndef REI_AUTO_CHARGING_H
#define REI_AUTO_CHARGING_H
#include <actionlib/client/simple_action_client.h>
#include <ar_pose/Track.h>
#include <auto_charging/SetCharge.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rei_robot_base/CarData.h>
#include <relative_move/SetRelativeMove.h>
#include <ros/ros.h>
namespace rei_auto_charging {
class AutoCharging {
 public:
  AutoCharging(ros::NodeHandle nh);
  ~AutoCharging();

  void SetChargePileInfo(double x, double y, double theta, uint64_t id,
                         geometry_msgs::Pose2D &move_pose);

 private:
  bool Backoff(float x_dist);
  bool Adjust(uint16_t id, float track_dist);
  bool Nav(geometry_msgs::Pose2D &goal);
  bool AutoChargingCallback(auto_charging::SetCharge::Request &req,
                            auto_charging::SetCharge::Response &res);

 private:
  ros::ServiceClient relative_move_client_;
  ros::ServiceClient track_client_;
  ros::ServiceServer auto_charging_server_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;
};
}  // namespace rei_auto_charging

#endif