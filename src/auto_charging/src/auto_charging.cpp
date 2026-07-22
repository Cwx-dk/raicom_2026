#include <auto_charging/auto_charging.h>
namespace rei_auto_charging {
AutoCharging::AutoCharging(ros::NodeHandle nh) {
  relative_move_client_ =
      nh.serviceClient<relative_move::SetRelativeMove>("relative_move");
  track_client_ = nh.serviceClient<ar_pose::Track>("track");
  auto_charging_server_ = nh.advertiseService(
      "auto_charging", &AutoCharging::AutoChargingCallback, this);
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
      "move_base", true);
}
AutoCharging::~AutoCharging() { delete ac_; }
bool AutoCharging::AutoChargingCallback(
    auto_charging::SetCharge::Request &req,
    auto_charging::SetCharge::Response &res) {
  ROS_INFO("Auto charging request received.");
  if (req.nav) {
    if (!Nav(req.pose)) {
      res.success = false;
      res.message = "Navigation failed.";
      return true;
    }
  }
  if (!Adjust(req.track_id, req.track_dist)) {
    res.success = false;
    res.message = "Adjust failed.";
    return true;
  }
  if (!Backoff(req.docking_dist)) {
    res.success = false;
    res.message = "Backoff failed.";
    return true;
  }
  ros::Duration(1.0).sleep();
  auto chagre_state = ros::topic::waitForMessage<rei_robot_base::CarData>(
      "car_data", ros::Duration(1.0));
  if (chagre_state == nullptr) {
    res.success = false;
    res.message = "Car data not received.";
    return true;
  }
  if (chagre_state->is_charge) {
    res.success = true;
    res.message = "Charging success.";
  } else
    res.message = "power input not detected";
  return true;
}
bool AutoCharging::Nav(geometry_msgs::Pose2D &goal) {
  move_base_msgs::MoveBaseGoal move_goal;
  move_goal.target_pose.header.frame_id = "map";
  move_goal.target_pose.header.stamp = ros::Time::now();
  move_goal.target_pose.pose.position.x = goal.x;
  move_goal.target_pose.pose.position.y = goal.y;
  move_goal.target_pose.pose.orientation.w = cos(goal.theta / 2);
  move_goal.target_pose.pose.orientation.z = sin(goal.theta / 2);
  ac_->sendGoal(move_goal);
  ac_->waitForResult();
  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  }
  return false;
}
bool AutoCharging::Adjust(uint16_t id, float track_dist) {
  ar_pose::Track track_srv;
  track_srv.request.ar_id = id;
  track_srv.request.goal_dist = track_dist;
  track_client_.call(track_srv);
  if (track_srv.response.success) {
    return true;
  }
  return false;
}
bool AutoCharging::Backoff(float x_dist) {
  relative_move::SetRelativeMove backoff_srv;
  backoff_srv.request.goal.x = x_dist;
  backoff_srv.request.global_frame = "odom";
  relative_move_client_.call(backoff_srv);
  if (backoff_srv.response.success) {
    return true;
  }
  return false;
}
}  // namespace rei_auto_charging