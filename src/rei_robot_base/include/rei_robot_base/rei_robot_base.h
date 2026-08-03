#pragma once

#include <nav_msgs/Odometry.h>
#include <rei_robot_base/BumperCliff.h>
#include <rei_robot_base/CarData.h>
#include <rei_robot_base/Int8.h>
#include <rei_robot_base/MotorCmd.h>
#include <rei_robot_base/SetIO.h>
// #include <rei_robot_base/CtrlMode.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

#include "communication/rei_base_communication.h"
#include "kinematics/kinematics_factory.h"

namespace reinovo_base {
class ReiBaseRos {
 public:
  ReiBaseRos(ros::NodeHandle& nh);
  bool Init();
  int8_t ConnectBase();
  void Run();

 private:
  std::shared_ptr<BaseKin> kinematics_ptr_;
  ros::ServiceServer set_io_server_, set_relay_server_, extra_motor_server_, set_buzzer_server_;
  // ros::ServiceServer reset_odom_server_, base_connect_server_, ctrl_mode_server_;
  ros::ServiceServer reset_odom_server_, base_connect_server_;

  ros::Publisher odom_pub_, car_data_pub_;
  std::vector<ros::Publisher> range_pubs_;

  ros::Subscriber motor_cmd_sub_, vel_sub_;
  
  float max_vel_x_;
  float max_vel_y_;
  float max_vel_th_;

  double odom_x_, odom_y_, odom_th_;

  bool publish_tf_;
  bool publish_odom_;
  bool soft_estop_;

  // int8_t ctrl_mode_;

  ros::Time last_odom_time_;

  std::string robot_type_, odom_frame_, base_frame_;
  std::vector<std::string> range_frame_;
  std::vector<int> output_io_;
  KinematicData kin_data_;
  BaseComParams communicate_params_;

  ros::NodeHandle nh_;

private:
  void MotorCmdCallback(const rei_robot_base::MotorCmd::ConstPtr& msg);
  void VelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  bool SetIOCallback(rei_robot_base::SetIO::Request& req, rei_robot_base::SetIO::Response& res);
  bool SetRelayCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool ExtraMotorCallback(rei_robot_base::Int8::Request& req, rei_robot_base::Int8::Response& res);
  bool SetBuzzerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool ResetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool BaseConnectCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  // bool CtrlModeCallback(rei_robot_base::CtrlMode::Request& req, rei_robot_base::CtrlMode::Response& res);

  void VelLimit(float& vx, float& vy, float& vth);
};

}  // namespace reinovo_base