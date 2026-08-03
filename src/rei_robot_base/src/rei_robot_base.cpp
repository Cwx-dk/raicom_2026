#include <rei_robot_base/rei_robot_base.h>

namespace reinovo_base {
ReiBaseRos::ReiBaseRos(ros::NodeHandle &nh)
    : nh_(nh), soft_estop_(false), kinematics_ptr_(nullptr) {
  max_vel_x_ = 0.0;
  max_vel_y_ = 0.0;
  max_vel_th_ = 0.0;

  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_th_ = 0.0;

  publish_odom_ = false;
  publish_tf_ = false;

  robot_type_ = "unknown";
  RobotBaseCom::GetInstance();
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
  car_data_pub_ = nh_.advertise<rei_robot_base::CarData>("car_data", 50);
  range_pubs_.push_back(nh_.advertise<sensor_msgs::Range>("range1", 5));
  range_pubs_.push_back(nh_.advertise<sensor_msgs::Range>("range2", 5));
  motor_cmd_sub_ =
      nh_.subscribe("motor_cmd", 1, &ReiBaseRos::MotorCmdCallback, this);
  vel_sub_ = nh_.subscribe("cmd_vel", 1, &ReiBaseRos::VelCallback, this);
  set_io_server_ =
      nh_.advertiseService("set_io", &ReiBaseRos::SetIOCallback, this);
  set_relay_server_ =
      nh_.advertiseService("set_relay", &ReiBaseRos::SetRelayCallback, this);
  extra_motor_server_ = nh_.advertiseService(
      "set_extra_motor", &ReiBaseRos::ExtraMotorCallback, this);
  set_buzzer_server_ =
      nh_.advertiseService("set_buzzer", &ReiBaseRos::SetBuzzerCallback, this);
  reset_odom_server_ =
      nh_.advertiseService("reset_odom", &ReiBaseRos::ResetOdomCallback, this);
  base_connect_server_ = nh_.advertiseService(
      "base_connect", &ReiBaseRos::BaseConnectCallback, this);
  // ctrl_mode_server_ =
  //     nh_.advertiseService("ctrl_mode", &ReiBaseRos::CtrlModeCallback, this);
}
bool ReiBaseRos::Init() {
  ros::NodeHandle pnh("~");
  setlocale(LC_CTYPE, "zh_CN.UTF-8");
  std::vector<std::string> existed_robots;
  existed_robots.push_back("bobac2");
  existed_robots.push_back("bobac3");
  existed_robots.push_back("oryxbot");
  existed_robots.push_back("fox");
  range_frame_.clear();
  if (!pnh.getParam("robot", robot_type_)) {
    ROS_ERROR_STREAM("未设置机器人型号");
    return false;
  } else {
    for (size_t i = 0; i < existed_robots.size(); i++) {
      if (robot_type_ == existed_robots[i]) break;
      if (i == existed_robots.size() - 1) {
        ROS_ERROR_STREAM("未知机器人型号: " << robot_type_);
        return false;
      }
    }
    ROS_INFO_STREAM("机器人型号: " << robot_type_);
  }
  if (!pnh.getParam("port", communicate_params_.port)) {
    ROS_ERROR_STREAM("未设置串口号");
    return false;
  }
  if (!pnh.getParam("baudrate", communicate_params_.baud)) {
    ROS_ERROR_STREAM("未设置波特率");
    return false;
  }
  communicate_params_.parity = 'N';
  communicate_params_.data_bit = 8;
  communicate_params_.stop_bit = 1;
  communicate_params_.slave = 1;
  RobotBaseCom::GetInstance().communicate_params_ = communicate_params_;
  if (ConnectBase() < 0) {
    ROS_ERROR("连接下位机失败");
    return false;
  }
  if (!pnh.getParam("kinematics_mode", kin_data_.car_mode)) {
    kin_data_.car_mode = reinovo_base::MotionModel::kNone;
    ROS_INFO("no kinematics_mode set");
    return false;
  } else {
    ROS_INFO("kinematics_mode = %d", kin_data_.car_mode);
  }
  // get wheel_diameter and wheel_separation
  if (!pnh.getParam("wheel_radius", kin_data_.wheel_radius)) {
    // kin_data_.wheel_radius = default_wheel_radius;
    ROS_INFO("use default param \"wheel_radius = %g\"", kin_data_.wheel_radius);
  } else {
    ROS_INFO("wheel_radius = %g", kin_data_.wheel_radius);
  }
  if (!pnh.getParam("wheel_separation_x", kin_data_.wheel_separation_x)) {
    // kin_data_.wheel_separation_x = 0.1;
    ROS_INFO("use default param \"wheel_separation_x = %g\"",
             kin_data_.wheel_separation_x);
  } else {
    ROS_INFO("wheel_separation_x = %g", kin_data_.wheel_separation_x);
  }
  if (!pnh.getParam("wheel_separation_y", kin_data_.wheel_separation_y)) {
    // kin_data_.wheel_separation_y = 0.1;
    ROS_INFO("use default param \"wheel_separation_y = %g\"",
             kin_data_.wheel_separation_y);
  } else {
    ROS_INFO("wheel_separation_y = %g", kin_data_.wheel_separation_y);
  }

  if (!pnh.getParam("motors_index", kin_data_.motors_index)) {
    ROS_INFO("use default param: motors_index = [%d, %d, %d, %d]",
             kin_data_.motors_index[0], kin_data_.motors_index[1],
             kin_data_.motors_index[2], kin_data_.motors_index[3]);
  } else {
    ROS_INFO("motors_index = [%d, %d, %d, %d]", kin_data_.motors_index[0],
             kin_data_.motors_index[1], kin_data_.motors_index[2],
             kin_data_.motors_index[3]);
  }
  if (!pnh.getParam("motors_signs", kin_data_.signs)) {
    ROS_INFO("use default param: motors_signs = [%f, %f, %f, %f]",
             kin_data_.signs[0], kin_data_.signs[1], kin_data_.signs[2],
             kin_data_.signs[3]);
  } else {
    ROS_INFO("motors_signs = [%f, %f, %f, %f]", kin_data_.signs[0],
             kin_data_.signs[1], kin_data_.signs[2], kin_data_.signs[3]);
  }
  if (kinematics_ptr_ != nullptr) kinematics_ptr_.reset();
  kinematics_ptr_ = BaseKinematicsFactory::CreateBaseKinematics(kin_data_);
  if (kinematics_ptr_ == nullptr) {
    ROS_ERROR("底盘运动模型(%d)不支持", kin_data_.car_mode);
    return false;
  }
  if ((robot_type_ == "bobac2") || (robot_type_ == "bobac3")) {
    std::string range_frame_id = "range_right_link";
    pnh.param("range_1_frame_id", range_frame_id, range_frame_id);
    range_frame_.push_back(range_frame_id);

    range_frame_id = "range_left_link";
    pnh.param("range_2_frame_id", range_frame_id, range_frame_id);
    range_frame_.push_back(range_frame_id);
  }
  pnh.param("publish_odom", publish_odom_, true);
  pnh.param("publish_odom_tf", publish_tf_, true);
  pnh.param("odom_frame", odom_frame_, std::string("odom"));
  pnh.param("base_frame", base_frame_, std::string("base_link"));
  pnh.param("max_vel_x", max_vel_x_, max_vel_x_);
  pnh.param("max_vel_y", max_vel_y_, max_vel_y_);
  pnh.param("max_vel_th", max_vel_th_, max_vel_th_);
  output_io_.resize(7, 0);
  return true;
}
int8_t ReiBaseRos::ConnectBase() {
  RobotBaseCom::GetInstance().communicate_params_ = communicate_params_;
  return RobotBaseCom::GetInstance().Connect();
}
void ReiBaseRos::VelLimit(float &vx, float &vy, float &vth) {
  if (vx > max_vel_x_)
    vx = max_vel_x_;
  else if (vx < -max_vel_x_)
    vx = -max_vel_x_;
  if (vy > max_vel_y_)
    vy = max_vel_y_;
  else if (vy < -max_vel_y_)
    vy = -max_vel_y_;
  if (vth > max_vel_th_)
    vth = max_vel_th_;
  else if (vth < -max_vel_th_)
    vth = -max_vel_th_;
}
void ReiBaseRos::MotorCmdCallback(
    const rei_robot_base::MotorCmd::ConstPtr &msg) {
  if (soft_estop_) return;
  std::vector<double> motors_speed;
  for (size_t i = 0; i < msg->motor_expect_speed.size(); i++) {
    motors_speed.push_back(msg->motor_expect_speed[i]);
  }
  if (RobotBaseCom::GetInstance().SendSpeed(motors_speed) < 0) {
    setlocale(LC_CTYPE, "zh_CN.UTF-8");
    ROS_ERROR("发送电机控制指令失败");
  }
}
void ReiBaseRos::VelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  if (soft_estop_) return;
  float vx = msg->linear.x;
  float vy = msg->linear.y;
  float vth = msg->angular.z;
  VelLimit(vx, vy, vth);
  Eigen::Vector3f vel;
  vel << vx, vy, vth;
  Eigen::VectorXf speed = kinematics_ptr_->InverseKinematics(vel);
  std::vector<double> motors_speed;
  for (size_t i = 0; i < speed.size(); i++) {
    motors_speed.push_back(speed[i]);
  }
  if (RobotBaseCom::GetInstance().SendSpeed(motors_speed) < 0) {
    setlocale(LC_CTYPE, "zh_CN.UTF-8");
    ROS_ERROR("发送电机控制指令失败");
  }
}
bool ReiBaseRos::SetIOCallback(rei_robot_base::SetIO::Request &req,
                               rei_robot_base::SetIO::Response &res) {
  uint16_t data[7];
  if ((robot_type_ != "bobac2") && (robot_type_ != "fox")) {
    if (output_io_.size() == 7) {
      for (size_t i = 0; i < 7; i++) {
        data[i] = output_io_[i];
      }

      if (req.all_on && req.all_off) {
        res.success = false;
        res.message = "could not set all_off and all_on both as true";
      } else {
        if (req.all_on) {
          for (size_t i = 0; i < 7; i++) data[i] = 1;
        } else if (req.all_off) {
          for (size_t i = 0; i < 7; i++) data[i] = 0;
        } else {
          for (size_t i = 0; i < req.io.size(); i++) {
            if ((req.io[i] > 7) || req.io[i] < 0) continue;
            if (req.state)
              data[req.io[i]] = 1;
            else
              data[req.io[i]] = 0;
          }
        }
        if (RobotBaseCom::GetInstance().SetIO(data)==0) {
          res.success = true;
          res.message = "ok";
        } else {
          res.success = false;
          res.message = "set output err";
        }
      }
    } else {
      res.success = false;
      res.message = "set output err";
    }
  } else {
    res.success = false;
    res.message = robot_type_ + " is not support";
  }
  return true;
}
bool ReiBaseRos::SetRelayCallback(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res) {
  bool state = req.data;
  if (RobotBaseCom::GetInstance().SetRelay(state)==0) {
    res.success = true;
    res.message = "ok";
  } else {
    res.success = false;
    res.message = "get relay err";
  }
  return true;
}
bool ReiBaseRos::ExtraMotorCallback(rei_robot_base::Int8::Request &req,
                                    rei_robot_base::Int8::Response &res) {
  int state = req.data;
  if (RobotBaseCom::GetInstance().SetExtraMotor(state)==0) {
    res.success = true;
    res.message = "ok";
  } else {
    res.success = false;
    res.message = "get extra motor err";
  }
  return true;
}
bool ReiBaseRos::SetBuzzerCallback(std_srvs::SetBool::Request &req,
                                   std_srvs::SetBool::Response &res) {
  int time = req.data;
  if (RobotBaseCom::GetInstance().SetBuzzer(time)==0) {
    res.success = true;
    res.message = "ok";
  } else {
    res.success = false;
    res.message = "get buzzer err";
  }
  return true;
}
bool ReiBaseRos::ResetOdomCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res) {
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_th_ = 0.0;
  last_odom_time_ = ros::Time::now();
  return true;
}
bool ReiBaseRos::BaseConnectCallback(std_srvs::SetBool::Request &req,
                                     std_srvs::SetBool::Response &res) {
  if (req.data) {
    int ret = ConnectBase();
    if (ConnectBase() < 0) {
      res.success = false;
      res.message = "connect failed";
    } else {
      res.success = true;
      res.message = "ok";
    }
  } else {
    RobotBaseCom::GetInstance().CloseConnect();
    res.success = true;
    res.message = "ok";
  }
  return true;
}
void ReiBaseRos::Run() {
  ros::Rate loop_rate(50);
  last_odom_time_ = ros::Time::now();
  ros::Time current_time;
  std::vector<double> d_data_v;
  std::vector<float> f_data_v;
  std::vector<int> i_data_v;
  double d_data;
  float f_data;
  int i_data;
  bool b_data;
  while (ros::ok()) {
    if (RobotBaseCom::GetInstance().ReadData() < 0) {
      setlocale(LC_CTYPE, "zh_CN.UTF-8");
      ROS_ERROR("读取下位机数据失败");
    } else {
      current_time = ros::Time::now();
      rei_robot_base::CarData car_data;
      RobotBaseCom::GetInstance().GetMotorSpeed(d_data_v);
      car_data.motor_speed.assign(d_data_v.begin(), d_data_v.end());
      // read ultrasound
      if (range_frame_.size() > 0) {
        sensor_msgs::Range range_msg;
        if (RobotBaseCom::GetInstance().GetUltraSound(f_data_v) == 0) {
          for (size_t i = 0; i < range_frame_.size(); i++) {
            range_msg.header.stamp = current_time;
            range_msg.header.frame_id = range_frame_[i];
            if (f_data_v[i] < 0)
              range_msg.range = 0;
            else
              range_msg.range = f_data_v[i] / 1000.0f;
            car_data.ultrasound.push_back(range_msg.range);
            range_pubs_[i].publish(range_msg);
          }
        }
      }
      // read battery
      if (RobotBaseCom::GetInstance().GetPower(f_data) == 0) {
        car_data.power_voltage = f_data;
      }
      if (RobotBaseCom::GetInstance().GetCharge(i_data) == 0) {
        car_data.is_charge = i_data;
      }

      // read input io
      if (RobotBaseCom::GetInstance().GetInputIO(i_data_v) == 0) {
        car_data.input_io[0] = i_data_v[0];
        car_data.input_io[1] = i_data_v[1];
        car_data.input_io[2] = i_data_v[2];
        car_data.input_io[3] = i_data_v[3];
      }

      // read smoke
      if (RobotBaseCom::GetInstance().GetSmoke(i_data) == 0) {
        car_data.smoke = i_data;
      }

      // read Temperature
      if (RobotBaseCom::GetInstance().GetTemperature(f_data) == 0) {
        car_data.tempareture = f_data;
      }

      // read relativehumidity
      if (RobotBaseCom::GetInstance().GetRelativeHumidity(f_data) == 0) {
        car_data.relative_humidity = f_data;
      }
      // read bumper cliff
      if (robot_type_ == "bobac2") {
        // get bumper
        bool bc_flag = false;
        if (RobotBaseCom::GetInstance().GetBumper(i_data_v) == 0) {
          for (size_t i = 0; i < i_data_v.size(); i++) {
            car_data.crash.push_back(i_data_v[i]);
            if (i_data_v[i] == 1) bc_flag = true;
          }
        }

        // get cliff
        if (RobotBaseCom::GetInstance().GetCliff(i_data_v) == 0) {
          for (size_t i = 0; i < i_data_v.size(); i++) {
            car_data.cliff.push_back(i_data_v[i]);
            if (i_data_v[i] == 1) bc_flag = true;
          }
        }
        if (bc_flag && (!soft_estop_))
          soft_estop_ = true;
        else if ((!bc_flag) && soft_estop_)
          soft_estop_ = false;
      }

      // read Relay
      if (RobotBaseCom::GetInstance().GetRelay(b_data) == 0)
        car_data.relay_status = b_data;

      // read output io
      if (RobotBaseCom::GetInstance().GetOutputIO(i_data_v) == 0) {
        for (size_t i = 0; i < i_data_v.size(); i++) {
          car_data.output_io[i] = i_data_v[i];
          output_io_[i] = i_data_v[i];
        }
      }
      double dt = (current_time - last_odom_time_).toSec();
      last_odom_time_ = current_time;
      Eigen::Vector3f real_vel;
      Eigen::VectorXf motor_speed(car_data.motor_speed.size());
      for (size_t i = 0; i < car_data.motor_speed.size(); i++) {
        motor_speed[i] = car_data.motor_speed[i];
      }
      real_vel = kinematics_ptr_->ForwardKinematics(motor_speed);

      double delta_x =
          (real_vel[0] * cos(odom_th_) - real_vel[1] * sin(odom_th_)) * dt;
      double delta_y =
          (real_vel[0] * sin(odom_th_) + real_vel[1] * cos(odom_th_)) * dt;
      double delta_th = real_vel[2] * dt;

      odom_x_ += delta_x;
      odom_y_ += delta_y;
      odom_th_ += delta_th;
      if (publish_odom_) {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.z = sin(odom_th_ / 2);
        odom.pose.pose.orientation.w = cos(odom_th_ / 2);
        odom.twist.twist.linear.x = real_vel[0];
        odom.twist.twist.linear.y = real_vel[1];
        odom.twist.twist.angular.z = real_vel[2];
        odom_pub_.publish(odom);
      }
      if (publish_tf_) {
        static tf2_ros::TransformBroadcaster odom_br;
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = odom_frame_;
        odom_tf.child_frame_id = base_frame_;
        odom_tf.transform.translation.x = odom_x_;
        odom_tf.transform.translation.y = odom_y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.z = sin(odom_th_ / 2);
        odom_tf.transform.rotation.w = cos(odom_th_ / 2);
        odom_br.sendTransform(odom_tf);
      }
      car_data_pub_.publish(car_data);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
// bool ReiBaseRos::CtrlModeCallback(rei_robot_base::Int8::Request &req,
// rei_robot_base::Int8::Response &res) {
//   int mode = req.data;
//   if (mode == 0)
//   {
//     ctrl_mode_ = 0;
//     res.success = true;
//     res.message = "ok";
//   }
//   else if (mode == 1)
//   {
//     ctrl_mode_ = 1;
//     res.success = true;
//     res.message = "ok";
//   }
//   else
//   {
//     res.success = false;
//     res.message = "mode error";
//   }
//   return true;
// }
}  // namespace reinovo_base