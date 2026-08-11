/**
 * @file auto_charge.cpp
 * @brief 自动充电节点 - 订阅充电指令，完成后发布充电完成消息
 * 
 * 工作流程：
 * 1. 订阅 /charge_command 话题，等待充电指令
 * 2. 收到指令后执行充电流程：
 *    - 导航到充电桩附近（初始角度优化为143.2°）
 *    - AR码二次定位（若失败则旋转搜索作为备用）
 *    - 相对移动对接
 *    - 回退移动断开
 *    - 模拟充电（或等待实际充电完成）
 * 3. 充电完成后发布 /charge_complete 消息
 */

#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ar_pose/Track.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* nav_client;

ros::ServiceClient relmove_client;
ros::ServiceClient track_client;
ros::Publisher charge_complete_pub;
ros::Publisher cmd_vel_pub;  // 用于旋转控制

// 充电桩位置（地图坐标系）
const double CHARGE_POS_X = 0.2529;    
const double CHARGE_POS_Y = 0.0152;    

// ✅ 优化后的四元数：朝向143.2度方向（根据实际旋转搜索结果优化）
// 这样导航到达后，机器人直接面向充电桩，AR码在视野中央
// yaw = 143.2° = 2.499 rad
// sin(yaw/2) = sin(1.2495) = 0.949
// cos(yaw/2) = cos(1.2495) = 0.316
const double CHARGE_POS_Z = 0.985;     // sin(159.5°/2)
const double CHARGE_POS_W = 0.174;     // cos(159.5°/2)

const int AR_ID = 0;
const double AR_DIST = 0.4;
const double REL_MOVE_FORWARD = -0.18;
const double REL_MOVE_BACK = 0.18;
const int CHARGE_DURATION_SECONDS = 30;

// 旋转搜索参数（作为备用方案）
const double ROTATION_SPEED = 0.3;      // 旋转速度 (rad/s)
const double ROTATION_INCREMENT = 0.5;  // 每次旋转角度增量 (弧度)
const int MAX_ROTATION_ATTEMPTS = 12;   // 最大旋转尝试次数

bool set_ARtrack(int id, float dist) {
    ROS_INFO("等待服务 /track 启动...");
    track_client.waitForExistence();
    ROS_INFO("服务已连接！");
    
    ar_pose::Track srv;
    srv.request.ar_id = id;
    srv.request.goal_dist = dist;
    
    if (track_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("✅ 二次定位成功：%s", srv.response.message.c_str());
            return true;
        } else {
            ROS_WARN("❌ 二次定位失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("track服务调用失败！");
        return false;
    }
}

/**
 * @brief 执行旋转搜索AR码（备用方案）
 * @return true 找到AR码
 * @return false 未找到AR码
 */
bool searchForARCode() {
    ROS_INFO("🔄 开始旋转搜索AR码（备用方案）...");
    
    // 停止所有移动
    geometry_msgs::Twist stop_cmd;
    stop_cmd.angular.z = 0;
    cmd_vel_pub.publish(stop_cmd);
    ros::Duration(0.5).sleep();
    
    // 尝试旋转搜索
    for (int attempt = 0; attempt < MAX_ROTATION_ATTEMPTS; ++attempt) {
        if (!ros::ok()) {
            ROS_WARN("ROS节点关闭，停止搜索");
            return false;
        }
        
        // 发布旋转指令（顺时针旋转，即负方向）
        geometry_msgs::Twist rotate_cmd;
        rotate_cmd.angular.z = -ROTATION_SPEED;  // 顺时针旋转
        cmd_vel_pub.publish(rotate_cmd);
        
        // 计算旋转时间
        double rotation_time = ROTATION_INCREMENT / ROTATION_SPEED;
        ros::Duration(rotation_time).sleep();
        
        // 停止旋转
        cmd_vel_pub.publish(stop_cmd);
        ros::Duration(0.3).sleep();
        
        ROS_INFO("🔄 旋转尝试 %d/%d，角度: %.1f°", 
                 attempt + 1, MAX_ROTATION_ATTEMPTS, 
                 (attempt + 1) * ROTATION_INCREMENT * 180 / M_PI);
        
        // 尝试定位AR码
        if (set_ARtrack(AR_ID, AR_DIST)) {
            ROS_INFO("✅ 旋转搜索成功！在第 %d 次尝试找到AR码", attempt + 1);
            return true;
        }
        
        // 短暂停顿，等待传感器稳定
        ros::Duration(0.2).sleep();
    }
    
    // 停止旋转
    geometry_msgs::Twist stop_cmd2;
    stop_cmd2.angular.z = 0;
    cmd_vel_pub.publish(stop_cmd2);
    
    ROS_ERROR("❌ 旋转搜索失败：未找到AR码");
    return false;
}

bool set_relmove(float x, float y, float theta) {
    ROS_INFO("等待服务 /relative_move 启动...");
    relmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    
    relative_move::SetRelativeMove srv;
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";
    
    if (relmove_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("✅ 移动成功：%s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("❌ 移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("服务调用失败！");
        return false;
    }
}

bool navToGoal(double x, double y, double z, double w) {
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    nav_client->cancelAllGoals();
    ROS_WARN("已清空所有导航任务！");
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;
    
    ROS_INFO("发送导航目标...");
    ROS_INFO("目标朝向角度: %.1f°", 2 * atan2(z, w) * 180 / M_PI);
    nav_client->sendGoal(goal);

    ros::Rate rate(5);
    while (ros::ok()) {
        actionlib::SimpleClientGoalState state = nav_client->getState();
        std::string state_str = state.toString();
        ROS_INFO("当前导航状态：%s", state_str.c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("✅ 导航成功：已到达目标点！");
            return true;
        } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_ERROR("❌ 导航失败：无法到达目标！");
            return false;
        } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_WARN("导航任务已被取消！");
            return false;
        } else if (state == actionlib::SimpleClientGoalState::REJECTED) {
            ROS_ERROR("导航目标被服务器拒绝！");
            return false;
        } else if (state == actionlib::SimpleClientGoalState::LOST) {
            ROS_ERROR("导航连接丢失！");
            return false;
        }
        rate.sleep();
    }
    return false;
}

/**
 * @brief 执行完整的充电流程
 * @return true 充电成功完成
 * @return false 充电失败
 */
bool performCharging() {
    ROS_INFO("🔋 开始执行充电流程...");
    
    // 第一步：导航到充电桩附近（优化朝向143.2°）
    ROS_INFO("🚗 步骤1: 导航到充电桩附近 (目标朝向: 143.2°)");
    if (!navToGoal(CHARGE_POS_X, CHARGE_POS_Y, CHARGE_POS_Z, CHARGE_POS_W)) {
        ROS_ERROR("❌ 导航到充电桩失败！");
        return false;
    }
    
    // 第二步：AR码二次定位
    ROS_INFO("🎯 步骤2: AR码二次定位");
    bool ar_found = set_ARtrack(AR_ID, AR_DIST);
    
    // 如果失败，使用旋转搜索作为备用方案
    if (!ar_found) {
        ROS_WARN("⚠️ 初始AR定位失败，启动旋转搜索备用方案...");
        ar_found = searchForARCode();
    }
    
    if (!ar_found) {
        ROS_ERROR("❌ AR定位失败：无法找到充电桩AR码");
        return false;
    }
    
    // 第三步：前进对接充电
    ROS_INFO("📥 步骤3: 前进对接充电");
    if (!set_relmove(REL_MOVE_FORWARD, 0, 0)) {
        ROS_ERROR("❌ 前进对接失败！");
        return false;
    }
    
    // 第四步：模拟充电
    ROS_INFO("🔌 步骤4: 正在充电... 预计 %d 秒", CHARGE_DURATION_SECONDS);
    ros::Rate rate(1);
    for (int i = 0; i < CHARGE_DURATION_SECONDS; ++i) {
        ROS_INFO("⏳ 充电中... %d%% 完成", (i * 100) / CHARGE_DURATION_SECONDS);
        rate.sleep();
        if (!ros::ok()) {
            ROS_WARN("ROS节点关闭，中断充电");
            return false;
        }
    }
    ROS_INFO("✅ 充电完成！");
    
    // 第五步：回退移动断开
    ROS_INFO("📤 步骤5: 回退断开连接");
    if (!set_relmove(REL_MOVE_BACK, 0, 0)) {
        ROS_ERROR("❌ 回退断开失败！");
        return false;
    }
    
    ros::Duration(2.0).sleep();
    ROS_INFO("✅ 充电流程全部完成！");
    return true;
}

/**
 * @brief 充电指令回调函数
 */
void chargeCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) {
        ROS_INFO("收到充电取消指令，忽略");
        return;
    }
    
    ROS_INFO("📨 收到充电指令，开始执行充电流程...");
    bool success = performCharging();
    
    std_msgs::Bool complete_msg;
    complete_msg.data = success;
    charge_complete_pub.publish(complete_msg);
    
    if (success) {
        ROS_INFO("✅ 充电完成消息已发布到 /charge_complete");
    } else {
        ROS_WARN("⚠️ 充电流程失败，但仍发布完成消息（带失败标志）");
    }
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "auto_charge_node");
    ros::NodeHandle nh;
    
    // 初始化服务客户端
    relmove_client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");
    track_client = nh.serviceClient<ar_pose::Track>("/track");
    nav_client = new MoveBaseClient("move_base", true);
    
    // 初始化发布者
    charge_complete_pub = nh.advertise<std_msgs::Bool>("/charge_complete", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 初始化订阅者
    ros::Subscriber charge_cmd_sub = nh.subscribe("/charge_command", 10,
                                                  chargeCommandCallback);
    
    ROS_INFO("🤖 自动充电节点已启动");
    ROS_INFO("📡 订阅话题: /charge_command");
    ROS_INFO("📡 发布话题: /charge_complete, /cmd_vel");
    ROS_INFO("📍 充电桩位置: (%.4f, %.4f)", CHARGE_POS_X, CHARGE_POS_Y);
    ROS_INFO("🧭 目标朝向: 143.2° (优化角度，使AR码在视野中央)");
    ROS_INFO("⏳ 等待充电指令...");
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    delete nav_client;
    ROS_INFO("🛑 自动充电节点已关闭");
    return 0;
}