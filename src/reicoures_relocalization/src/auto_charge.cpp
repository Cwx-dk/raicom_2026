/**
 * @file auto_charge.cpp
 * @brief 自动充电节点 - 订阅充电指令，完成后发布充电完成消息
 * 
 * 工作流程：
 * 1. 订阅 /charge_command 话题，等待充电指令
 * 2. 收到指令后执行充电流程：
 *    - 导航到充电桩附近（支持失败重试）
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

// 优化后的四元数：朝向143.2度方向
const double CHARGE_POS_Z = 0.985;
const double CHARGE_POS_W = 0.174;

const int AR_ID = 0;
const double AR_DIST = 0.4;
const double REL_MOVE_FORWARD = -0.18;
const double REL_MOVE_BACK = 0.18;
const int CHARGE_DURATION_SECONDS = 10;

// =========================================================
// 🔥 新增：导航重试参数
// =========================================================
const int MAX_NAV_RETRIES = 5;           // 最大导航重试次数
const double NAV_RETRY_DELAY = 3.0;      // 重试前等待时间（秒）

// 旋转搜索参数（作为备用方案）
const double ROTATION_SPEED = 0.3;
const double ROTATION_INCREMENT = 0.5;
const int MAX_ROTATION_ATTEMPTS = 12;

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
        
        geometry_msgs::Twist rotate_cmd;
        rotate_cmd.angular.z = -ROTATION_SPEED;
        cmd_vel_pub.publish(rotate_cmd);
        
        double rotation_time = ROTATION_INCREMENT / ROTATION_SPEED;
        ros::Duration(rotation_time).sleep();
        
        cmd_vel_pub.publish(stop_cmd);
        ros::Duration(0.3).sleep();
        
        ROS_INFO("🔄 旋转尝试 %d/%d，角度: %.1f°", 
                 attempt + 1, MAX_ROTATION_ATTEMPTS, 
                 (attempt + 1) * ROTATION_INCREMENT * 180 / M_PI);
        
        if (set_ARtrack(AR_ID, AR_DIST)) {
            ROS_INFO("✅ 旋转搜索成功！在第 %d 次尝试找到AR码", attempt + 1);
            return true;
        }
        
        ros::Duration(0.2).sleep();
    }
    
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

/**
 * @brief 导航到目标点（支持重试）
 * @param x 目标x坐标
 * @param y 目标y坐标
 * @param z 四元数z
 * @param w 四元数w
 * @param max_retries 最大重试次数
 * @param retry_delay 重试间隔（秒）
 * @return true 导航成功
 * @return false 导航失败
 */
bool navToGoalWithRetry(double x, double y, double z, double w, 
                        int max_retries = MAX_NAV_RETRIES, 
                        double retry_delay = NAV_RETRY_DELAY) {
    
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    
    int attempt = 0;
    bool success = false;
    
    while (attempt < max_retries && ros::ok()) {
        attempt++;
        ROS_INFO("========================================");
        ROS_INFO("🚗 导航尝试 %d/%d", attempt, max_retries);
        ROS_INFO("📍 目标位置: (%.4f, %.4f)", x, y);
        ROS_INFO("🧭 目标朝向: %.1f°", 2 * atan2(z, w) * 180 / M_PI);
        ROS_INFO("========================================");
        
        // 取消之前的所有导航目标
        nav_client->cancelAllGoals();
        ROS_WARN("已清空所有导航任务！");
        ros::Duration(0.5).sleep();
        
        // 构建导航目标
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.z = z;
        goal.target_pose.pose.orientation.w = w;
        
        ROS_INFO("发送导航目标...");
        nav_client->sendGoal(goal);
        
        // 等待导航完成或失败
        ros::Rate rate(5);
        bool goal_active = true;
        bool nav_failed = false;
        
        while (ros::ok() && goal_active) {
            actionlib::SimpleClientGoalState state = nav_client->getState();
            
            switch (state.state_) {
                case actionlib::SimpleClientGoalState::SUCCEEDED:
                    ROS_INFO("✅ 导航成功！已到达目标点！");
                    success = true;
                    goal_active = false;
                    break;
                    
                case actionlib::SimpleClientGoalState::ABORTED:
                    ROS_ERROR("❌ 导航失败：无法到达目标！");
                    nav_failed = true;
                    goal_active = false;
                    break;
                    
                case actionlib::SimpleClientGoalState::PREEMPTED:
                    ROS_WARN("⚠️ 导航任务已被取消！");
                    nav_failed = true;
                    goal_active = false;
                    break;
                    
                case actionlib::SimpleClientGoalState::REJECTED:
                    ROS_ERROR("❌ 导航目标被服务器拒绝！");
                    nav_failed = true;
                    goal_active = false;
                    break;
                    
                case actionlib::SimpleClientGoalState::LOST:
                    ROS_ERROR("❌ 导航连接丢失！");
                    nav_failed = true;
                    goal_active = false;
                    break;
                    
                default:
                    // 仍在导航中
                    if (attempt > 1) {
                        ROS_INFO_THROTTLE(5.0, "⏳ 导航中... 状态: %s", state.toString().c_str());
                    }
                    break;
            }
            rate.sleep();
        }
        
        // 如果导航成功，返回true
        if (success) {
            return true;
        }
        
        // 如果导航失败，检查是否还有重试机会
        if (nav_failed && attempt < max_retries) {
            ROS_WARN("========================================");
            ROS_WARN("🔄 导航失败，%d 秒后重新尝试...", (int)retry_delay);
            ROS_WARN("剩余重试次数: %d", max_retries - attempt);
            ROS_WARN("========================================");
            
            // 等待后重新规划路径
            ros::Duration(retry_delay).sleep();
            
            // 可选：发布一个停止命令，让机器人停下来重新规划
            geometry_msgs::Twist stop_cmd;
            stop_cmd.angular.z = 0;
            stop_cmd.linear.x = 0;
            cmd_vel_pub.publish(stop_cmd);
            ros::Duration(0.5).sleep();
        }
    }
    
    ROS_ERROR("========================================");
    ROS_ERROR("❌ 导航失败！已尝试 %d 次，无法到达充电桩", max_retries);
    ROS_ERROR("========================================");
    return false;
}

/**
 * @brief 执行完整的充电流程
 * @return true 充电成功完成
 * @return false 充电失败
 */
bool performCharging() {
    ROS_INFO("========================================");
    ROS_INFO("🔋 开始执行充电流程...");
    ROS_INFO("========================================");
    
    // 第一步：导航到充电桩附近（支持重试）
    ROS_INFO("🚗 步骤1: 导航到充电桩附近 (目标朝向: 143.2°)");
    ROS_INFO("📌 最大重试次数: %d", MAX_NAV_RETRIES);
    
    if (!navToGoalWithRetry(CHARGE_POS_X, CHARGE_POS_Y, CHARGE_POS_Z, CHARGE_POS_W)) {
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
    
    ROS_INFO("========================================");
    ROS_INFO("📨 收到充电指令，开始执行充电流程...");
    ROS_INFO("========================================");
    
    bool success = performCharging();
    
    std_msgs::Bool complete_msg;
    complete_msg.data = success;
    charge_complete_pub.publish(complete_msg);
    
    if (success) {
        ROS_INFO("========================================");
        ROS_INFO("✅ 充电完成消息已发布到 /charge_complete");
        ROS_INFO("========================================");
    } else {
        ROS_WARN("========================================");
        ROS_WARN("⚠️ 充电流程失败，但仍发布完成消息（带失败标志）");
        ROS_WARN("========================================");
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
    
    ROS_INFO("========================================");
    ROS_INFO("🤖 自动充电节点已启动");
    ROS_INFO("📡 订阅话题: /charge_command");
    ROS_INFO("📡 发布话题: /charge_complete, /cmd_vel");
    ROS_INFO("📍 充电桩位置: (%.4f, %.4f)", CHARGE_POS_X, CHARGE_POS_Y);
    ROS_INFO("🧭 目标朝向: 143.2° (优化角度，使AR码在视野中央)");
    ROS_INFO("🔄 导航重试次数: %d 次", MAX_NAV_RETRIES);
    ROS_INFO("⏳ 重试间隔: %.1f 秒", NAV_RETRY_DELAY);
    ROS_INFO("========================================");
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