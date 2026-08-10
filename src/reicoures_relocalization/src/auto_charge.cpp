/**
 * @file auto_charge.cpp
 * @brief 自动充电节点 - 订阅充电指令，完成后发布充电完成消息
 * 
 * 工作流程：
 * 1. 订阅 /charge_command 话题，等待充电指令
 * 2. 收到指令后执行充电流程：
 *    - 导航到充电桩附近
 *    - AR码二次定位
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

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* nav_client;

ros::ServiceClient relmove_client;
ros::ServiceClient track_client;
ros::Publisher charge_complete_pub;

// 充电参数（可根据实际情况调整）
// 在地图中测量充电桩的实际坐标
const double CHARGE_POS_X = 0.5058;
const double CHARGE_POS_Y = -0.0052;
const double CHARGE_POS_Z = 0;
const double CHARGE_POS_W = 0;
const int AR_ID = 0;
// 根据摄像头和充电口的实际距离调整
const double AR_DIST = 0.4;
// 如果充电口需要更精确对接，调整这个值
const double REL_MOVE_FORWARD = -0.18;
// 确保完全断开连接
const double REL_MOVE_BACK = 0.18;
const int CHARGE_DURATION_SECONDS = 30;  // 充电持续时间（秒），实际可改为等待电池电量到达阈值

bool set_ARtrack(int id, float dist) {
    // 等待服务上线
    ROS_INFO("等待服务 /track 启动...");
    track_client.waitForExistence();
    ROS_INFO("服务已连接！");
    ar_pose::Track srv;
    srv.request.ar_id = id;
    srv.request.goal_dist = dist;
    // 发送请求
    if (track_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("二次定位成功：%s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("二次定位失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("track服务调用失败！");
        return false;
    }
}

bool set_relmove(float x, float y, float theta) {
    // 等待服务上线
    ROS_INFO("等待服务 /relative_move 启动...");
    relmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    // 定义服务消息
    relative_move::SetRelativeMove srv;

    // 填充请求数据
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";
    // 发送请求
    if (relmove_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("移动成功：%s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("服务调用失败！");
        return false;
    }
}

bool navToGoal(double x, double y, double z, double w) {
    // 等待服务器连接成功
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    nav_client->cancelAllGoals();
    ROS_WARN("已清空所有导航任务！");
    // 构造导航目标消息
    move_base_msgs::MoveBaseGoal goal;

    // 设置坐标系为 map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标坐标（可修改 x, y）
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // 设置朝向
    goal.target_pose.pose.orientation.z = z;   // sin(yaw/2);
    goal.target_pose.pose.orientation.w = w;    // cos(yaw/2);
    // 发送目标点
    ROS_INFO("发送导航目标...");
    nav_client->sendGoal(goal);

    // 循环监听导航状态
    ros::Rate rate(5);
    while (ros::ok()) {
        actionlib::SimpleClientGoalState state = nav_client->getState();
        std::string state_str = state.toString();

        // 实时打印状态
        ROS_INFO("当前导航状态：%s", state_str.c_str());

        // 导航成功
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("导航成功：已到达目标点！");
            return true;
        }
        // 导航失败（内部错误/障碍物/无法规划）
        else if (state == actionlib::SimpleClientGoalState::ABORTED) {
            ROS_ERROR("导航失败：无法到达目标！");
            return false;
        }
        // 任务被取消
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_WARN("导航任务已被取消！");
            return false;
        }
        // 任务被拒绝
        else if (state == actionlib::SimpleClientGoalState::REJECTED) {
            ROS_ERROR("导航目标被服务器拒绝！");
            return false;
        }
        // 导航超时
        else if (state == actionlib::SimpleClientGoalState::LOST) {
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
    
    // 第一步：导航到充电桩附近
    ROS_INFO("🚗 步骤1: 导航到充电桩附近");
    if (!navToGoal(CHARGE_POS_X, CHARGE_POS_Y, CHARGE_POS_Z, CHARGE_POS_W)) {
        ROS_ERROR("导航到充电桩失败！");
        return false;
    }
    
    // 第二步：设置AR追踪定位
    ROS_INFO("🎯 步骤2: AR码二次定位");
    if (!set_ARtrack(AR_ID, AR_DIST)) {
        ROS_ERROR("AR定位失败！");
        return false;
    }
    
    // 第三步：前进对接充电
    ROS_INFO("📥 步骤3: 前进对接充电");
    if (!set_relmove(REL_MOVE_FORWARD, 0, 0)) {
        ROS_ERROR("前进对接失败！");
        return false;
    }
    
    // 第四步：模拟充电（等待充电完成）
    ROS_INFO("🔌 步骤4: 正在充电... 预计 %d 秒", CHARGE_DURATION_SECONDS);
    
    // 实际项目中，这里可以改为：
    // 1. 订阅电池电量话题，直到电量达到阈值
    // 2. 或者等待充电桩的状态反馈
    // 3. 或者等待固定时间后认为充电完成
    
    ros::Rate rate(1);  // 1Hz
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
        ROS_ERROR("回退断开失败！");
        return false;
    }
    
    // 延时2秒
    ros::Duration(2.0).sleep();
    
    ROS_INFO("✅ 充电流程全部完成！");
    return true;
}

/**
 * @brief 充电指令回调函数
 * 收到充电指令后执行充电流程，完成后发布充电完成消息
 */
void chargeCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) {
        ROS_INFO("收到充电取消指令，忽略");
        return;
    }
    
    ROS_INFO("📨 收到充电指令，开始执行充电流程...");
    
    // 执行充电流程
    bool success = performCharging();
    
    // 发布充电完成消息
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
    
    // 初始化发布者（充电完成消息）
    charge_complete_pub = nh.advertise<std_msgs::Bool>("/charge_complete", 10);
    
    // 初始化订阅者（充电指令）
    ros::Subscriber charge_cmd_sub = nh.subscribe("/charge_command", 10,
                                                  chargeCommandCallback);
    
    ROS_INFO("🤖 自动充电节点已启动");
    ROS_INFO("📡 订阅话题: /charge_command");
    ROS_INFO("📡 发布话题: /charge_complete");
    ROS_INFO("⏳ 等待充电指令...");
    
    // 主循环
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    delete nav_client;
    ROS_INFO("🛑 自动充电节点已关闭");
    return 0;
}