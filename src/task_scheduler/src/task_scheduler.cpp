#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>

// 目标点结构
struct Waypoint {
    std::string name;
    double x;
    double y;
    double angle;        // 弧度
    std::string description;
};

class TaskScheduler {
private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber status_sub_;
    
    std::vector<Waypoint> route_;
    int current_index_ = 0;
    bool waiting_for_nav_ = false;
    bool nav_success_ = false;
    
public:
    TaskScheduler() {
        // 发布导航目标
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/nav_goal", 10);
        
        // 订阅导航状态
        status_sub_ = nh_.subscribe("/nav_status", 10, &TaskScheduler::statusCallback, this);
        
        // 初始化路线（你可以动态从参数服务器或文件加载）
        initRoute();
        
        ROS_INFO("调度节点已启动，共 %zu 个目标点", route_.size());
    }
    
    void initRoute() {
        // ===== 定义路线 =====
        route_ = {
            {"吉林", 2.490, 2.190, 0.0, "这里是展厅A，主要展示智能机器人技术"},
            {"展厅B", 2.474, 1.181, 0.0, "这里是展厅B，介绍人工智能算法应用"},
            {"展厅C", 2.476, 0.203, 0.0, "这里是展厅C，展示自动驾驶模拟系统"},
            // 可以继续添加更多目标
        };
        
        // 也可以从 ROS 参数服务器加载
        // nh_.getParam("/route", route_);
    }
    
    void statusCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!waiting_for_nav_) {
            return;  // 不是我们等待的状态
        }
        
        if (msg->data) {
            // 导航成功
            ROS_INFO("✅ 成功到达 %s", route_[current_index_].name.c_str());
            nav_success_ = true;
            waiting_for_nav_ = false;
            
            // 播报场地介绍
            speak(route_[current_index_].description);
            
            // 等待一下，让用户有时间听介绍
            ros::Duration(2.0).sleep();
            
            // 移动到下一个目标
            current_index_++;

            // 自动发送下一个目标
            sendNextGoal();
        } else {
            // 导航失败
            ROS_ERROR("❌ 导航到 %s 失败！", route_[current_index_].name.c_str());
            waiting_for_nav_ = false;
            
            // 可以选择重试或跳过
            if (current_index_ < route_.size() - 1) {
                ROS_WARN("跳过该目标，继续下一个...");
                current_index_++;
                sendNextGoal();
            }
        }
    }
    
    void speak(const std::string& text) {
        // 方法1: 使用 espeak（需安装）
        std::string cmd = "espeak -v zh \"" + text + "\" 2>/dev/null &";
        system(cmd.c_str());
        
        // 方法2: 发布到语音话题（如果你有语音合成节点）
        // std_msgs::String msg;
        // msg.data = text;
        // speech_pub_.publish(msg);
        
        ROS_INFO("🔊 语音播报: %s", text.c_str());
    }
    
    void sendNextGoal() {
        if (current_index_ >= route_.size()) {
            ROS_INFO("所有目标点已访问完成！");
            // 返回起点
            returnToStart();
            return;
        }
        
        Waypoint& wp = route_[current_index_];
        ROS_INFO("🔄 发送第 %d/%zu 个目标: %s (%.2f, %.2f)", 
                 current_index_+1, route_.size(), wp.name.c_str(), wp.x, wp.y);
        
        // 构造目标消息
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.orientation.z = sin(wp.angle / 2.0);
        goal.pose.orientation.w = cos(wp.angle / 2.0);
        
        // 发布目标
        goal_pub_.publish(goal);
        waiting_for_nav_ = true;
        nav_success_ = false;
    }
    
    void returnToStart() {
        ROS_INFO("🏠 所有任务完成，返回起点...");
        
        // 假设起点是 (0, 0)
        geometry_msgs::PoseStamped home;
        home.header.frame_id = "map";
        home.header.stamp = ros::Time::now();
        home.pose.position.x = 0.0;
        home.pose.position.y = 0.0;
        home.pose.orientation.w = 1.0;
        
        goal_pub_.publish(home);
        waiting_for_nav_ = true;
        
        // 等待返回完成
        ros::Rate rate(5);
        while (waiting_for_nav_ && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("🎉 全部任务完成！");
    }
    
    void start() {
        // 延迟一下确保订阅已经建立
        ros::Duration(0.5).sleep();
        
        // 发送第一个目标
        current_index_ = 0;
        sendNextGoal();
        
        // 进入主循环
        ros::spin();
    }
};

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "task_scheduler");
    
    TaskScheduler scheduler;
    scheduler.start();
    
    return 0;
}