#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Publisher status_pub_;
    MoveBaseClient client_;
    bool is_navigating_ = false;
    
public:
    NavNode() : client_("move_base", true) {
        // 等待 move_base 服务器
        ROS_INFO("等待 move_base 服务器...");
        client_.waitForServer();
        ROS_INFO("连接成功！");
        
        // 订阅目标话题（调度节点发布）
        goal_sub_ = nh_.subscribe("/nav_goal", 10, &NavNode::goalCallback, this);
        
        // 发布导航状态话题（调度节点订阅）
        status_pub_ = nh_.advertise<std_msgs::Bool>("/nav_status", 10);
        
        ROS_INFO("导航节点已启动，等待目标...");
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (is_navigating_) {
            ROS_WARN("正在导航中，忽略新目标");
            return;
        }
        
        // 提取目标坐标
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        ROS_INFO("收到导航目标: (%.2f, %.2f)", x, y);
        
        // 构造 move_base 目标
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = msg->pose.orientation;
        if (goal.target_pose.pose.orientation.w == 0 && goal.target_pose.pose.orientation.z == 0) {
            // 如果没设置朝向，默认朝前
            goal.target_pose.pose.orientation.w = 1.0;
        }
        
        // 发送目标并开始导航
        client_.sendGoal(goal);
        is_navigating_ = true;
        
        // 启动状态监控线程（在 spin 中处理）
    }
    
    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            
            if (is_navigating_) {
                actionlib::SimpleClientGoalState state = client_.getState();
                
                // 发布当前状态（用于调试）
                // 也可以发布更详细的状态信息
                
                if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("✅ 导航成功到达目标！");
                    std_msgs::Bool status_msg;
                    status_msg.data = true;
                    status_pub_.publish(status_msg);
                    is_navigating_ = false;
                } 
                else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                    ROS_ERROR("❌ 导航失败！");
                    std_msgs::Bool status_msg;
                    status_msg.data = false;
                    status_pub_.publish(status_msg);
                    is_navigating_ = false;
                }
                else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                    // 正在导航中，不发布状态
                }
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "move_base_nav_client");
    
    NavNode nav_node;
    nav_node.spin();
    
    return 0;
}