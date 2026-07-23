#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"


// 创建服务客户端
ros::ServiceClient client;

bool set_relmove(float x,float y,float theta){
    // 等待服务上线
    ROS_INFO("等待服务 /relative_move 启动...");
    client.waitForExistence();
    ROS_INFO("服务已连接！");
    // 定义服务消息
    relative_move::SetRelativeMove srv;

    // 填充请求数据
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";
    // 发送请求
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("移动成功：%s", srv.response.message.c_str());
            return 1;
        }
        else
        {
            ROS_ERROR("移动失败：%s", srv.response.message.c_str());
            return 0;
        }
    }
    else
    {
        ROS_ERROR("服务调用失败！");
        return 0;
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "relative_move_client");
    ros::NodeHandle nh;
    client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");
    set_relmove(0.5,0,0);
    return 0;
}