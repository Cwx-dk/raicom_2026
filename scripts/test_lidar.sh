#!/bin/bash

# 设置颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}准备启动激光雷达驱动节点...${NC}"
roslaunch cspc_lidar bobac3_lidar.launch >/dev/null 2>&1 &
LAUNCH_PID=$!
sleep 5
roslaunch bobac3_description display.launch >/dev/null 2>&1 &
DISPLAY_PID=$!

# 检查节点是否成功运行
if ! rostopic list | grep -q "/front_lidar/scan"; then
    echo -e "${RED}未检测到 /front_lidar/scan 话题，请检查雷达连接或驱动配置！${NC}"
    kill $LAUNCH_PID
    kill $DISPLAY_PID
    exit 1
fi

if ! rostopic list | grep -q "/rear_lidar/scan"; then
    echo -e "${RED}未检测到 /rear_lidar/scan 话题，请检查雷达连接或驱动配置！${NC}"
    kill $LAUNCH_PID
    kill $DISPLAY_PID
    exit 1
fi

rviz -d ~/bobac3_ws/src/bobac3_test/rviz/display_scan.rviz &
RVIZ_PID=$!

echo -e "${GREEN}激光雷达话题已检测到，开始运行帧率测试脚本...${NC}"
rosrun bobac3_test lidar_test.py

echo -e "${GREEN}测试完成，准备关闭激光雷达驱动...${NC}"
kill $LAUNCH_PID
kill $DISPLAY_PID
kill $RVIZ_PID

sleep 1
echo -e "${GREEN}所有测试任务完成 ✅${NC}"
