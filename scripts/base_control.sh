#!/bin/bash

# 设置颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}准备启动底盘驱动...${NC}"
roslaunch rei_robot_base base.launch >/dev/null 2>&1 &
BASE_PID=$!

sleep 5

echo -e "${GREEN}准备启动手柄节点...${NC}"
roslaunch robot_joy robot_joy.launch >/dev/null 2>&1 &
JOY_PID=$!

sleep 3

# 检查底盘 odom 是否发布
if ! rostopic list | grep -q "/odom"; then
    echo -e "${RED}未检测到 /odom 话题，请检查底盘连接！${NC}"
    kill $BASE_PID $JOY_PID
    exit 1
fi

# 检查手柄话题
if ! rostopic list | grep -q "/joy"; then
    echo -e "${RED}未检测到 /joy 话题，请检查手柄连接！${NC}"
    kill $BASE_PID $JOY_PID
    exit 1
fi

echo -e "${GREEN}底盘与手柄均已检测到，按 'q' 键退出控制...${NC}"

# 等待用户输入 q 退出
while true; do
    read -n1 key
    if [[ $key == "q" ]]; then
        echo -e "\n${GREEN}正在退出控制...${NC}"
        kill $BASE_PID $JOY_PID
        break
    fi
done

sleep 1
echo -e "${GREEN}所有节点已关闭 ✅${NC}"
