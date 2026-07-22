#!/bin/bash

echo "🚗 启动底盘驱动节点..."
roslaunch rei_robot_base base.launch >/dev/null 2>&1 &
BASE_PID=$!
sleep 4  # 等待驱动初始化

echo "📡 开始执行底盘 /odom 频率测试..."
rosrun bobac3_test test_base.py

echo "🛑 测试完成，关闭底盘驱动..."
sleep 1
kill $BASE_PID
