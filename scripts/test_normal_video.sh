#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}===== 启动 base_camera 驱动 =====${NC}"
roslaunch ar_pose usbcam_base.launch >/dev/null 2>&1 &
PID1=$!
sleep 2

echo -e "${GREEN}===== 启动 head_camera 驱动 =====${NC}"
roslaunch ar_pose usbcam_head.launch >/dev/null 2>&1 &
PID2=$!
sleep 5

echo -e "${GREEN}===== 启动图像显示窗口 =====${NC}"
rqt_image_view /base_camera/image_raw &
RQT_PID1=$!
rqt_image_view /head_camera/image_raw &
RQT_PID2=$!

sleep 2  # 等待rqt_image_view启动

echo -e "${GREEN}===== 开始执行双摄像头帧率测试脚本 =====${NC}"
rosrun bobac3_test normal_video.py

echo -e "${GREEN}===== 关闭摄像头驱动和图像窗口 =====${NC}"
kill $RQT_PID1 $RQT_PID2
kill $PID1 $PID2
wait $PID1 $PID2 $RQT_PID1 $RQT_PID2 2>/dev/null

echo -e "${GREEN}✅ 普通相机测试完成${NC}"
