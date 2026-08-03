#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}===== 启动 Berxel Base Color 相机驱动 =====${NC}"
roslaunch berxel_camera berxel_base_color.launch >/dev/null 2>&1 &
LAUNCH_PID=$!
sleep 5

echo -e "${GREEN}===== 启动图像查看器 rqt_image_view =====${NC}"
rqt_image_view /bobac3_base/rgb/rgb_raw &
RQT_PID=$!

echo -e "${GREEN}===== 开始执行 Berxel Base Color 帧率测试脚本 =====${NC}"
rosrun bobac3_test berxel_camera_test.py

echo -e "${GREEN}===== 关闭 Berxel 相机驱动及图像查看器 =====${NC}"
kill $LAUNCH_PID
kill $RQT_PID
wait $LAUNCH_PID 2>/dev/null
wait $RQT_PID 2>/dev/null

echo -e "${GREEN}✅ Berxel Base Color 相机测试完成${NC}"
