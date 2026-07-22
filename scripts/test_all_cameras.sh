#!/bin/bash

# 设置颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'  # No Color

# 存储所有进程ID
PIDS=()
RQT_PIDS=()

# 清理函数，确保所有进程都被关闭
trap "echo -e '\n${RED}检测到中断，正在清理资源...${NC}'; cleanup; exit 1" SIGINT SIGTERM

cleanup() {
    echo -e "${GREEN}正在关闭所有相机驱动和图像窗口...${NC}"
    for pid in "${RQT_PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo -e "${GREEN}所有进程已关闭 ✅${NC}"
}

# 检查命令是否存在
check_command() {
    command -v "$1" >/dev/null 2>&1 || {
        echo -e "${RED}错误：未找到命令 $1，请先安装！${NC}"
        exit 1
    }
}

# 检查必要的命令
echo -e "${GREEN}检查必要的命令...${NC}"
check_command roscore
check_command roslaunch
check_command rosrun
check_command rqt_image_view
check_command rostopic

# 启动普通相机驱动
echo -e "${GREEN}===== 启动 base_camera 驱动 =====${NC}"
roslaunch ar_pose usbcam_base.launch >/dev/null 2>&1 &
BASE_CAMERA_PID=$!
PIDS+=($BASE_CAMERA_PID)
sleep 2

echo -e "${GREEN}===== 启动 head_camera 驱动 =====${NC}"
roslaunch ar_pose usbcam_head.launch >/dev/null 2>&1 &
HEAD_CAMERA_PID=$!
PIDS+=($HEAD_CAMERA_PID)
sleep 2

# 启动 Berxel 相机驱动
echo -e "${GREEN}===== 启动 Berxel Base Color 相机驱动 =====${NC}"
roslaunch berxel_camera berxel_base_color.launch >/dev/null 2>&1 &
BERXEL_PID=$!
PIDS+=($BERXEL_PID)
sleep 3

# 启动图像显示窗口
    echo -e "${GREEN}===== 启动图像显示窗口 =====${NC}"
    rqt_image_view /base_camera/image_raw &
    RQT_BASE_PID=$!
    RQT_PIDS+=($RQT_BASE_PID)

    sleep 1  # 给rqt_image_view启动时间
    sleep 1

    # 添加10秒观察时间
    echo -e "${GREEN}===== 正在进行10秒rqt观察，您可以查看相机画面... =====${NC}"
    sleep 10

    echo -e "${GREEN}===== 开始执行所有相机的帧率测试 =====${NC}"}]}}}

# 检查所有相机话题是否正常发布
ALL_TOPICS_OK=true

if ! rostopic list | grep -q "/base_camera/image_raw"; then
    echo -e "${RED}未检测到 /base_camera/image_raw 话题，请检查相机连接！${NC}"
    ALL_TOPICS_OK=false
fi

if ! rostopic list | grep -q "/head_camera/image_raw"; then
    echo -e "${RED}未检测到 /head_camera/image_raw 话题，请检查相机连接！${NC}"
    ALL_TOPICS_OK=false
fi

if ! rostopic list | grep -q "/bobac3_base/rgb/rgb_raw"; then
    echo -e "${RED}未检测到 /bobac3_base/rgb/rgb_raw 话题，请检查Berxel相机连接！${NC}"
    ALL_TOPICS_OK=false
fi

if [ "$ALL_TOPICS_OK" = true ]; then
    echo -e "${GREEN}所有相机话题已检测到，开始测试...${NC}"
    # 运行普通相机测试脚本
    rosrun bobac3_test normal_video.py
    
    echo -e "${GREEN}\n===== 开始 Berxel 相机测试 =====${NC}"
    # 运行Berxel相机测试脚本
    rosrun bobac3_test berxel_camera_test.py
else
    echo -e "${RED}部分相机话题未检测到，无法进行全部测试！${NC}"
fi

# 测试完成，清理资源
cleanup

echo -e "${GREEN}✅ 所有相机测试完成${NC}"

# 确保脚本有执行权限
if [ ! -x "$0" ]; then
    chmod +x "$0"
    echo -e "${GREEN}已添加脚本执行权限，请重新运行脚本。${NC}"
fi