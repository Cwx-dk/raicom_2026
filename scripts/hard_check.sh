#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

LOG_FILE="hardware_test_$(date +%Y%m%d_%H%M%S).log"

REQUIRED_DEVICES=(
    "/dev/base_camera"
    "/dev/head_camera"
    "/dev/rear_lidar"
    "/dev/front_lidar"
)
ORBBEC_DEVICES=(
    "/dev/Orbbec"
    "/dev/Gemini"
)

# ✅ 保证交互提示正常，不提前记录日志
echo -e "${YELLOW}🛠️  请确认已插好所有设备，按回车继续...${NC}"
read -p "" _

# ✅ 从这里开始记录日志
exec > >(tee -a "$LOG_FILE") 2>&1
echo -e "${GREEN}📄 日志文件保存为：$LOG_FILE${NC}"

check_devices() {
    local missing=()
    for dev in "$@"; do
        [ ! -e "$dev" ] && missing+=("$dev")
    done
    echo "${missing[@]}"
}

missing=$(check_devices "${REQUIRED_DEVICES[@]}")
if [ -n "$missing" ]; then
    echo -e "${RED}❌ 缺失以下基础设备：${NC}"
    for dev in $missing; do echo " - $dev"; done
    read -p "是否复制 reinovo_bobac3.rules？(y/n): " choice
    if [[ "$choice" =~ ^[Yy]$ ]]; then
        sudo cp ~/bobac3_ws/scripts/reinovo_bobac3.rules /etc/udev/rules.d/
        sudo udevadm control --reload
        sudo udevadm trigger
        sleep 2
    fi
else
    echo -e "${GREEN}✅ 基础设备就绪${NC}"
fi

missing=$(check_devices "${ORBBEC_DEVICES[@]}")
if [ -n "$missing" ]; then
    echo -e "${RED}❌ 缺失以下奥比中光设备：${NC}"
    for dev in $missing; do echo " - $dev"; done
    read -p "是否复制 99-obsensor-libusb.rules？(y/n): " obs
    if [[ "$obs" =~ ^[Yy]$ ]]; then
        sudo cp ~/dobot_ws/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
        sudo udevadm control --reload
        sudo udevadm trigger
        sleep 2
    fi
else
    echo -e "${GREEN}✅ 奥比中光设备就绪${NC}"
fi

# 最终确认所有设备
final_missing=$(check_devices "${REQUIRED_DEVICES[@]}" "${ORBBEC_DEVICES[@]}")
if [ -z "$final_missing" ]; then
    echo -e "${GREEN}🎉 所有设备检测成功${NC}"
else
    echo -e "${RED}⚠️ 以下设备仍缺失：${NC}"
    for dev in $final_missing; do echo " - $dev"; done
fi

# 启动各项测试
read -p "是否立即测试 Berxel 相机？(y/n): " b
[[ "$b" =~ ^[Yy]$ ]] && ./test_berxel.sh

read -p "是否立即测试 Gemini2？(y/n): " g
[[ "$g" =~ ^[Yy]$ ]] && ./test_gemini2.sh

read -p "是否立即测试普通相机？(y/n): " n
[[ "$n" =~ ^[Yy]$ ]] && ./test_normal_video.sh

read -p "是否立即测试激光雷达？(y/n): " l
[[ "$l" =~ ^[Yy]$ ]] && ./test_lidar.sh

read -p "是否立即测试底盘驱动及 /odom？(y/n): " choice
if [[ "$choice" =~ ^[Yy]$ ]]; then
    ./test_base.sh
    echo -e "${GREEN}✅ 底盘测试完成${NC}"
fi

echo -e "${GREEN}🎯 所有测试任务完成 ✅${NC}"