#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 所有相机设备列表
CAMERA_DEVICES=(
    "/dev/base_camera"
    "/dev/head_camera"
)
# 激光雷达设备列表
LIDAR_DEVICES=(
    "/dev/rear_lidar"
    "/dev/front_lidar"
)
# 奥比相机设备列表
ORBBEC_DEVICES=(
    "/dev/Orbbec"
    "/dev/Gemini"
)

check_devices() {
    local missing=()
    for dev in "$@"; do
        [ ! -e "$dev" ] && missing+=("$dev")
    done
    echo "${missing[@]}"
}

print_header() {
    clear
    echo -e "${YELLOW}=============================================="
    echo "        硬件设备检测与测试脚本界面"
    echo -e "==============================================${NC}"
}

press_any_key() {
    echo -e "${YELLOW}按任意键返回菜单...${NC}"
    read -n1 -s
}

while true; do
    print_header
    
    missing_cameras=$(check_devices "${CAMERA_DEVICES[@]}")
    missing_lidars=$(check_devices "${LIDAR_DEVICES[@]}")
    missing_orbbec=$(check_devices "${ORBBEC_DEVICES[@]}")

    # 相机设备检测
    if [ -n "$missing_cameras" ]; then
        echo -e "${RED}❌ 缺失相机设备：${NC}"
        echo -e "请检查是否存在video设备文件"
        for d in $missing_cameras; do echo " - $d"; done
    else
        echo -e "${GREEN}✅ 所有相机设备（普通相机和Berxel相机）就绪${NC}"
    fi
    
    # 激光雷达设备检测
    if [ -n "$missing_lidars" ]; then
        echo -e "${RED}❌ 缺失激光雷达设备：${NC}"
        for d in $missing_lidars; do echo " - $d"; done
    else
        echo -e "${GREEN}✅ 所有激光雷达设备就绪${NC}"
    fi
    
    # 奥比相机设备检测
    if [ -n "$missing_orbbec" ]; then
        echo -e "${RED}❌ 缺失奥比中光设备：${NC}"
        for d in $missing_orbbec; do echo " - $d"; done
    else
        echo -e "${GREEN}✅ 奥比中光设备就绪${NC}"
    fi

    echo
    echo -e "${YELLOW}请选择要执行的测试：${NC}"
    echo "1) 复制 reinovo_bobac3.rules (基础设备权限)"
    echo "2) 测试所有相机（普通相机和Berxel相机）"
    echo "3) 奥比相机检测"
    echo "4) 二次定位检测"
    echo "5) 测试 base_control"
    echo "6) 建图和导航测试"
    echo "7) 设置WiFi热点"
    echo "0) 退出"

    read -p "请输入数字选择: " choice
    case $choice in
        1)
            sudo cp ~/bobac3_ws/scripts/reinovo_bobac3.rules /etc/udev/rules.d/
            sudo udevadm control --reload
            sudo udevadm trigger
            echo -e "${GREEN}权限规则已复制并刷新${NC}"
            press_any_key
            ;;
        2)
            ./test_all_cameras.sh
            press_any_key
            ;;
        3)
            clear
            echo -e "${GREEN}===== 奥比相机检测 =====${NC}"
            echo -e "${YELLOW}正在检查奥比相机设备...${NC}"
            
            missing_orbbec=$(check_devices "${ORBBEC_DEVICES[@]}")
            if [ -n "$missing_orbbec" ]; then
                echo -e "${RED}❌ 缺失以下奥比中光设备：${NC}"
                for d in $missing_orbbec; do echo " - $d"; done
                read -p "是否复制 99-obsensor-libusb.rules？(y/n): " obs
                if [[ "$obs" =~ ^[Yy]$ ]]; then
                    sudo cp ~/dobot_ws/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
                    sudo udevadm control --reload
                    sudo udevadm trigger
                    sleep 2
                    echo -e "${GREEN}✅ 奥比中光设备规则已复制并刷新${NC}"
                fi
            else
                echo -e "${GREEN}✅ 奥比中光设备就绪${NC}"
            fi
            press_any_key
            ;;
        4)
            clear
            echo -e "${GREEN}===== 二次定位检测 =====${NC}"
            echo -e "${YELLOW}正在启动二次定位相关节点...${NC}"
            echo
            
            # 检查是否有roscore在运行
            if ! rostopic list >/dev/null 2>&1; then
                echo -e "${YELLOW}未检测到roscore，正在启动roscore...${NC}"
                roscore >/dev/null 2>&1 &
                ROSCORE_PID=$!
                sleep 5
            fi
            
            # 启动各个节点
            echo -e "${GREEN}启动 bobac3_navigation 节点...${NC}"
            roslaunch bobac3_navigation bobac3_nav_2d.launch >/dev/null 2>&1 &
            NAV_PID=$!
            sleep 3
            
            echo -e "${GREEN}启动 ar_pose 节点...${NC}"
            roslaunch ar_pose ar_base.launch >/dev/null 2>&1 &
            AR_PID=$!
            sleep 2
            
            echo -e "${GREEN}启动 relative_move 节点...${NC}"
            roslaunch relative_move relative_move.launch >/dev/null 2>&1 &
            REL_PID=$!
            sleep 2
            
            echo -e "${GREEN}启动 auto_charging 节点...${NC}"
            roslaunch auto_charging auto_charging.launch >/dev/null 2>&1 &
            CHARGE_PID=$!
            sleep 3
            
            # 提供按键执行服务的交互界面
            echo -e "\n${GREEN}===== 二次定位控制界面 =====${NC}"
            echo -e "${YELLOW}按 'e' 键执行 /auto_charging 服务调用${NC}"
            echo -e "${YELLOW}按 'q' 键退出控制界面${NC}"
            
            while true; do
                echo -ne "\r等待按键操作..."
                read -n1 -s key  # 读取单个按键，不回显
                
                case "$key" in
                    e|E)
                        echo -e "\n\n${GREEN}正在执行 /auto_charging 服务调用...${NC}"
                        rosservice call /auto_charging "{nav: false, track_id: 0, track_dist: 0.3, docking_dist: -0.2, pose: {x: 0.0, y: 0.0, theta: 0.0}}"
                        echo -e "${GREEN}✅ 服务调用完成${NC}\n"
                        ;;
                    q|Q)
                        echo -e "\n\n${GREEN}退出控制界面${NC}"
                        break
                        ;;
                esac
            done
            
            echo -e "\n${GREEN}✅ 二次定位检测完成${NC}"
            
            # 提供关闭节点的选项
            read -p "是否关闭已启动的节点？(y/n): " close_nodes
            if [[ "$close_nodes" =~ ^[Yy]$ ]]; then
                echo -e "${GREEN}正在关闭节点...${NC}"
                kill $CHARGE_PID $REL_PID $AR_PID $NAV_PID 2>/dev/null
                if [ -n "$ROSCORE_PID" ]; then
                    kill $ROSCORE_PID 2>/dev/null
                fi
                echo -e "${GREEN}✅ 所有节点已关闭${NC}"
            fi
            press_any_key
            ;;
        5)  
            ./base_control.sh
            echo -e "${GREEN}✅ base_control 测试完成${NC}"
            press_any_key
            ;;
        6)
            ./test_slam_navigation.sh
            echo -e "${GREEN}✅ 建图和导航测试完成${NC}"
            press_any_key
            ;;
        7)
            clear
            echo -e "${GREEN}===== 设置WiFi热点 =====${NC}"
            echo -e "${YELLOW}提示您输入需要的后三位数...${NC}"
            echo
            sudo ./setup_wifi_ap.sh
            press_any_key
            ;;
        0)
            echo "退出程序"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选项，请重新输入${NC}"
            press_any_key
            ;;
    esac
done
