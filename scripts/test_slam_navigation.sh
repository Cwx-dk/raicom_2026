#!/bin/bash

# 设置颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'  # No Color

# 存储所有进程ID
PIDS=()

# 清理函数，确保所有进程都被关闭
cleanup() {
    echo -e "\n${RED}正在清理资源...${NC}"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo -e "${GREEN}所有进程已关闭 ✅${NC}"
}

trap "cleanup; exit 1" SIGINT SIGTERM

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
check_command rostopic

# 启动建图功能
echo -e "${GREEN}===== 启动建图功能 =====${NC}"
roslaunch bobac3_slam bobac3_slam.launch >/dev/null 2>&1 &
SLAM_PID=$!
PIDS+=($SLAM_PID)
sleep 5  # 等待建图初始化

# 监控 /car_data 话题并检测超声波和温湿度数据
echo -e "${GREEN}===== 开始监控传感器数据 =====${NC}"
echo -e "${YELLOW}提示：推动机器人可以观察电机转速变化，接近障碍物可以观察超声波变化${NC}"

echo -e "${GREEN}开始检测超声波和温湿度数据，按 Ctrl+C 停止监控...${NC}"

# 显示数据头信息
echo -e "\n${GREEN}监控数据:${NC}\n"

temp_value=""
humidity_value=""
ultrasound1_prev=""
ultrasound2_prev=""
monitor_count=0

while true; do
    # 获取 /car_data 话题数据
    car_data=$(rostopic echo -n 1 /car_data 2>/dev/null)
    
    if [ -n "$car_data" ]; then
        # 提取超声波数据
        ultrasound=$(echo "$car_data" | grep "ultrasound" | head -1 | awk -F':' '{print $2}' | tr -d '[] ')
        ultrasound1=$(echo "$ultrasound" | cut -d',' -f1)
        ultrasound2=$(echo "$ultrasound" | cut -d',' -f2)
        
        # 提取温湿度数据 - 
        temperature=$(echo "$car_data" | grep "tempareture" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
        humidity=$(echo "$car_data" | grep "relative_humidity" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
        
        # 提取其他重要数据
        power_voltage=$(echo "$car_data" | grep "power_voltage" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
        is_charge=$(echo "$car_data" | grep "is_charge" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
        smoke=$(echo "$car_data" | grep "smoke" | head -1 | awk -F':' '{print $2}' | tr -d ' ')
        
        # 检测超声波是否变化
        ultrasound_changed="否"
        if [ "$ultrasound1" != "$ultrasound1_prev" ] || [ "$ultrasound2" != "$ultrasound2_prev" ]; then
            ultrasound_changed="是"
            ultrasound1_prev="$ultrasound1"
            ultrasound2_prev="$ultrasound2"
        fi
        
        # 检测温湿度是否有数据和变化
        temp_ok="否"
        humidity_ok="否"
        temp_changed="否"
        humidity_changed="否"
        
        # 检查温度数据是否存在
        if [ -n "$temperature" ]; then
            temp_ok="是"
            # 检查温度是否变化
            if [ "$temperature" != "$temp_value" ]; then
                temp_changed="是"
                temp_value="$temperature"
            fi
        fi
        
        # 检查湿度数据是否存在
        if [ -n "$humidity" ]; then
            humidity_ok="是"
            # 检查湿度是否变化
            if [ "$humidity" != "$humidity_value" ]; then
                humidity_changed="是"
                humidity_value="$humidity"
            fi
        fi
        
        # 显示数据
        if [ $monitor_count -eq 0 ]; then
            # 清除之前的显示
            echo -e "\033[2J\033[1;1H"
            echo -e "${GREEN}===== 开始监控传感器数据 =====${NC}"
            echo -e "提示：推动机器人可以观察电机转速变化，接近障碍物可以观察超声波变化"
            echo -e "开始检测超声波和温湿度数据，按 Ctrl+C 停止监控..."
            echo -e "监控数据："
            echo -e "${GREEN}超声波传感器1:${NC} $ultrasound1  ${GREEN}超声波传感器2:${NC} $ultrasound2  ${GREEN}超声波变化:${NC} $ultrasound_changed"
            echo -e "${GREEN}温度:${NC} $temperature °C  ${GREEN}温度正常:${NC} $temp_ok  ${GREEN}温度变化:${NC} $temp_changed"
            echo -e "${GREEN}湿度:${NC} $humidity %  ${GREEN}湿度正常:${NC} $humidity_ok  ${GREEN}湿度变化:${NC} $humidity_changed"
            echo -e "${GREEN}电池电压:${NC} $power_voltage V  ${GREEN}充电状态:${NC} $is_charge  ${GREEN}烟雾传感器:${NC} $smoke"
            echo -e "按 b 键保存地图并启动导航模式"
            
            # 电池电压警告
            if (( $(echo "$power_voltage < 21" | bc -l) )); then
                echo -e "${RED}⚠️ 警告：电池电压低于21V，需要立即充电！${NC}"
            fi
            
            # 烟雾传感器警告
            if [ "$smoke" = "0" ]; then
                echo -e "${RED}⚠️ 警告：检测到可燃气体！${NC}"
            fi
        fi
        
        monitor_count=$((monitor_count + 1))
        if [ $monitor_count -gt 5 ]; then  # 每5个循环刷新一次显示
            monitor_count=0
        fi
    else
        echo -e "${RED}未检测到 /car_data 话题，请检查机器人连接！${NC}"
        sleep 2
    fi
    
    # 检查是否按下b键保存地图
    # 设置终端为非阻塞模式读取键盘输入
    read -t 0.1 -n 1 key 2>/dev/null
    if [ "$key" = "b" ] || [ "$key" = "B" ]; then
        echo -e "\n${GREEN}===== 检测到 b 键，正在保存地图 =====${NC}"
        # 保存地图
        roslaunch map_manager map_save.launch map_file:=home >/dev/null 2>&1
        
        # 检查地图是否保存成功
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}地图保存成功！${NC}"
            # 跳出监控循环，继续执行导航功能
            break
        else
            echo -e "${RED}地图保存失败，请检查map_manager功能包！${NC}"
        fi
    fi
    
    sleep 0.5  # 控制监控频率
done

# 提示如果没有按b键保存地图，可以手动保存
if [ -z "$key" ]; then
    echo -e "\n${YELLOW}您没有按b键保存地图，地图未保存。${NC}"
fi

# 关闭所有进程
echo -e "${GREEN}===== 关闭建图功能 =====${NC}"
cleanup

sleep 3  # 等待进程完全关闭

# 启动导航功能
echo -e "${GREEN}===== 启动导航功能 =====${NC}"
roslaunch bobac3_navigation bobac3_nav_2d.launch map_file_name:=home >/dev/null 2>&1 &
NAV_PID=$!
PIDS+=($NAV_PID)

echo -e "${GREEN}✅ 导航功能已启动，您可以使用导航功能控制机器人移动${NC}"
echo -e "${YELLOW}提示：按 Ctrl+C 可以退出导航功能${NC}"

# 等待导航进程结束
wait $NAV_PID

# 清理资源
cleanup

echo -e "${GREEN}✅ 建图和导航测试完成${NC}"

# 确保脚本有执行权限
if [ ! -x "$0" ]; then
    chmod +x "$0"
    echo -e "${GREEN}已添加脚本执行权限，请重新运行脚本。${NC}"
fi