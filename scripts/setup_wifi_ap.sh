#!/bin/bash

# 设置颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'  # No Color

# 脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_FILE="${SCRIPT_DIR}/rc.local.template"
RC_LOCAL_PATH="/etc/rc.local"

# 检查是否在Windows环境下运行（仅供提示）
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    echo -e "${YELLOW}注意：当前在Windows环境下运行，某些功能可能受限。在实际部署时请在Linux系统上运行。${NC}\n"
fi

# 提示用户输入WiFi名称的最后三位
read -p "请输入WiFi名称的最后三位字符: " suffix

# 验证输入长度
if [ ${#suffix} -ne 3 ]; then
    echo -e "${RED}错误：输入必须是3个字符！${NC}"
    exit 1
fi

# 构建完整的WiFi名称
wifi_name="RB3H01${suffix}"

# 检查模板文件是否存在
if [ ! -f "${TEMPLATE_FILE}" ]; then
    echo -e "${RED}错误：模板文件 ${TEMPLATE_FILE} 不存在！${NC}"
    exit 1
fi

# 创建临时文件用于修改
TEMP_FILE="${SCRIPT_DIR}/rc.local.tmp"

# 复制模板文件并替换WiFi名称
echo -e "${GREEN}正在修改WiFi名称...${NC}"
sed "s/RB3H01xxx/${wifi_name}/g" "${TEMPLATE_FILE}" > "${TEMP_FILE}"

if [ $? -ne 0 ]; then
    echo -e "${RED}修改WiFi名称失败！${NC}"
    rm -f "${TEMP_FILE}" 2>/dev/null
    exit 1
fi

# 输出即将应用的WiFi信息
echo -e "\n${GREEN}即将应用的WiFi信息：${NC}"
echo -e "WiFi名称: ${YELLOW}${wifi_name}${NC}"
echo -e "WiFi密码: ${YELLOW}reinovo888${NC}"

# 显示修改后的内容摘要
echo -e "\n${GREEN}修改后的rc.local内容：${NC}"
grep -E "create_ap|RB3H01|wlan1" "${TEMP_FILE}"

echo -e "\n${YELLOW}注意：此操作需要root权限才能应用到系统中。${NC}"

# 询问用户是否立即应用
read -p "是否立即将修改后的内容应用到${RC_LOCAL_PATH}？(y/n): " apply_now

if [ "${apply_now}" = "y" ] || [ "${apply_now}" = "Y" ]; then
    # 检查是否有sudo权限
    if ! sudo -n true 2>/dev/null; then
        echo -e "${YELLOW}需要输入密码以获取root权限...${NC}"
    fi
    
    # 备份原文件（如果存在）
    if [ -f "${RC_LOCAL_PATH}" ]; then
        echo -e "${GREEN}正在备份原文件...${NC}"
        sudo cp "${RC_LOCAL_PATH}" "${RC_LOCAL_PATH}.bak" 2>/dev/null
        if [ $? -ne 0 ]; then
            echo -e "${RED}备份原文件失败！${NC}"
        fi
    fi
    
    # 复制修改后的文件到目标位置
    echo -e "${GREEN}正在应用修改...${NC}"
    sudo cp "${TEMP_FILE}" "${RC_LOCAL_PATH}" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        # 设置执行权限
        sudo chmod +x "${RC_LOCAL_PATH}" 2>/dev/null
        echo -e "${GREEN}\n✅ WiFi名称已成功修改并应用到${RC_LOCAL_PATH}！${NC}"
        echo -e "${YELLOW}提示：系统重启后，WiFi热点将自动以新名称启动。${NC}"
    else
        echo -e "${RED}应用修改失败！请手动以root权限执行以下命令：${NC}"
        echo -e "${YELLOW}sudo cp ${TEMP_FILE} ${RC_LOCAL_PATH} && sudo chmod +x ${RC_LOCAL_PATH}${NC}"
    fi
else
    echo -e "${GREEN}\n已生成修改后的文件：${TEMP_FILE}${NC}"
    echo -e "${YELLOW}如需手动应用，请以root权限执行：${NC}"
    echo -e "${YELLOW}sudo cp ${TEMP_FILE} ${RC_LOCAL_PATH} && sudo chmod +x ${RC_LOCAL_PATH}${NC}"
fi

# 清理临时文件
rm -f "${TEMP_FILE}" 2>/dev/null

# 显示完整应用命令
echo -e "\n${GREEN}完整应用命令：${NC}"
echo -e "${YELLOW}sudo cp ${TEMPLATE_FILE} ${RC_LOCAL_PATH} && sudo chmod +x ${RC_LOCAL_PATH}${NC}"
