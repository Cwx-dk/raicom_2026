/**
 * @file task_scheduler.cpp
 * @brief 智能导览机器人 - 语音交互导航系统
 * 
 * 工作流程：
 * 1. 机器人导航到第一个目标点（展厅A），不播报介绍
 * 2. 到达后播报欢迎语，进入空闲状态，等待语音指令
 * 3. 用户说"参观"触发导览
 * 4. 机器人回应并开始导航到第二个目标点（展厅B）
 * 5. 到达目标点，播报展馆介绍
 * 6. 循环访问所有目标点
 * 7. 所有目标完成，发布充电指令
 * 8. 等待充电完成
 * 9. 返回起点并结束
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <cctype>

/**
 * @brief 目标点结构体
 * 存储每个参观点的信息
 */
struct Waypoint {
    std::string name;           // 目标点名称（如"展厅A"）
    double x;                   // 地图X坐标（米）
    double y;                   // 地图Y坐标（米）
    double angle;               // 机器人朝向角度（弧度）
    std::string description;    // 该地点的语音介绍文本
    bool need_speech;           // 是否需要播报介绍（第一个点为false）
};

/**
 * @brief 任务调度器类
 * 实现语音交互导览机器人的完整状态机
 */
class TaskScheduler {
private:
    // ==================== 状态枚举 ====================
    /**
     * @brief 系统运行状态
     */
    enum class State {
        WAITING_START,      // 等待启动：系统初始化
        NAVIGATING_TO_FIRST,// 导航到第一个目标点（不播报）
        IDLE,               // 空闲状态：等待用户语音指令
        NAVIGATING,         // 导航中：机器人正在前往目标点
        WAITING_SPEECH,     // 语音播报中：已到达目标点，播放介绍
        CHARGING,           // 充电中：机器人正在充电
        RETURNING,          // 返回中：充电完成，返回起点
        TASK_COMPLETE,      // 任务完成：程序准备退出
        ERROR              // 错误状态：发生异常
    };

    // ==================== ROS通信成员 ====================
    ros::NodeHandle nh_;                    // ROS节点句柄
    ros::Publisher goal_pub_;              // 导航目标发布者（发布到/nav_goal）
    ros::Publisher charge_pub_;            // 充电指令发布者（发布到/charge_command）
    ros::Subscriber status_sub_;           // 导航状态订阅者（订阅/nav_status）
    ros::Subscriber voice_sub_;            // 语音识别订阅者（订阅/voice_recognition）
    ros::Subscriber charge_complete_sub_;  // 充电完成订阅者（订阅/charge_complete）
    
    // ==================== 任务数据 ====================
    std::vector<Waypoint> route_;          // 参观路线数据
    int current_index_ = 0;                // 当前访问的目标点索引（从0开始）
    bool first_goal_reached_ = false;      // 第一个目标点是否已到达
    
    // ==================== 状态标志 ====================
    State current_state_ = State::WAITING_START;  // 当前系统状态
    std::atomic<bool> speech_finished_{false};    // 语音播报完成标志（线程安全）
    std::atomic<bool> voice_command_received_{false}; // 是否收到语音指令
    std::atomic<bool> charge_completed_{false};   // 充电是否完成
    bool all_done_ = false;                       // 所有任务是否完成
    
    // ==================== 语音关键词 ====================
    // 用户可以说这些词来启动导览（支持中英文）
    const std::vector<std::string> start_keywords_ = {
        "参观"
    };
    
    // ==================== 系统参数 ====================
    const int NAV_TIMEOUT_SECONDS = 120;          // 导航超时时间（秒）
    const double SPEECH_BUFFER_SECONDS = 0.5;     // 语音播报后缓冲时间
    const int LOOP_RATE_HZ = 10;                  // 主循环频率（Hz）
    const int IDLE_REMIND_INTERVAL = 30;          // 空闲提醒间隔（秒）
    const int CHARGE_TIMEOUT_SECONDS = 60;       // 充电超时时间（秒）
    
public:
    /**
     * @brief 构造函数
     * 初始化ROS通信接口和路线数据
     */
    TaskScheduler() {
        // ----- 初始化发布者和订阅者 -----
        // 发布导航目标话题
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/nav_goal", 10);
        
        // 发布充电指令话题
        charge_pub_ = nh_.advertise<std_msgs::Bool>("/charge_command", 10);
        
        // 订阅导航状态反馈
        status_sub_ = nh_.subscribe("/nav_status", 10, 
                                   &TaskScheduler::statusCallback, this);
        
        // 订阅语音识别结果（需要外部语音识别节点发布）
        voice_sub_ = nh_.subscribe("/voice_recognition", 10, 
                                  &TaskScheduler::voiceCallback, this);
        
        // 订阅充电完成消息
        charge_complete_sub_ = nh_.subscribe("/charge_complete", 10,
                                            &TaskScheduler::chargeCompleteCallback, this);
        
        // ----- 加载参观路线 -----
        initRoute();
        
        // ----- 打印启动信息 -----
        ROS_INFO("🚀 智能导览机器人已启动");
        ROS_INFO("📍 共加载 %zu 个参观目标点", route_.size());
        ROS_INFO("📊 初始状态: %s", stateToString(current_state_).c_str());
    }
    
    /**
     * @brief 初始化参观路线
     * 定义机器人需要访问的所有目标点
     * 第一个目标点（展厅A）不播报介绍，只作为起始位置
     */
    void initRoute() {
        route_ = {
            // 第一个目标点（起始点，不播报介绍）
            {
                "展厅A",                    // 名称
                2.490, 2.190,              // 地图坐标 (x, y)
                0.0,                       // 朝向角度（0度）
                "",                        // 无介绍文本
                false                      // 不需要播报
            },
            // 第二个目标点（第一个需要播报介绍的点）
            {
                "展厅B",
                2.474, 1.181,
                0.0,
                "这里是展厅B，主要展示智能机器人技术",
                true
            },
            // 第三个目标点
            {
                "展厅C",
                2.476, 0.203,
                0.0,
                "这里是展厅C，介绍人工智能算法",
                true
            },
            {
                "展厅D",
                1.500, 0.500,
                0.0,
                "这里是展厅D，展示自动驾驶模拟系统",
                true
            },
            // 可以继续添加更多目标点...
        };
    }

    // ==================== 语音交互函数 ====================
    
    /**
     * @brief 语音识别回调函数
     * 处理用户的语音指令
     * 
     * @param msg 语音识别结果文本
     */
    void voiceCallback(const std_msgs::String::ConstPtr& msg) {
        std::string recognized_text = msg->data;
        ROS_INFO("🎤 识别到语音: %s", recognized_text.c_str());
        
        // ----- 在空闲状态下检测启动指令 -----
        if (current_state_ == State::IDLE) {
            if (containsKeyword(recognized_text, start_keywords_)) {
                ROS_INFO("✅ 检测到启动指令: '%s'", recognized_text.c_str());
                voice_command_received_ = true;
                
                // 语音回应：告诉用户即将开始
                speakSync("好的，请跟我来");
                
                // 启动导览任务（从第二个目标点开始）
                startTour();
            }
        }
    }

    /**
     * @brief 检查文本是否包含关键词（不区分大小写）
     * 
     * @param text 待检查的文本
     * @param keywords 关键词列表
     * @return true 包含关键词
     * @return false 不包含关键词
     */
    bool containsKeyword(const std::string& text, 
                        const std::vector<std::string>& keywords) {
        // 转换为小写以支持不区分大小写匹配
        std::string lower_text = text;
        std::transform(lower_text.begin(), lower_text.end(), 
                      lower_text.begin(), ::tolower);
        
        for (const auto& keyword : keywords) {
            std::string lower_keyword = keyword;
            std::transform(lower_keyword.begin(), lower_keyword.end(), 
                          lower_keyword.begin(), ::tolower);
            
            if (lower_text.find(lower_keyword) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 同步语音播报（阻塞函数）
     * 等待语音播报完成后才返回
     * 
     * @param text 要播报的文本
     */
    void speakSync(const std::string& text) {
        if (text.empty()) {
            return;
        }
        ROS_INFO("🔊 语音播报: %s", text.c_str());
        
        // 使用Microsoft Edge TTS进行高质量语音合成
        // 音色：zh-CN-YunxiNeural（中文男声，自然流畅）
        std::string cmd = "/home/reicom2025/.local/bin/edge-playback "
                         "--voice zh-CN-YunxiNeural "
                         "--text \"" + text + "\" "
                         "2>/dev/null";
        
        int ret = system(cmd.c_str());
        
        if (ret != 0) {
            ROS_WARN("⚠️ 语音播报异常，返回码: %d", ret);
        }
    }

    /**
     * @brief 异步语音播报（非阻塞）
     * 在新线程中播报，不阻塞主流程
     * 
     * @param text 要播报的文本
     * @param name 目标点名称（用于日志）
     */
    void speakAsync(const std::string& text, const std::string& name) {
        if (text.empty()) {
            speech_finished_ = true;
            return;
        }
        
        // 重置完成标志
        speech_finished_ = false;
        
        ROS_INFO("🎤 启动语音播报: %s", name.c_str());
        
        // 创建新线程执行语音播报
        std::thread([this, text, name]() {
            ROS_INFO("🔊 开始播报 %s 的介绍", name.c_str());
            ROS_INFO("📝 内容: %s", text.c_str());
            
            // 构建语音合成命令
            std::string cmd = "/home/reicom2025/.local/bin/edge-playback "
                             "--voice zh-CN-YunxiNeural "
                             "--text \"" + text + "\" "
                             "2>/dev/null";
            
            int ret = system(cmd.c_str());
            
            if (ret == 0) {
                ROS_INFO("✅ 语音播报完成: %s", name.c_str());
            } else {
                ROS_WARN("⚠️ 语音播报异常，返回码: %d", ret);
            }
            
            // 原子操作：标记语音完成
            speech_finished_ = true;
            
        }).detach();  // 分离线程，让它在后台运行
    }

    // ==================== 导航相关函数 ====================
    
    /**
     * @brief 导航状态回调函数
     * 处理导航系统返回的成功/失败状态
     * 
     * @param msg 导航状态消息 (true=成功, false=失败)
     */
    void statusCallback(const std_msgs::Bool::ConstPtr& msg) {
        // 检查当前状态：只有在导航状态才处理
        if (current_state_ != State::NAVIGATING_TO_FIRST && 
            current_state_ != State::NAVIGATING) {
            ROS_WARN("⚠️ 当前状态不是导航中(%s)，忽略导航状态消息", 
                     stateToString(current_state_).c_str());
            return;
        }
        
        ROS_INFO("📩 收到导航反馈: %s", msg->data ? "✅ 成功" : "❌ 失败");
        
        if (msg->data) {
            // ===== 导航成功 =====
            if (current_state_ == State::NAVIGATING_TO_FIRST) {
                // 到达第一个目标点
                handleFirstGoalReached();
            } else {
                handleNavigationSuccess();
            }
        } else {
            // ===== 导航失败 =====
            handleNavigationFailure();
        }
    }

    /**
     * @brief 处理第一个目标点到达
     * 状态转换: NAVIGATING_TO_FIRST -> IDLE
     */
    void handleFirstGoalReached() {
        ROS_INFO("✅ 成功到达起始点: %s", route_[0].name.c_str());
        first_goal_reached_ = true;
        
        // 播报欢迎语
        speakSync("你好，需要帮助吗？");
        
        // 进入空闲状态，等待语音指令
        transitionTo(State::IDLE);
        ROS_INFO("💬 机器人已就绪，等待语音指令...");
    }

    /**
     * @brief 处理导航成功
     * 状态转换: NAVIGATING -> WAITING_SPEECH
     */
    void handleNavigationSuccess() {
        // 安全检查：确保索引有效
        if (current_index_ >= route_.size()) {
            ROS_ERROR("❌ 索引越界: %d >= %zu", 
                     current_index_, route_.size());
            transitionTo(State::ERROR);
            return;
        }
        
        ROS_INFO("✅ 成功到达目标点: %s (索引: %d)", 
                 route_[current_index_].name.c_str(), current_index_);
        
        // 获取当前目标点的信息
        std::string description = route_[current_index_].description;
        std::string name = route_[current_index_].name;
        bool need_speech = route_[current_index_].need_speech;
        
        // 如果需要播报，进入语音播报状态
        if (need_speech && !description.empty()) {
            transitionTo(State::WAITING_SPEECH);
            speakAsync(description, name);
        } else {
            // 不需要播报，直接前进到下一个目标
            ROS_INFO("⏭️ 该目标点无需播报，直接前进");
            proceedToNextGoal();
        }
    }

    /**
     * @brief 处理导航失败
     * 策略：跳过失败的目标点，继续下一个
     */
    void handleNavigationFailure() {
        if (current_state_ == State::NAVIGATING_TO_FIRST) {
            ROS_ERROR("❌ 导航到起始点 %s 失败！", route_[0].name.c_str());
            // 尝试继续，跳过起始点
            first_goal_reached_ = true;
            transitionTo(State::IDLE);
            speakSync("定位完成，欢迎参观");
            return;
        }
        
        ROS_ERROR("❌ 导航到 %s 失败！", route_[current_index_].name.c_str());
        
        // 跳过当前失败的目标，尝试下一个
        proceedToNextGoal();
    }

    /**
     * @brief 前进到下一个目标点
     */
    void proceedToNextGoal() {
        current_index_++;
        ROS_INFO("📊 前进到下一个目标，索引: %d", current_index_);
        
        // 判断是否所有目标都已访问
        if (current_index_ >= route_.size()) {
            // 所有目标完成 -> 进入充电流程
            transitionTo(State::TASK_COMPLETE);
            handleTaskComplete();
        } else {
            // 继续下一个目标
            transitionTo(State::NAVIGATING);
            sendNextGoal();
        }
    }

    /**
     * @brief 发送导航目标
     * @param index 目标点索引
     */
    void sendGoal(int index) {
        // 安全检查
        if (index >= route_.size()) {
            ROS_ERROR("❌ 索引越界，无法发送目标");
            transitionTo(State::ERROR);
            return;
        }
        
        Waypoint& wp = route_[index];
        ROS_INFO("🔄 发送第 %d/%zu 个目标: %s (%.2f, %.2f)", 
                 index + 1, route_.size(), 
                 wp.name.c_str(), wp.x, wp.y);
        
        // ----- 构造导航目标消息 -----
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";               // 使用地图坐标系
        goal.header.stamp = ros::Time::now();       // 当前时间戳
        
        // 设置位置坐标
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = 0.0;                 // 地面高度
        
        // 将角度转换为四元数（绕Z轴旋转）
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = sin(wp.angle / 2.0);
        goal.pose.orientation.w = cos(wp.angle / 2.0);
        
        // 发布导航目标
        goal_pub_.publish(goal);
        ROS_INFO("📤 目标已发布到 /nav_goal，等待导航反馈...");
    }

    /**
     * @brief 发送下一个导航目标
     * 使用当前索引 current_index_
     */
    void sendNextGoal() {
        sendGoal(current_index_);
    }

    /**
     * @brief 导航到第一个目标点（起始点）
     */
    void navigateToFirstGoal() {
        ROS_INFO("🚀 导航到起始点: %s", route_[0].name.c_str());
        current_index_ = 0;
        transitionTo(State::NAVIGATING_TO_FIRST);
        sendGoal(0);
    }

    /**
     * @brief 启动导览任务
     * 从第二个目标点开始（索引1）
     */
    void startTour() {
        ROS_INFO("🚀 开始导览任务");
        current_index_ = 1;  // 从第二个目标点开始
        
        if (current_index_ >= route_.size()) {
            ROS_WARN("⚠️ 没有需要参观的目标点");
            transitionTo(State::TASK_COMPLETE);
            handleTaskComplete();
            return;
        }
        
        transitionTo(State::NAVIGATING);
        sendNextGoal();
    }

    // ==================== 充电相关函数 ====================
    
    /**
     * @brief 充电完成回调函数
     * 收到充电完成消息后，触发返回起点
     */
    void chargeCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && current_state_ == State::CHARGING) {
            ROS_INFO("🔋 收到充电完成消息！");
            charge_completed_ = true;
        }
    }

    /**
     * @brief 发布充电指令
     * 通知 auto_charge 节点开始充电
     */
    void sendChargeCommand() {
        ROS_INFO("🔋 发布充电指令...");
        std_msgs::Bool charge_cmd;
        charge_cmd.data = true;
        charge_pub_.publish(charge_cmd);
        ROS_INFO("📤 充电指令已发布到 /charge_command");
    }

    /**
     * @brief 处理所有目标完成
     * 状态转换: -> CHARGING -> (等待充电完成) -> RETURNING -> TASK_COMPLETE
     */
    void handleTaskComplete() {
        ROS_INFO("🏁 所有目标点已参观完成！");
        
        // 发送充电指令
        sendChargeCommand();
        
        // 进入充电状态
        transitionTo(State::CHARGING);
        charge_completed_ = false;
        
        // 设置充电超时计时器
        int charge_timeout_counter = 0;
        ros::Rate loop_rate(LOOP_RATE_HZ);
        
        ROS_INFO("⏳ 等待充电完成...");
        
        // 等待充电完成或超时
        while (ros::ok() && !charge_completed_) {
            ros::spinOnce();
            
            charge_timeout_counter++;
            if (charge_timeout_counter > CHARGE_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                ROS_ERROR("⏰ 充电超时！(%d秒)，跳过充电直接返回起点", CHARGE_TIMEOUT_SECONDS);
                break;
            }
            
            // 每30秒打印一次等待信息
            if (charge_timeout_counter % (30 * LOOP_RATE_HZ) == 0) {
                ROS_INFO("⏳ 等待充电完成... 已等待 %d 秒", 
                         charge_timeout_counter / LOOP_RATE_HZ);
            }
            
            loop_rate.sleep();
        }
        
        if (charge_completed_) {
            ROS_INFO("✅ 充电完成，开始返回起点");
        }
        
        // 返回起点
        returnToStart();
    }

    /**
     * @brief 返回起点
     * 发送返回起点的导航命令
     */
    void returnToStart() {
        ROS_INFO("🏠 发送返回起点命令...");
        
        // 构造起点位置（假设起点在原点）
        geometry_msgs::PoseStamped home;
        home.header.frame_id = "map";
        home.header.stamp = ros::Time::now();
        home.pose.position.x = 0.0;
        home.pose.position.y = 0.0;
        home.pose.position.z = 0.0;
        home.pose.orientation.w = 1.0;  // 朝向角为0
        
        // 发布起点目标
        goal_pub_.publish(home);
        ROS_INFO("📤 已发送返回起点命令");
        
        // 转换状态
        transitionTo(State::RETURNING);
        
        // 简单等待一下，让机器人开始返回
        ros::Duration(2.0).sleep();
        
        // 直接结束程序（不等待到达起点）
        ROS_INFO("💡 机器人正在返回起点，调度程序结束");
        all_done_ = true;
        transitionTo(State::TASK_COMPLETE);
    }

    /**
     * @brief 检查语音播报状态并推进流程
     * 在主循环中定期调用
     */
    void checkSpeechAndProceed() {
        // 只有在语音播报状态才检查
        if (current_state_ != State::WAITING_SPEECH) {
            return;
        }
        
        // 检查语音是否播报完成
        if (speech_finished_.load()) {
            ROS_INFO("✅ 检测到语音播报完成");
            
            // 重置标志
            speech_finished_ = false;
            
            // 短暂缓冲，确保语音完全播放完毕
            ros::Duration(SPEECH_BUFFER_SECONDS).sleep();
            
            // 前进到下一个目标点
            proceedToNextGoal();
        }
    }

    /**
     * @brief 处理错误状态
     * 尝试恢复或安全退出
     */
    void handleError() {
        ROS_ERROR("🚨 系统进入错误状态！");
        
        // 错误恢复策略：跳过当前目标
        if (current_index_ < route_.size()) {
            ROS_WARN("🔄 跳过当前目标: %s", 
                    route_[current_index_].name.c_str());
            proceedToNextGoal();
        } else {
            // 无法恢复，安全结束
            transitionTo(State::TASK_COMPLETE);
            all_done_ = true;
        }
    }

    // ==================== 状态管理函数 ====================
    
    /**
     * @brief 状态转换函数
     * 统一管理状态转换并记录日志
     */
    void transitionTo(State new_state) {
        if (current_state_ == new_state) {
            return;
        }
        
        ROS_INFO("🔄 状态转换: %s -> %s", 
                 stateToString(current_state_).c_str(),
                 stateToString(new_state).c_str());
        
        current_state_ = new_state;
    }

    /**
     * @brief 将状态枚举转换为可读字符串
     */
    std::string stateToString(State state) {
        switch(state) {
            case State::WAITING_START:      return "等待启动";
            case State::NAVIGATING_TO_FIRST: return "导航到起始点";
            case State::IDLE:               return "空闲(等待指令)";
            case State::NAVIGATING:         return "导航中";
            case State::WAITING_SPEECH:     return "语音播报中";
            case State::CHARGING:           return "充电中";
            case State::RETURNING:          return "返回起点中";
            case State::TASK_COMPLETE:      return "任务完成";
            case State::ERROR:              return "错误状态";
            default:                        return "未知状态";
        }
    }

    // ==================== 主循环函数 ====================
    
    /**
     * @brief 主循环启动函数
     * 包含完整的状态机逻辑
     */
    void start() {
        ROS_INFO("▶️  开始执行任务调度");
        
        // 等待订阅者建立连接
        ros::Duration(1.0).sleep();
        
        // ===== 步骤1: 导航到第一个目标点 =====
        navigateToFirstGoal();
        
        // ===== 步骤2: 主循环 =====
        ros::Rate loop_rate(LOOP_RATE_HZ);  // 设置循环频率
        int timeout_counter = 0;
        int idle_counter = 0;
        
        while (ros::ok() && !all_done_) {
            // 处理所有ROS回调
            ros::spinOnce();
            
            // ----- 根据当前状态执行相应操作 -----
            switch (current_state_) {
                case State::NAVIGATING_TO_FIRST:
                    // 等待到达第一个目标点
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                        ROS_ERROR("⏰ 导航到起始点超时！(%d秒)", NAV_TIMEOUT_SECONDS);
                        timeout_counter = 0;
                        // 超时也继续，直接进入空闲状态
                        first_goal_reached_ = true;
                        transitionTo(State::IDLE);
                        speakSync("定位完成，欢迎参观");
                    }
                    break;
                    
                case State::IDLE:
                    // 空闲状态：定期提醒用户
                    idle_counter++;
                    if (idle_counter % (IDLE_REMIND_INTERVAL * LOOP_RATE_HZ) == 0) {
                        // 每30秒提醒一次
                        if (idle_counter % (3 * IDLE_REMIND_INTERVAL * LOOP_RATE_HZ) == 0) {
                            speakAsync("请说参观开始导览", "提醒");
                        }
                    }
                    break;
                    
                case State::NAVIGATING:
                    // 导航状态：检测超时
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                        ROS_ERROR("⏰ 导航超时！(%d秒)", NAV_TIMEOUT_SECONDS);
                        timeout_counter = 0;
                        handleNavigationFailure();
                    }
                    break;
                    
                case State::WAITING_SPEECH:
                    // 语音播报状态：检查是否完成
                    checkSpeechAndProceed();
                    break;
                    
                case State::CHARGING:
                    // 充电状态：在 handleTaskComplete() 中处理
                    break;
                    
                case State::RETURNING:
                    // 返回状态：已经发送返回命令，等待退出
                    ROS_INFO_THROTTLE(5, "🔄 机器人正在返回起点...");
                    break;
                    
                case State::TASK_COMPLETE:
                    // 任务完成状态：准备退出
                    ROS_INFO("✅ 任务已完成，调度器准备退出");
                    all_done_ = true;
                    break;
                    
                case State::ERROR:
                    // 错误状态
                    ROS_WARN_THROTTLE(5, "⛔ 系统处于错误状态");
                    break;
                    
                case State::WAITING_START:
                default:
                    ROS_WARN_THROTTLE(5, "⚠️ 未预期的状态: %s", 
                                      stateToString(current_state_).c_str());
                    break;
            }
            
            // 按照设定频率睡眠
            loop_rate.sleep();
        }
        
        // ===== 步骤3: 结束 =====
        ROS_INFO("🛑 调度器结束运行");
        ROS_INFO("  导览任务完成，感谢使用！");
    }
};

// ==================== 主函数 ====================

/**
 * @brief 程序入口
 */
int main(int argc, char** argv) {
    // 设置locale以支持中文输出
    setlocale(LC_CTYPE, "zh_CN.utf8");
    
    // 初始化ROS节点
    ros::init(argc, argv, "task_scheduler");
    
    // 打印启动信息
    ROS_INFO("  智能导览机器人系统 v1.0");
    
    // 创建调度器实例
    TaskScheduler scheduler;
    
    // 启动调度器（阻塞直到任务完成）
    scheduler.start();
    
    ROS_INFO("  程序正常退出");
    
    return 0;
}