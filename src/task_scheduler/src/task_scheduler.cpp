/**
 * @file task_scheduler.cpp
 * @brief 智能导览机器人 - 语音交互导航系统
 * 
 * 工作流程（更新）：
 * 1. 机器人启动，导航到走廊位置 (0.80, 0.231)
 * 2. 进入待机状态，等待人脸识别唤醒
 * 3. 识别到人脸后，播报欢迎语："你好，需要帮助吗？"
 * 4. 进入空闲状态，等待语音指令
 * 5. 用户说"参观"触发导览
 * 6. 机器人回应并开始导航到第一个目标点
 * 7. 到达目标点，播报展馆介绍
 * 8. 循环访问所有目标点
 * 9. 所有目标完成，发布充电指令
 * 10. 等待充电完成（超时则强制继续）
 * 11. 返回起点并结束
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
#include <mutex>

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
};

/**
 * @brief 任务调度器类
 * 实现语音交互导览机器人的完整状态机
 */
class TaskScheduler {
private:
    // ==================== 状态枚举 ====================
    enum class State {
        WAITING_START,      // 等待启动：系统初始化
        GOING_TO_LOBBY,     // 前往走廊位置
        WAITING_FACE,       // 等待人脸识别唤醒
        IDLE,               // 空闲状态：等待用户语音指令
        NAVIGATING,         // 导航中：机器人正在前往目标点
        WAITING_SPEECH,     // 语音播报中：已到达目标点，播放介绍
        CHARGING,           // 充电中：机器人正在充电
        RETURNING,          // 返回中：充电完成，返回起点
        TASK_COMPLETE,      // 任务完成：程序准备退出
        ERROR               // 错误状态：发生异常
    };

    // ==================== ROS通信成员 ====================
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Publisher charge_pub_;
    ros::Subscriber status_sub_;
    ros::Subscriber voice_sub_;
    ros::Subscriber face_sub_;          // 新增：人脸识别订阅者
    ros::Subscriber charge_complete_sub_;
    
    // ==================== 任务数据 ====================
    std::vector<Waypoint> route_;
    int current_index_ = 0;
    
    // ==================== 状态标志 ====================
    State current_state_ = State::WAITING_START;
    std::atomic<bool> speech_finished_{false};
    std::atomic<bool> voice_command_received_{false};
    std::atomic<bool> charge_completed_{false};
    std::atomic<bool> face_detected_{false};      // 新增：人脸检测标志
    bool all_done_ = false;
    
    // ==================== 互斥锁 ====================
    std::mutex state_mutex_;                      // 状态保护锁
    
    // ==================== 语音关键词 ====================
    const std::vector<std::string> start_keywords_ = {
        "参观"
    };
    
    // ==================== 系统参数 ====================
    const int NAV_TIMEOUT_SECONDS = 120;
    const double SPEECH_BUFFER_SECONDS = 0.5;
    const int LOOP_RATE_HZ = 10;
    const int IDLE_REMIND_INTERVAL = 30;
    const int CHARGE_TIMEOUT_SECONDS = 60;
    
    // ==================== 坐标常量 ====================
    const double LOBBY_X = 0.80;      // 走廊/大厅位置 X
    const double LOBBY_Y = 0.231;     // 走廊/大厅位置 Y
    const double START_X = 0.0;       // 起点/出发区 X
    const double START_Y = 0.0;       // 起点/出发区 Y
    
public:
    /**
     * @brief 构造函数
     */
    TaskScheduler() {
        // ----- 初始化发布者和订阅者 -----
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/nav_goal", 10);
        charge_pub_ = nh_.advertise<std_msgs::Bool>("/charge_command", 10);
        
        status_sub_ = nh_.subscribe("/nav_status", 10, 
                                   &TaskScheduler::statusCallback, this);
        voice_sub_ = nh_.subscribe("/voice_recognition", 10, 
                                  &TaskScheduler::voiceCallback, this);
        charge_complete_sub_ = nh_.subscribe("/charge_complete", 10,
                                            &TaskScheduler::chargeCompleteCallback, this);
        
        // ----- 新增：订阅人脸识别结果 -----
        face_sub_ = nh_.subscribe("/recognized_face", 10,
                                 &TaskScheduler::faceCallback, this);
        
        // ----- 加载参观路线 -----
        initRoute();
        
        // ----- 打印启动信息 -----
        ROS_INFO("🚀 智能导览机器人已启动");
        ROS_INFO("📍 共加载 %zu 个参观目标点", route_.size());
        ROS_INFO("📊 初始状态: %s", stateToString(current_state_).c_str());
        // ROS_INFO("📍 走廊位置: (%.2f, %.2f)", LOBBY_X, LOBBY_Y);
        // ROS_INFO("📍 出发区位置: (%.2f, %.2f)", START_X, START_Y);
    }
    
    /**
     * @brief 初始化参观路线
     */
    void initRoute() {
        route_ = {
            {
                "展厅A",
                2.490, 2.190,
                0.0,
                "这里是展厅A，主要展示智能机器人技术"
            },
            {
                "展厅B",
                2.474, 1.181,
                0.0,
                "这里是展厅B，介绍人工智能算法应用"
            },
            {
                "展厅C",
                2.476, 0.203,
                0.0,
                "这里是展厅C，展示自动驾驶模拟系统"
            }
        };
    }

    // ==================== 新增：人脸识别回调函数 ====================
    
    /**
     * @brief 人脸识别回调函数
     * 接收人脸识别结果，用于唤醒机器人
     * 
     * @param msg 人脸识别结果（格式："姓名|置信度"）
     */
    void faceCallback(const std_msgs::String::ConstPtr& msg) {
        // 只有在等待人脸状态才处理
        if (current_state_ != State::WAITING_FACE) {
            return;
        }
        
        std::string face_data = msg->data;
        ROS_INFO("👤 收到人脸识别结果: %s", face_data.c_str());
        
        // 解析姓名和置信度
        size_t sep = face_data.find('|');
        if (sep != std::string::npos) {
            std::string name = face_data.substr(0, sep);
            std::string confidence_str = face_data.substr(sep + 1);
            
            // 检查是否识别到已知人脸
            if (name != "Unknown") {
                ROS_INFO("✅ 识别到已知人脸: %s，机器人被唤醒！", name.c_str());
                
                // 原子操作：标记人脸检测
                face_detected_ = true;
                
                // 唤醒机器人：播报欢迎语
                speakSync("你好，需要帮助吗？");
                
                // 转换到空闲状态，等待语音指令
                transitionTo(State::IDLE);
                
                ROS_INFO("💬 机器人已唤醒，等待语音指令...");
                // ROS_INFO("💡 请说 '参观' 开始导览");
            } else {
                ROS_INFO("❌ 未知人脸，不唤醒机器人");
            }
        }
    }

    // ==================== 语音交互函数 ====================
    
    /**
     * @brief 语音识别回调函数
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
                
                // 启动导航任务
                startNavigation();
            }
        }
    }

    /**
     * @brief 检查文本是否包含关键词
     */
    bool containsKeyword(const std::string& text, 
                        const std::vector<std::string>& keywords) {
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
     * @brief 同步语音播报
     */
    void speakSync(const std::string& text) {
        ROS_INFO("🔊 语音播报: %s", text.c_str());
        
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
     * @brief 异步语音播报
     */
    void speakAsync(const std::string& text, const std::string& name) {
        speech_finished_ = false;
        
        ROS_INFO("🎤 启动语音播报: %s", name.c_str());
        
        std::thread([this, text, name]() {
            ROS_INFO("🔊 开始播报 %s 的介绍", name.c_str());
            ROS_INFO("📝 内容: %s", text.c_str());
            
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
            
            speech_finished_ = true;
            
        }).detach();
    }

    // ==================== 导航相关函数 ====================
    
    /**
     * @brief 导航状态回调函数
     */
    void statusCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (current_state_ != State::NAVIGATING && 
            current_state_ != State::GOING_TO_LOBBY &&
            current_state_ != State::RETURNING) {
            ROS_WARN("⚠️ 当前状态不是导航中，忽略导航状态消息");
            return;
        }
        
        ROS_INFO("📩 收到导航反馈: %s", msg->data ? "✅ 成功" : "❌ 失败");
        
        if (msg->data) {
            // ===== 导航成功 =====
            if (current_state_ == State::GOING_TO_LOBBY) {
                // 到达走廊，等待人脸识别
                ROS_INFO("✅ 到达走廊位置，进入等待人脸识别状态");
                transitionTo(State::WAITING_FACE);
                ROS_INFO("👤 等待人脸识别唤醒...");
            } else if (current_state_ == State::NAVIGATING) {
                handleNavigationSuccess();
            } else if (current_state_ == State::RETURNING) {
                // 返回起点成功
                ROS_INFO("✅ 已返回出发区");
                transitionTo(State::TASK_COMPLETE);
                all_done_ = true;
            }
        } else {
            // ===== 导航失败 =====
            if (current_state_ == State::GOING_TO_LOBBY) {
                ROS_ERROR("❌ 前往走廊失败，尝试重试...");
                // 重试一次
                ros::Duration(1.0).sleep();
                sendLobbyGoal();
            } else if (current_state_ == State::NAVIGATING) {
                handleNavigationFailure();
            } else if (current_state_ == State::RETURNING) {
                ROS_ERROR("❌ 返回起点失败，强制结束");
                transitionTo(State::TASK_COMPLETE);
                all_done_ = true;
            }
        }
    }

    /**
     * @brief 处理导航成功
     */
    void handleNavigationSuccess() {
        if (current_index_ >= route_.size()) {
            ROS_ERROR("❌ 索引越界: %d >= %zu", 
                     current_index_, route_.size());
            transitionTo(State::ERROR);
            return;
        }
        
        ROS_INFO("✅ 成功到达目标点: %s (索引: %d)", 
                 route_[current_index_].name.c_str(), current_index_);
        
        std::string description = route_[current_index_].description;
        std::string name = route_[current_index_].name;
        
        transitionTo(State::WAITING_SPEECH);
        speakAsync(description, name);
    }

    /**
     * @brief 处理导航失败
     */
    void handleNavigationFailure() {
        ROS_ERROR("❌ 导航到 %s 失败！", route_[current_index_].name.c_str());
        
        if (current_index_ < route_.size() - 1) {
            ROS_WARN("🔄 跳过失败的目标，继续下一个");
            current_index_++;
            
            if (current_index_ < route_.size()) {
                transitionTo(State::NAVIGATING);
                sendNextGoal();
            } else {
                transitionTo(State::TASK_COMPLETE);
                handleTaskComplete();
            }
        } else {
            ROS_WARN("⚠️ 最后一个目标失败，结束任务");
            transitionTo(State::TASK_COMPLETE);
            handleTaskComplete();
        }
    }

    /**
     * @brief 发送导航到走廊的目标
     */
    void sendLobbyGoal() {
        ROS_INFO("📍 导航到走廊位置 (%.2f, %.2f)", LOBBY_X, LOBBY_Y);
        
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = LOBBY_X;
        goal.pose.position.y = LOBBY_Y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        
        goal_pub_.publish(goal);
        ROS_INFO("📤 走廊目标已发布到 /nav_goal");
    }

    /**
     * @brief 发送下一个导航目标
     */
    void sendNextGoal() {
        if (current_index_ >= route_.size()) {
            ROS_ERROR("❌ 索引越界，无法发送目标");
            transitionTo(State::ERROR);
            return;
        }
        
        if (current_state_ != State::NAVIGATING) {
            transitionTo(State::NAVIGATING);
        }
        
        Waypoint& wp = route_[current_index_];
        ROS_INFO("🔄 发送第 %d/%zu 个目标: %s (%.2f, %.2f)", 
                 current_index_ + 1, route_.size(), 
                 wp.name.c_str(), wp.x, wp.y);
        
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = sin(wp.angle / 2.0);
        goal.pose.orientation.w = cos(wp.angle / 2.0);
        
        goal_pub_.publish(goal);
        ROS_INFO("📤 目标已发布到 /nav_goal，等待导航反馈...");
    }

    /**
     * @brief 启动导航任务
     */
    void startNavigation() {
        ROS_INFO("🚀 开始导览任务");
        current_index_ = 0;
        transitionTo(State::NAVIGATING);
        sendNextGoal();
    }

    // ==================== 充电相关函数 ====================
    
    /**
     * @brief 充电完成回调函数
     */
    void chargeCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && current_state_ == State::CHARGING) {
            ROS_INFO("🔋 收到充电完成消息！");
            charge_completed_ = true;
        }
    }

    /**
     * @brief 发布充电指令
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
     */
    void handleTaskComplete() {
        ROS_INFO("🏁 所有目标点已参观完成！");
        
        speakSync("参观结束，现在去充电");
        sendChargeCommand();
        
        transitionTo(State::CHARGING);
        charge_completed_ = false;
        
        int charge_timeout_counter = 0;
        ros::Rate loop_rate(LOOP_RATE_HZ);
        
        ROS_INFO("⏳ 等待充电完成...");
        
        while (ros::ok() && !charge_completed_) {
            ros::spinOnce();
            
            charge_timeout_counter++;
            if (charge_timeout_counter > CHARGE_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                ROS_ERROR("⏰ 充电超时！(%d秒)，强制返回起点", CHARGE_TIMEOUT_SECONDS);
                break;
            }
            
            if (charge_timeout_counter % (30 * LOOP_RATE_HZ) == 0) {
                ROS_INFO("⏳ 等待充电完成... 已等待 %d 秒", 
                         charge_timeout_counter / LOOP_RATE_HZ);
            }
            
            loop_rate.sleep();
        }
        
        if (charge_completed_) {
            ROS_INFO("✅ 充电完成，开始返回起点");
        }
        
        returnToStart();
    }

    /**
     * @brief 返回起点
     */
    void returnToStart() {
        ROS_INFO("🏠 返回出发区 (%.2f, %.2f)", START_X, START_Y);
        
        geometry_msgs::PoseStamped home;
        home.header.frame_id = "map";
        home.header.stamp = ros::Time::now();
        home.pose.position.x = START_X;
        home.pose.position.y = START_Y;
        home.pose.position.z = 0.0;
        home.pose.orientation.w = 1.0;
        
        goal_pub_.publish(home);
        ROS_INFO("📤 已发送返回出发区命令");
        
        transitionTo(State::RETURNING);
        
        // 等待导航完成（由statusCallback处理）
        ROS_INFO("🔄 正在返回出发区...");
    }

    /**
     * @brief 检查语音播报状态并推进流程
     */
    void checkSpeechAndProceed() {
        if (current_state_ != State::WAITING_SPEECH) {
            return;
        }
        
        if (speech_finished_.load()) {
            ROS_INFO("✅ 检测到语音播报完成");
            speech_finished_ = false;
            ros::Duration(SPEECH_BUFFER_SECONDS).sleep();
            
            current_index_++;
            ROS_INFO("📊 前进到下一个目标，索引: %d", current_index_);
            
            if (current_index_ >= route_.size()) {
                transitionTo(State::TASK_COMPLETE);
                handleTaskComplete();
            } else {
                transitionTo(State::NAVIGATING);
                sendNextGoal();
            }
        }
    }

    /**
     * @brief 处理错误状态
     */
    void handleError() {
        ROS_ERROR("🚨 系统进入错误状态！");
        
        if (current_index_ < route_.size()) {
            ROS_WARN("🔄 跳过当前目标: %s", 
                    route_[current_index_].name.c_str());
            current_index_++;
            
            if (current_index_ < route_.size()) {
                transitionTo(State::NAVIGATING);
                sendNextGoal();
            } else {
                transitionTo(State::TASK_COMPLETE);
                handleTaskComplete();
            }
        } else {
            transitionTo(State::TASK_COMPLETE);
            all_done_ = true;
        }
    }

    // ==================== 状态管理函数 ====================
    
    /**
     * @brief 状态转换函数
     */
    void transitionTo(State new_state) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
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
            case State::WAITING_START:  return "等待启动";
            case State::GOING_TO_LOBBY: return "前往走廊";
            case State::WAITING_FACE:   return "等待人脸识别";
            case State::IDLE:           return "空闲(等待指令)";
            case State::NAVIGATING:     return "导航中";
            case State::WAITING_SPEECH: return "语音播报中";
            case State::CHARGING:       return "充电中";
            case State::RETURNING:      return "返回起点中";
            case State::TASK_COMPLETE:  return "任务完成";
            case State::ERROR:          return "错误状态";
            default:                    return "未知状态";
        }
    }

    // ==================== 主循环函数 ====================
    
    /**
     * @brief 主循环启动函数
     */
    void start() {
        ROS_INFO("▶️  开始执行任务调度");
        
        ros::Duration(1.0).sleep();
        
        // ===== 步骤1: 导航到走廊 =====
        ROS_INFO("📍 开始导航到走廊位置");
        transitionTo(State::GOING_TO_LOBBY);
        sendLobbyGoal();
        
        // ===== 步骤2: 主循环 =====
        ros::Rate loop_rate(LOOP_RATE_HZ);
        int timeout_counter = 0;
        int waiting_face_counter = 0;
        
        while (ros::ok() && !all_done_) {
            ros::spinOnce();
            
            switch (current_state_) {
                case State::GOING_TO_LOBBY:
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                        ROS_ERROR("⏰ 前往走廊超时！(%d秒)", NAV_TIMEOUT_SECONDS);
                        timeout_counter = 0;
                        // 重试一次
                        sendLobbyGoal();
                    }
                    break;
                    
                case State::WAITING_FACE:
                    waiting_face_counter++;
                    if (waiting_face_counter % (10 * LOOP_RATE_HZ) == 0) {
                        ROS_INFO("👤 等待人脸识别唤醒...");
                    }
                    break;
                    
                case State::IDLE:
                    // 空闲状态：定期提醒
                    timeout_counter++;
                    if (timeout_counter % (IDLE_REMIND_INTERVAL * LOOP_RATE_HZ) == 0) {
                        ROS_INFO("💬 请说 '参观' 开始导览");
                    }
                    break;
                    
                case State::NAVIGATING:
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ) {
                        ROS_ERROR("⏰ 导航超时！(%d秒)", NAV_TIMEOUT_SECONDS);
                        timeout_counter = 0;
                        handleNavigationFailure();
                    }
                    break;
                    
                case State::WAITING_SPEECH:
                    checkSpeechAndProceed();
                    break;
                    
                case State::CHARGING:
                    // 充电状态在 handleTaskComplete() 中处理
                    break;
                    
                case State::RETURNING:
                    ROS_INFO_THROTTLE(5, "🔄 机器人正在返回出发区...");
                    break;
                    
                case State::TASK_COMPLETE:
                    ROS_INFO("✅ 任务已完成，调度器准备退出");
                    all_done_ = true;
                    break;
                    
                case State::ERROR:
                    ROS_WARN_THROTTLE(5, "⛔ 系统处于错误状态");
                    break;
                    
                case State::WAITING_START:
                default:
                    break;
            }
            
            loop_rate.sleep();
        }
        
        ROS_INFO("🛑 调度器结束运行");
        ROS_INFO("  导览任务完成，感谢使用！");
    }
};

// ==================== 主函数 ====================

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    
    ros::init(argc, argv, "task_scheduler");
    
    ROS_INFO("  智能导览机器人系统 v2.0");
    ROS_INFO("  1. 机器人将前往走廊位置");
    ROS_INFO("  2. 等待人脸识别唤醒");
    ROS_INFO("  3. 说 '参观' 开始导览");
    ROS_INFO("  4. 依次参观展厅A、B、C");
    ROS_INFO("  5. 充电后返回出发区");
    
    TaskScheduler scheduler;
    scheduler.start();
    
    ROS_INFO("  程序正常退出");
    
    return 0;
}