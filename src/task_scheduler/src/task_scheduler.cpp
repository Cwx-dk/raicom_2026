/**
 * @file task_scheduler.cpp
 * @brief 智能导览机器人 - 语音交互导航系统
 * @version 2.4 - 使用auto_charge节点处理充电
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
#include <cmath>
#include <clocale>


struct Waypoint
{
    std::string name;
    double x;
    double y;
    double angle;
    std::string description;
};


class TaskScheduler
{

private:

    // =========================================================
    // 状态
    // =========================================================

    enum class State
    {
        WAITING_START,
        GOING_TO_LOBBY,
        WAITING_FACE,
        IDLE,
        NAVIGATING,
        WAITING_SPEECH,
        CHARGING,          // 充电中（由auto_charge节点处理）
        RETURNING,
        TASK_COMPLETE,
        ERROR
    };


    // =========================================================
    // ROS
    // =========================================================

    ros::NodeHandle nh_;

    ros::Publisher goal_pub_;
    ros::Publisher charge_pub_;          // 发布充电指令给auto_charge节点

    ros::Publisher face_enable_pub_;
    ros::Publisher voice_enable_pub_;

    ros::Subscriber status_sub_;
    ros::Subscriber voice_sub_;
    ros::Subscriber face_sub_;
    ros::Subscriber charge_complete_sub_;  // 监听充电完成消息


    // =========================================================
    // 任务
    // =========================================================

    std::vector<Waypoint> route_;

    int current_index_ = 0;


    // =========================================================
    // 状态变量
    // =========================================================

    State current_state_ =
        State::WAITING_START;

    std::atomic<bool>
        speech_finished_{false};

    std::atomic<bool>
        voice_command_received_{false};

    std::atomic<bool>
        charge_completed_{false};

    std::atomic<bool>
        face_detected_{false};

    bool all_done_ = false;


    // =========================================================
    // 地点停留计时
    // =========================================================

    std::chrono::steady_clock::time_point
        waypoint_arrival_time_;

    bool waypoint_timer_started_ =
        false;

    const double
        MIN_WAYPOINT_STOP_SECONDS = 5.0;


    // =========================================================
    // 状态锁
    // =========================================================

    std::mutex state_mutex_;


    // =========================================================
    // 语音关键词
    // =========================================================

    const std::vector<std::string>
        start_keywords_ = {
            "参观"
        };


    // =========================================================
    // 参数
    // =========================================================

    const int
        NAV_TIMEOUT_SECONDS = 120;

    const int
        LOOP_RATE_HZ = 10;

    const int
        IDLE_REMIND_INTERVAL = 30;

    const int
        CHARGE_TIMEOUT_SECONDS = 120;  // 增加超时时间，因为auto_charge需要导航+充电


    // =========================================================
    // 坐标
    // =========================================================

    const double
        LOBBY_X = 1.241;

    const double
        LOBBY_Y = 2.137;

    const double
        START_X = 0.0;

    const double
        START_Y = 0.0;


    // =========================================================
    // 导航目标类型标识
    // =========================================================

    enum class GoalType
    {
        LOBBY,
        WAYPOINT,
        HOME,
        NONE
    };

    GoalType current_goal_type_ = GoalType::NONE;
    bool is_returning_to_home_ = false;


    // =========================================================
    // TTS预热标志
    // =========================================================

    std::atomic<bool> tts_warmed_up_{false};


public:

    // =========================================================
    // 构造函数
    // =========================================================

    TaskScheduler()
    {
        goal_pub_ =
            nh_.advertise<
                geometry_msgs::PoseStamped
            >(
                "/nav_goal",
                10
            );

        charge_pub_ =
            nh_.advertise<
                std_msgs::Bool
            >(
                "/charge_command",
                10
            );

        face_enable_pub_ =
            nh_.advertise<
                std_msgs::Bool
            >(
                "/face_detection_enable",
                1,
                true
            );

        voice_enable_pub_ =
            nh_.advertise<
                std_msgs::Bool
            >(
                "/voice_listen_enable",
                1,
                true
            );

        status_sub_ =
            nh_.subscribe(
                "/nav_status",
                10,
                &TaskScheduler::statusCallback,
                this
            );

        voice_sub_ =
            nh_.subscribe(
                "/voice_recognition",
                10,
                &TaskScheduler::voiceCallback,
                this
            );

        face_sub_ =
            nh_.subscribe(
                "/face_detected",
                10,
                &TaskScheduler::faceCallback,
                this
            );

        charge_complete_sub_ =
            nh_.subscribe(
                "/charge_complete",
                10,
                &TaskScheduler::chargeCompleteCallback,
                this
            );

        initRoute();
        warmUpTTS();

        ROS_INFO("🚀 智能导览机器人已启动");
        ROS_INFO("📍 共加载 %zu 个参观目标点", route_.size());
        ROS_INFO("📊 初始状态: %s", stateToString(current_state_).c_str());
        ROS_INFO("📡 充电将由 auto_charge 节点处理");
    }


    // =========================================================
    // 预热TTS引擎
    // =========================================================

    void warmUpTTS()
    {
        if (tts_warmed_up_.load()) return;

        ROS_INFO("🔥 预热TTS引擎...");
        std::string cmd =
            "/home/reicom2025/.local/bin/edge-playback "
            "--voice zh-CN-YunxiNeural "
            "--text \" \" "
            "2>/dev/null";
        system(cmd.c_str());
        tts_warmed_up_ = true;
        ROS_INFO("✅ TTS引擎预热完成");
    }


    // =========================================================
    // 路线
    // =========================================================

    void initRoute()
    {
        const double ANGLE_0 = 0.0;
        const double ANGLE_90 = M_PI / 2.0;
        const double ANGLE_NEG_90 = -M_PI / 2.0;
        
        route_ = {
            {"餐厅", 2.220, 1.034, ANGLE_0, "这里是餐厅"},
            {"厨房", 1.251, 1.013, ANGLE_90, "这里是厨房"},
            {"客厅", 1.267, -0.028, ANGLE_NEG_90, "这里是客厅"},
            {"卧室", 2.252, -0.056, ANGLE_NEG_90, "这里是卧室"}
        };
    }


    // =========================================================
    // 人脸启停
    // =========================================================

    void setFaceDetection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        face_enable_pub_.publish(msg);
        ROS_INFO("👤 人脸检测控制: %s", enable ? "开启" : "关闭");
    }


    // =========================================================
    // 用户语音监听启停
    // =========================================================

    void setVoiceListening(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        voice_enable_pub_.publish(msg);
        ROS_INFO("🎤 语音监听控制: %s", enable ? "开启" : "关闭");
    }


    // =========================================================
    // 等待nav_goal_node准备完成
    // =========================================================

    bool waitForNavGoalReady(double timeout_seconds = 20.0)
    {
        ROS_INFO("⏳ 等待 nav_goal_node 准备完成...");

        ros::WallTime start_time = ros::WallTime::now();
        ros::WallRate rate(10.0);

        while (ros::ok() && goal_pub_.getNumSubscribers() == 0)
        {
            if ((ros::WallTime::now() - start_time).toSec() >= timeout_seconds)
            {
                ROS_ERROR("❌ 等待 nav_goal_node 超时！");
                return false;
            }
            ROS_INFO_THROTTLE(2.0, "⏳ /nav_goal 暂无订阅者...");
            rate.sleep();
        }

        ROS_INFO("✅ nav_goal_node 已就绪");
        return true;
    }


    // =========================================================
    // 人脸回调
    // =========================================================

    void faceCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data || current_state_ != State::WAITING_FACE) return;

        ROS_INFO("========================================");
        ROS_INFO("👤 检测到用户人脸");

        transitionTo(State::IDLE);
        face_detected_ = true;

        setFaceDetection(false);
        setVoiceListening(false);

        speakSync("你好，需要帮助吗？");

        setVoiceListening(true);
        ROS_INFO("🎧 现在开始等待用户语音...");
        ROS_INFO("========================================");
    }


    // =========================================================
    // 用户语音回调
    // =========================================================

    void voiceCallback(const std_msgs::String::ConstPtr& msg)
    {
        const std::string recognized_text = msg->data;

        ROS_INFO("========================================");
        ROS_INFO("🎤 收到用户语音: [%s]", recognized_text.c_str());
        ROS_INFO("📊 当前状态: [%s]", stateToString(current_state_).c_str());
        ROS_INFO("========================================");

        if (current_state_ != State::IDLE)
        {
            ROS_WARN("⚠️ 当前不在等待用户指令状态，忽略本次语音");
            return;
        }

        if (!containsKeyword(recognized_text, start_keywords_))
        {
            ROS_WARN("⚠️ 当前文字不是有效导览命令: [%s]", recognized_text.c_str());
            setVoiceListening(false);
            face_detected_ = false;
            voice_command_received_ = false;
            transitionTo(State::WAITING_FACE);
            setFaceDetection(true);
            ROS_INFO("👤 指令中没有“参观”，重新进入人脸等待阶段");
            return;
        }

        if (voice_command_received_.exchange(true))
        {
            ROS_WARN("⚠️ 导览任务已经触发，忽略重复命令");
            return;
        }

        ROS_INFO("✅ 检测到任务一导览指令");
        setVoiceListening(false);
        setFaceDetection(false);
        speakSync("好的，请跟我来");
        startNavigation();
    }


    // =========================================================
    // 关键词判断
    // =========================================================

    bool containsKeyword(const std::string& text, const std::vector<std::string>& keywords)
    {
        for (const auto& keyword : keywords)
        {
            if (text.find(keyword) != std::string::npos)
            {
                ROS_INFO("🔍 匹配到关键词: [%s]", keyword.c_str());
                return true;
            }
        }
        return false;
    }


    // =========================================================
    // 同步TTS
    // =========================================================

    void speakSync(const std::string& text)
    {
        ROS_INFO("🔊 语音播报: %s", text.c_str());

        std::string cmd =
            "/home/reicom2025/.local/bin/edge-playback "
            "--voice zh-CN-YunxiNeural "
            "--text \"" + text + "\" "
            "2>/dev/null";

        int ret = system(cmd.c_str());
        if (ret != 0)
        {
            ROS_WARN("⚠️ 语音播报异常，返回码: %d", ret);
        }
    }


    // =========================================================
    // 导航反馈
    // =========================================================

    void statusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (current_state_ != State::NAVIGATING &&
            current_state_ != State::GOING_TO_LOBBY &&
            current_state_ != State::RETURNING)
        {
            return;
        }

        ROS_INFO(
            "📩 收到导航反馈: %s (目标类型: %s)",
            msg->data ? "✅ 成功" : "❌ 失败",
            goalTypeToString(current_goal_type_).c_str()
        );

        if (msg->data)
        {
            // -------------------------------------------------
            // 到达走廊
            // -------------------------------------------------
            if (current_state_ == State::GOING_TO_LOBBY)
            {
                ROS_INFO("========================================");
                ROS_INFO("✅ 已到达走廊位置 (%.3f, %.3f)", LOBBY_X, LOBBY_Y);

                face_detected_ = false;
                voice_command_received_ = false;

                setVoiceListening(false);
                transitionTo(State::WAITING_FACE);
                setFaceDetection(true);
                ros::spinOnce();

                ROS_INFO("👤 人脸检测已开启，等待用户...");
                ROS_INFO("========================================");
                return;
            }

            // -------------------------------------------------
            // 到达参观点
            // -------------------------------------------------
            if (current_state_ == State::NAVIGATING)
            {
                handleNavigationSuccess();
                return;
            }

            // -------------------------------------------------
            // 返回出发区
            // -------------------------------------------------
            if (current_state_ == State::RETURNING)
            {
                ROS_INFO("========================================");
                ROS_INFO("✅ 已返回出发区");

                if (is_returning_to_home_)
                {
                    ROS_INFO("🏁 任务结束返回，不再继续导航");
                    ROS_INFO("========================================");
                    transitionTo(State::TASK_COMPLETE);
                    all_done_ = true;
                    return;
                }
                else
                {
                    ROS_INFO("📍 准备再次前往走廊");
                    ROS_INFO("========================================");

                    current_index_ = 0;
                    speech_finished_ = false;
                    waypoint_timer_started_ = false;
                    face_detected_ = false;
                    voice_command_received_ = false;

                    setFaceDetection(false);
                    setVoiceListening(false);

                    transitionTo(State::GOING_TO_LOBBY);
                    current_goal_type_ = GoalType::LOBBY;
                    sendLobbyGoal();
                    return;
                }
            }
        }
        else
        {
            // -------------------------------------------------
            // 导航失败处理
            // -------------------------------------------------
            if (current_state_ == State::GOING_TO_LOBBY)
            {
                ROS_ERROR("❌ 前往走廊失败，1秒后重新尝试");
                ros::WallDuration(1.0).sleep();
                sendLobbyGoal();
                return;
            }

            if (current_state_ == State::NAVIGATING)
            {
                handleNavigationFailure();
                return;
            }

            if (current_state_ == State::RETURNING)
            {
                ROS_ERROR("❌ 返回出发区失败");
                transitionTo(State::ERROR);
                return;
            }
        }
    }


    // =========================================================
    // 成功到达某个参观点
    // =========================================================

    void handleNavigationSuccess()
    {
        if (current_index_ >= static_cast<int>(route_.size()))
        {
            ROS_ERROR("❌ 目标点索引越界");
            transitionTo(State::ERROR);
            return;
        }

        const std::string description = route_[current_index_].description;
        const std::string name = route_[current_index_].name;

        ROS_INFO("========================================");
        ROS_INFO("✅ 成功到达目标点: %s", name.c_str());

        speakSync(description);

        waypoint_arrival_time_ = std::chrono::steady_clock::now();
        waypoint_timer_started_ = true;

        ROS_INFO("⏱️ %s 语音播报完成，开始5秒停留", name.c_str());

        transitionTo(State::WAITING_SPEECH);
    }


    // =========================================================
    // 导航失败
    // =========================================================

    void handleNavigationFailure()
    {
        if (current_index_ >= static_cast<int>(route_.size()))
        {
            transitionTo(State::ERROR);
            return;
        }

        ROS_ERROR("❌ 导航到 %s 失败", route_[current_index_].name.c_str());

        if (current_index_ < static_cast<int>(route_.size()) - 1)
        {
            ROS_WARN("🔄 跳过失败目标，继续下一目标");
            current_index_++;
            transitionTo(State::NAVIGATING);
            current_goal_type_ = GoalType::WAYPOINT;
            sendNextGoal();
        }
        else
        {
            ROS_WARN("⚠️ 最后一个目标失败，本轮导览结束");
            handleTaskComplete();
        }
    }


    // =========================================================
    // 发送走廊目标
    // =========================================================

    void sendLobbyGoal()
    {
        ROS_INFO("📍 导航到走廊位置 (%.2f, %.2f)", LOBBY_X, LOBBY_Y);

        current_goal_type_ = GoalType::LOBBY;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = LOBBY_X;
        goal.pose.position.y = LOBBY_Y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        goal_pub_.publish(goal);
        ROS_INFO("📤 走廊目标已发布到 /nav_goal");
    }


    // =========================================================
    // 发送下一个任务目标
    // =========================================================

    void sendNextGoal()
    {
        if (current_index_ >= static_cast<int>(route_.size()))
        {
            ROS_ERROR("❌ 目标索引越界，无法发送导航目标");
            transitionTo(State::ERROR);
            return;
        }

        if (current_state_ != State::NAVIGATING)
        {
            transitionTo(State::NAVIGATING);
        }

        current_goal_type_ = GoalType::WAYPOINT;

        Waypoint& wp = route_[current_index_];

        ROS_INFO("🔄 发送第 %d/%zu 个目标: %s (%.2f, %.2f)",
                 current_index_ + 1, route_.size(), wp.name.c_str(), wp.x, wp.y);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(wp.angle / 2.0);
        goal.pose.orientation.w = std::cos(wp.angle / 2.0);

        goal_pub_.publish(goal);
        ROS_INFO("📤 目标已发布到 /nav_goal，等待导航反馈...");
    }


    // =========================================================
    // 开始任务一
    // =========================================================

    void startNavigation()
    {
        ROS_INFO("🚀 开始导览任务");

        current_index_ = 0;
        transitionTo(State::NAVIGATING);
        current_goal_type_ = GoalType::WAYPOINT;
        sendNextGoal();
    }


    // =========================================================
    // 🔥 充电完成回调
    // =========================================================

    void chargeCompleteCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (current_state_ == State::CHARGING)
        {
            ROS_INFO("========================================");
            if (msg->data)
            {
                ROS_INFO("🔋 收到充电完成消息 - 充电成功");
            }
            else
            {
                ROS_WARN("⚠️ 收到充电完成消息 - 充电失败");
            }
            ROS_INFO("========================================");

            charge_completed_ = true;
        }
    }


    // =========================================================
    // 🔥 发送充电指令给auto_charge节点
    // =========================================================

    void sendChargeCommand()
    {
        ROS_INFO("🔋 发送充电指令到 auto_charge 节点...");
        std_msgs::Bool msg;
        msg.data = true;
        charge_pub_.publish(msg);
        ROS_INFO("📤 充电指令已发布到 /charge_command");
    }


    // =========================================================
    // 🔥 任务一完成 -> 触发auto_charge节点充电
    // =========================================================

    void handleTaskComplete()
    {
        ROS_INFO("========================================");
        ROS_INFO("🏁 任务一导览完成");
        ROS_INFO("✅ 餐厅、厨房、客厅、卧室全部参观完成");
        ROS_INFO("🔋 触发 auto_charge 节点进行充电");
        ROS_INFO("========================================");

        speakSync("参观结束，现在去充电");

        // 发送充电指令给auto_charge节点
        sendChargeCommand();

        // 进入充电状态
        transitionTo(State::CHARGING);
        charge_completed_ = false;

        // 等待充电完成（由auto_charge节点发布/charge_complete）
        waitForChargeComplete();
    }


    // =========================================================
    // 🔥 等待充电完成
    // =========================================================

    void waitForChargeComplete()
    {
        ROS_INFO("⏳ 等待 auto_charge 节点完成充电...");

        int charge_timeout_counter = 0;
        ros::Rate loop_rate(LOOP_RATE_HZ);

        while (ros::ok() && !charge_completed_.load())
        {
            ros::spinOnce();
            charge_timeout_counter++;

            if (charge_timeout_counter > CHARGE_TIMEOUT_SECONDS * LOOP_RATE_HZ)
            {
                ROS_ERROR("⏰ 充电超时！(%d秒)，跳过充电直接返回起点", CHARGE_TIMEOUT_SECONDS);
                break;
            }

            if (charge_timeout_counter % (10 * LOOP_RATE_HZ) == 0)
            {
                ROS_INFO("⏳ 等待充电完成... 已等待 %d 秒",
                         charge_timeout_counter / LOOP_RATE_HZ);
            }

            loop_rate.sleep();
        }

        if (charge_completed_.load())
        {
            ROS_INFO("✅ auto_charge 节点报告充电完成");
        }

        // 充电完成后返回起点
        returnToStart();
    }


    // =========================================================
    // 返回出发区
    // =========================================================

    void returnToStart()
    {
        ROS_INFO("========================================");
        ROS_INFO("🏠 返回出发区 (%.2f, %.2f)", START_X, START_Y);
        ROS_INFO("========================================");

        is_returning_to_home_ = true;
        current_goal_type_ = GoalType::HOME;

        geometry_msgs::PoseStamped home;
        home.header.frame_id = "map";
        home.header.stamp = ros::Time::now();
        home.pose.position.x = START_X;
        home.pose.position.y = START_Y;
        home.pose.position.z = 0.0;
        home.pose.orientation.x = 0.0;
        home.pose.orientation.y = 0.0;
        home.pose.orientation.z = 0.0;
        home.pose.orientation.w = 1.0;

        transitionTo(State::RETURNING);
        goal_pub_.publish(home);

        ROS_INFO("📤 已发送返回出发区命令");
        ROS_INFO("🔄 正在返回出发区...");

        int return_timeout = 0;

        while (ros::ok() && current_state_ == State::RETURNING)
        {
            ros::spinOnce();
            ros::WallDuration(0.1).sleep();
            return_timeout++;

            if (return_timeout > NAV_TIMEOUT_SECONDS * 10)
            {
                ROS_ERROR("⏰ 返回出发区超时，强制结束");
                break;
            }
        }

        is_returning_to_home_ = false;

        speakSync("已回到起点，感谢参观");

        if (current_state_ != State::TASK_COMPLETE)
        {
            transitionTo(State::TASK_COMPLETE);
        }

        all_done_ = true;
        ROS_INFO("✅ 已回到起点，任务全部完成");
    }


    // =========================================================
    // 目标类型转字符串
    // =========================================================

    std::string goalTypeToString(GoalType type)
    {
        switch (type)
        {
            case GoalType::LOBBY:    return "走廊";
            case GoalType::WAYPOINT: return "参观目标点";
            case GoalType::HOME:     return "起点(任务结束)";
            case GoalType::NONE:
            default:                 return "无";
        }
    }


    // =========================================================
    // 地点5秒原则
    // =========================================================

    void checkSpeechAndProceed()
    {
        if (current_state_ != State::WAITING_SPEECH || !waypoint_timer_started_)
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed_seconds =
            std::chrono::duration<double>(now - waypoint_arrival_time_).count();

        if (elapsed_seconds < MIN_WAYPOINT_STOP_SECONDS)
        {
            ROS_INFO_THROTTLE(0.5, "⏱️ 继续停留 %.1f 秒以满足5秒要求",
                              MIN_WAYPOINT_STOP_SECONDS - elapsed_seconds);
            return;
        }

        ROS_INFO("✅ %s 停留完成：累计 %.2f 秒",
                 route_[current_index_].name.c_str(), elapsed_seconds);

        speech_finished_ = false;
        waypoint_timer_started_ = false;

        current_index_++;

        if (current_index_ >= static_cast<int>(route_.size()))
        {
            handleTaskComplete();
        }
        else
        {
            transitionTo(State::NAVIGATING);
            current_goal_type_ = GoalType::WAYPOINT;
            sendNextGoal();
        }
    }


    // =========================================================
    // 错误处理
    // =========================================================

    void handleError()
    {
        ROS_ERROR("🚨 系统进入错误状态");
    }


    // =========================================================
    // 状态转换
    // =========================================================

    void transitionTo(State new_state)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (current_state_ == new_state) return;

        ROS_INFO("🔄 状态转换: %s -> %s",
                 stateToString(current_state_).c_str(),
                 stateToString(new_state).c_str());

        current_state_ = new_state;
    }


    // =========================================================
    // 状态名称
    // =========================================================

    std::string stateToString(State state)
    {
        switch (state)
        {
            case State::WAITING_START:   return "等待启动";
            case State::GOING_TO_LOBBY:  return "前往走廊";
            case State::WAITING_FACE:    return "等待人脸识别";
            case State::IDLE:            return "空闲(等待指令)";
            case State::NAVIGATING:      return "导航中";
            case State::WAITING_SPEECH:  return "语音播报中";
            case State::CHARGING:        return "充电中(auto_charge)";
            case State::RETURNING:       return "返回起点中";
            case State::TASK_COMPLETE:   return "任务完成";
            case State::ERROR:           return "错误状态";
            default:                     return "未知状态";
        }
    }


    // =========================================================
    // 主循环
    // =========================================================

    void start()
    {
        ROS_INFO("▶️ 开始执行任务调度");

        ros::WallDuration(1.0).sleep();

        setFaceDetection(false);
        setVoiceListening(false);

        ROS_INFO("📍 开始导航到走廊位置");

        transitionTo(State::GOING_TO_LOBBY);

        if (!waitForNavGoalReady(20.0))
        {
            ROS_ERROR("❌ nav_goal_node 未就绪，无法开始任务");
            transitionTo(State::ERROR);
            return;
        }

        sendLobbyGoal();

        ros::Rate loop_rate(LOOP_RATE_HZ);

        int timeout_counter = 0;
        int waiting_face_counter = 0;
        State last_state = current_state_;

        while (ros::ok() && !all_done_)
        {
            ros::spinOnce();

            if (current_state_ != last_state)
            {
                timeout_counter = 0;
                waiting_face_counter = 0;
                last_state = current_state_;
            }

            switch (current_state_)
            {
                case State::GOING_TO_LOBBY:
                {
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 前往走廊超时");
                        timeout_counter = 0;
                        sendLobbyGoal();
                    }
                    break;
                }

                case State::WAITING_FACE:
                {
                    waiting_face_counter++;
                    if (waiting_face_counter % (10 * LOOP_RATE_HZ) == 0)
                    {
                        ROS_INFO("👤 等待人脸识别唤醒...");
                    }
                    break;
                }

                case State::IDLE:
                {
                    timeout_counter++;
                    if (timeout_counter % (IDLE_REMIND_INTERVAL * LOOP_RATE_HZ) == 0)
                    {
                        ROS_INFO("💬 请说带有“参观”的指令");
                    }
                    break;
                }

                case State::NAVIGATING:
                {
                    timeout_counter++;
                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 导航超时");
                        timeout_counter = 0;
                        handleNavigationFailure();
                    }
                    break;
                }

                case State::WAITING_SPEECH:
                {
                    checkSpeechAndProceed();
                    break;
                }

                case State::CHARGING:
                {
                    // 充电由auto_charge节点处理，这里只等待
                    ROS_INFO_THROTTLE(10.0, "🔋 auto_charge 节点正在处理充电...");
                    break;
                }

                case State::RETURNING:
                {
                    timeout_counter++;
                    ROS_INFO_THROTTLE(5.0, "🔄 机器人正在返回出发区...");

                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 返回出发区超时");
                        timeout_counter = 0;
                        returnToStart();
                    }
                    break;
                }

                case State::TASK_COMPLETE:
                {
                    all_done_ = true;
                    break;
                }

                case State::ERROR:
                {
                    ROS_WARN_THROTTLE(5.0, "⛔ 系统处于错误状态");
                    break;
                }

                case State::WAITING_START:
                default:
                {
                    break;
                }
            }

            loop_rate.sleep();
        }

        ROS_INFO("🛑 调度器结束运行");
    }
};


// =============================================================
// main
// =============================================================

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "task_scheduler");

    ROS_INFO("智能导览机器人系统");
    ROS_INFO("📡 充电由 auto_charge 节点控制");

    TaskScheduler scheduler;
    scheduler.start();

    return 0;
}
