/**
 * @file task_scheduler.cpp
 * @brief 服务机器人任务调度：任务一导览 + 任务三目标寻找
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
#include <mutex>
#include <cmath>
#include <clocale>


struct Waypoint
{
    std::string name;
    double x;
    double y;

    // 只给任务三使用：到点后面向柜子的角度
    double task3_angle;

    // 任务一地点介绍
    std::string description;
};


class TaskScheduler
{
private:

    enum class State
    {
        WAITING_START,
        GOING_TO_LOBBY,
        WAITING_FACE,
        IDLE,
        NAVIGATING,
        TASK3_DETECTING,
        WAITING_SPEECH,
        CHARGING,
        RETURNING,
        ERROR
    };


    enum class TaskType
    {
        NONE,
        TASK1_GUIDE,
        TASK3_FIND_PHONE,
        TASK3_FIND_BACKPACK
    };


    // =========================================================
    // ROS
    // =========================================================

    ros::NodeHandle nh_;

    ros::Publisher goal_pub_;
    ros::Publisher charge_pub_;
    ros::Publisher face_enable_pub_;
    ros::Publisher voice_enable_pub_;
    ros::Publisher task3_detection_enable_pub_;

    ros::Subscriber status_sub_;
    ros::Subscriber voice_sub_;
    ros::Subscriber face_sub_;
    ros::Subscriber charge_complete_sub_;
    ros::Subscriber task3_detection_result_sub_;


    // =========================================================
    // 任务数据
    // =========================================================

    std::vector<Waypoint> route_;
    int current_index_ = 0;

    TaskType current_task_ = TaskType::NONE;
    State current_state_ = State::WAITING_START;


    // =========================================================
    // 状态变量
    // =========================================================

    std::atomic<bool> speech_finished_{false};
    std::atomic<bool> voice_command_received_{false};
    std::atomic<bool> face_detected_{false};

    // 当前任务三地点是否已经找到“用户指定的目标”
    bool task3_target_found_current_ = false;


    // =========================================================
    // 地点停留计时
    // =========================================================

    std::chrono::steady_clock::time_point waypoint_arrival_time_;
    bool waypoint_timer_started_ = false;

    const double MIN_WAYPOINT_STOP_SECONDS = 5.0;


    // =========================================================
    // 状态锁
    // =========================================================

    std::mutex state_mutex_;


    // =========================================================
    // 参数
    // =========================================================

    const int NAV_TIMEOUT_SECONDS = 120;
    const int LOOP_RATE_HZ = 10;
    const int IDLE_REMIND_INTERVAL = 30;

    // detector 正常约 1.5 秒返回；10 秒仅作为视觉链路故障保护，不改变1.5秒识别窗口
    const int TASK3_DETECTION_TIMEOUT_SECONDS = 10;

    // auto_charge 本身含导航、AR定位、30秒充电，60秒容易不够
    const int CHARGE_TIMEOUT_SECONDS = 180;


    // =========================================================
    // 坐标
    // =========================================================

    const double LOBBY_X = 1.241;
    const double LOBBY_Y = 2.137;

    const double START_X = 0.0;
    const double START_Y = 0.0;


public:

    TaskScheduler()
    {
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/nav_goal",
            10
        );

        charge_pub_ = nh_.advertise<std_msgs::Bool>(
            "/charge_command",
            10
        );

        // 这些控制 Topic 使用 latched publisher，保证后启动的节点也能拿到最新状态
        face_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "/face_detection_enable",
            1,
            true
        );

        voice_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "/voice_listen_enable",
            1,
            true
        );

        task3_detection_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "/task3_detection_enable",
            1,
            true
        );

        status_sub_ = nh_.subscribe(
            "/nav_status",
            10,
            &TaskScheduler::statusCallback,
            this
        );

        voice_sub_ = nh_.subscribe(
            "/voice_recognition",
            10,
            &TaskScheduler::voiceCallback,
            this
        );

        face_sub_ = nh_.subscribe(
            "/face_detected",
            10,
            &TaskScheduler::faceCallback,
            this
        );

        charge_complete_sub_ = nh_.subscribe(
            "/charge_complete",
            10,
            &TaskScheduler::chargeCompleteCallback,
            this
        );

        task3_detection_result_sub_ = nh_.subscribe(
            "/task3_detection_result",
            10,
            &TaskScheduler::task3DetectionResultCallback,
            this
        );

        initRoute();

        ROS_INFO("========================================");
        ROS_INFO("🚀 任务调度器已启动");
        ROS_INFO("✅ 任务一：导览，不充电");
        ROS_INFO("✅ 任务三：找手机/书包，完成后充电");
        ROS_INFO("📍 共加载 %zu 个目标点", route_.size());
        ROS_INFO("========================================");
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
            {
                "餐厅",
                2.220,
                1.034,
                ANGLE_0,
                "这里是餐厅"
            },
            {
                "厨房",
                1.251,
                1.013,
                ANGLE_90,
                "这里是厨房"
            },
            {
                "客厅",
                1.267,
                -0.028,
                ANGLE_NEG_90,
                "这里是客厅"
            },
            {
                "卧室",
                2.252,
                -0.056,
                ANGLE_NEG_90,
                "这里是卧室"
            }
        };
    }


    // =========================================================
    // 基础控制
    // =========================================================

    void setFaceDetection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        face_enable_pub_.publish(msg);

        ROS_INFO(
            "👤 人脸检测控制: %s",
            enable ? "开启" : "关闭"
        );
    }


    void setVoiceListening(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        voice_enable_pub_.publish(msg);

        ROS_INFO(
            "🎤 语音监听控制: %s",
            enable ? "开启" : "关闭"
        );
    }


    void setTask3Detection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        task3_detection_enable_pub_.publish(msg);

        ROS_INFO(
            "📷 任务三目标检测控制: %s",
            enable ? "开启" : "关闭"
        );
    }


    bool waitForNavGoalReady(double timeout_seconds = 20.0)
    {
        ROS_INFO("⏳ 等待 nav_goal_node 准备完成...");

        ros::WallTime start_time = ros::WallTime::now();
        ros::WallRate rate(10.0);

        while (
            ros::ok()
            &&
            goal_pub_.getNumSubscribers() == 0
        )
        {
            const double elapsed = (
                ros::WallTime::now() - start_time
            ).toSec();

            if (elapsed >= timeout_seconds)
            {
                ROS_ERROR(
                    "❌ 等待 nav_goal_node 超时，%.1f 秒内 /nav_goal 没有订阅者",
                    timeout_seconds
                );
                return false;
            }

            ROS_INFO_THROTTLE(
                2.0,
                "⏳ /nav_goal 暂无订阅者，继续等待..."
            );

            rate.sleep();
        }

        if (!ros::ok())
        {
            return false;
        }

        ROS_INFO("✅ nav_goal_node 已就绪");
        return true;
    }


    // =========================================================
    // 人脸 -> 欢迎语 -> 开启语音
    // =========================================================

    void faceCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data)
        {
            return;
        }

        if (current_state_ != State::WAITING_FACE)
        {
            return;
        }

        ROS_INFO("========================================");
        ROS_INFO("👤 检测到用户人脸");

        transitionTo(State::IDLE);
        face_detected_ = true;

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        // 欢迎语完整播放后才开始听用户讲话
        speakSync("你好，需要帮助吗？");

        setVoiceListening(true);

        ROS_INFO("🎧 现在开始等待用户语音...");
        ROS_INFO("========================================");
    }


    // =========================================================
    // 语音任务分流
    // =========================================================

    void voiceCallback(const std_msgs::String::ConstPtr& msg)
    {
        const std::string text = msg->data;

        ROS_INFO("========================================");
        ROS_INFO("🎤 收到用户语音: [%s]", text.c_str());
        ROS_INFO("📊 当前状态: [%s]", stateToString(current_state_).c_str());
        ROS_INFO("========================================");

        if (current_state_ != State::IDLE)
        {
            ROS_WARN("⚠️ 当前不在等待指令状态，忽略本次语音");
            return;
        }

        const bool has_guide = containsText(text, "参观");
        const bool has_phone = containsText(text, "手机");
        const bool has_backpack = containsText(text, "书包");

        // -----------------------------------------------------
        // 同一句同时出现“手机”和“书包”：避免猜用户到底要找哪个
        // -----------------------------------------------------
        if (has_phone && has_backpack)
        {
            setVoiceListening(false);
            speakSync("请告诉我需要找手机还是书包");
            setVoiceListening(true);
            return;
        }

        // -----------------------------------------------------
        // 任务三优先匹配
        // -----------------------------------------------------
        if (has_phone || has_backpack)
        {
            if (voice_command_received_.exchange(true))
            {
                ROS_WARN("⚠️ 当前任务已经触发，忽略重复命令");
                return;
            }

            setVoiceListening(false);
            setFaceDetection(false);
            setTask3Detection(false);

            if (has_phone)
            {
                current_task_ = TaskType::TASK3_FIND_PHONE;
                ROS_INFO("✅ 触发任务三：寻找手机");
            }
            else
            {
                current_task_ = TaskType::TASK3_FIND_BACKPACK;
                ROS_INFO("✅ 触发任务三：寻找书包");
            }

            startCurrentTaskNavigation();
            return;
        }

        // -----------------------------------------------------
        // 任务一
        // -----------------------------------------------------
        if (has_guide)
        {
            if (voice_command_received_.exchange(true))
            {
                ROS_WARN("⚠️ 当前任务已经触发，忽略重复命令");
                return;
            }

            current_task_ = TaskType::TASK1_GUIDE;

            setVoiceListening(false);
            setFaceDetection(false);
            setTask3Detection(false);

            ROS_INFO("✅ 触发任务一导览");

            // 保留任务一原有要求：完整播完再起步
            speakSync("好的，请跟我来");

            startCurrentTaskNavigation();
            return;
        }

        // -----------------------------------------------------
        // 识别出文字但不是有效任务：重新走人脸唤醒流程
        // -----------------------------------------------------
        ROS_WARN("⚠️ 当前文字不是有效任务命令: [%s]", text.c_str());

        setVoiceListening(false);
        setTask3Detection(false);

        face_detected_ = false;
        voice_command_received_ = false;
        current_task_ = TaskType::NONE;

        transitionTo(State::WAITING_FACE);
        setFaceDetection(true);
    }


    bool containsText(
        const std::string& text,
        const std::string& keyword
    )
    {
        return text.find(keyword) != std::string::npos;
    }


    // =========================================================
    // TTS
    // =========================================================

    void speakSync(const std::string& text)
    {
        ROS_INFO("🔊 语音播报: %s", text.c_str());

        std::string cmd =
            "/home/reicom2025/.local/bin/edge-playback "
            "--voice zh-CN-YunxiNeural "
            "--text \"" + text + "\" "
            "2>/dev/null";

        const int ret = system(cmd.c_str());

        if (ret != 0)
        {
            ROS_WARN("⚠️ 语音播报异常，返回码: %d", ret);
        }
    }


    void speakAsync(
        const std::string& text,
        const std::string& name
    )
    {
        speech_finished_ = false;

        ROS_INFO("🎤 启动异步语音: %s", name.c_str());
        ROS_INFO("📝 内容: %s", text.c_str());

        std::thread(
            [this, text, name]()
            {
                std::string cmd =
                    "/home/reicom2025/.local/bin/edge-playback "
                    "--voice zh-CN-YunxiNeural "
                    "--text \"" + text + "\" "
                    "2>/dev/null";

                const int ret = system(cmd.c_str());

                if (ret == 0)
                {
                    ROS_INFO("✅ 语音完整结束: %s", name.c_str());
                }
                else
                {
                    ROS_WARN("⚠️ %s 语音播报异常，返回码: %d", name.c_str(), ret);
                }

                speech_finished_ = true;
            }
        ).detach();
    }


    // =========================================================
    // 导航反馈
    // =========================================================

    void statusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (
            current_state_ != State::NAVIGATING
            &&
            current_state_ != State::GOING_TO_LOBBY
            &&
            current_state_ != State::RETURNING
        )
        {
            return;
        }

        ROS_INFO(
            "📩 收到导航反馈: %s",
            msg->data ? "✅ 成功" : "❌ 失败"
        );

        if (msg->data)
        {
            if (current_state_ == State::GOING_TO_LOBBY)
            {
                handleLobbyArrival();
                return;
            }

            if (current_state_ == State::NAVIGATING)
            {
                handleWaypointArrival();
                return;
            }

            if (current_state_ == State::RETURNING)
            {
                handleStartArrival();
                return;
            }
        }
        else
        {
            if (current_state_ == State::GOING_TO_LOBBY)
            {
                ROS_ERROR("❌ 前往走廊失败，1秒后重试");
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


    void handleLobbyArrival()
    {
        ROS_INFO("========================================");
        ROS_INFO("✅ 已到达走廊位置 (%.3f, %.3f)", LOBBY_X, LOBBY_Y);

        current_task_ = TaskType::NONE;
        current_index_ = 0;
        task3_target_found_current_ = false;
        speech_finished_ = false;
        waypoint_timer_started_ = false;
        face_detected_ = false;
        voice_command_received_ = false;

        setTask3Detection(false);
        setVoiceListening(false);

        transitionTo(State::WAITING_FACE);
        setFaceDetection(true);

        ROS_INFO("👤 开始等待用户人脸...");
        ROS_INFO("========================================");
    }


    void handleStartArrival()
    {
        ROS_INFO("========================================");
        ROS_INFO("✅ 已返回出发区");
        ROS_INFO("📍 准备再次前往走廊");
        ROS_INFO("========================================");

        resetTaskRuntime();

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        transitionTo(State::GOING_TO_LOBBY);
        sendLobbyGoal();
    }


    // =========================================================
    // 到达餐厅/厨房/客厅/卧室
    // =========================================================

    void handleWaypointArrival()
    {
        if (
            current_index_ < 0
            ||
            current_index_ >= static_cast<int>(route_.size())
        )
        {
            ROS_ERROR("❌ 目标点索引越界");
            transitionTo(State::ERROR);
            return;
        }

        const Waypoint& wp = route_[current_index_];

        ROS_INFO("========================================");
        ROS_INFO("✅ 成功到达目标点: %s", wp.name.c_str());

        // 用户确认：任务一/任务三的5秒都从 move_base 成功到达这一刻开始
        waypoint_arrival_time_ = std::chrono::steady_clock::now();
        waypoint_timer_started_ = true;

        ROS_INFO("⏱️ %s：从现在开始计算至少5秒停留时间", wp.name.c_str());

        if (current_task_ == TaskType::TASK1_GUIDE)
        {
            transitionTo(State::WAITING_SPEECH);
            speakAsync(wp.description, wp.name);
            return;
        }

        if (isTask3())
        {
            task3_target_found_current_ = false;
            speech_finished_ = false;

            transitionTo(State::TASK3_DETECTING);

            ROS_INFO("📷 机器人已稳定到达任务三点位，开启1.5秒目标检测");
            setTask3Detection(true);
            return;
        }

        ROS_ERROR("❌ 到达目标点时没有有效任务类型");
        transitionTo(State::ERROR);
    }


    // =========================================================
    // 任务三视觉结果
    // detector 发布：none / phone / backpack / both / error
    // =========================================================

    void task3DetectionResultCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        if (current_state_ != State::TASK3_DETECTING)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "⚠️ 当前不在任务三检测阶段，忽略视觉结果: %s",
                msg->data.c_str()
            );
            return;
        }

        setTask3Detection(false);

        const std::string result = msg->data;

        ROS_INFO("========================================");
        ROS_INFO("📷 任务三1.5秒多帧识别结果: [%s]", result.c_str());

        if (result == "error")
        {
            ROS_ERROR("❌ 任务三视觉节点没有获得足够有效图像，停止任务以便调试");
            transitionTo(State::ERROR);
            return;
        }

        const bool has_phone = (
            result == "phone"
            ||
            result == "both"
        );

        const bool has_backpack = (
            result == "backpack"
            ||
            result == "both"
        );

        std::string speech;

        if (current_task_ == TaskType::TASK3_FIND_PHONE)
        {
            if (has_phone)
            {
                task3_target_found_current_ = true;
                speech = "找到手机啦，在这里！";
            }
            else if (has_backpack)
            {
                task3_target_found_current_ = false;
                speech = "我看到了书包";
            }
            else
            {
                task3_target_found_current_ = false;
                speech = "这里什么都没有";
            }
        }
        else if (current_task_ == TaskType::TASK3_FIND_BACKPACK)
        {
            if (has_backpack)
            {
                task3_target_found_current_ = true;
                speech = "找到书包啦，在这里！";
            }
            else if (has_phone)
            {
                task3_target_found_current_ = false;
                speech = "我看到了手机";
            }
            else
            {
                task3_target_found_current_ = false;
                speech = "这里什么都没有";
            }
        }
        else
        {
            ROS_ERROR("❌ 收到任务三视觉结果，但当前任务不是任务三");
            transitionTo(State::ERROR);
            return;
        }

        ROS_INFO(
            "🎯 当前地点是否找到用户目标: %s",
            task3_target_found_current_ ? "是" : "否"
        );
        ROS_INFO("🔊 本地点将播报: %s", speech.c_str());
        ROS_INFO("========================================");

        transitionTo(State::WAITING_SPEECH);
        speakAsync(speech, route_[current_index_].name + "巡检结果");
    }


    // =========================================================
    // 5秒规则：任务一与任务三共用
    // =========================================================

    void checkSpeechAndProceed()
    {
        if (current_state_ != State::WAITING_SPEECH)
        {
            return;
        }

        if (!waypoint_timer_started_)
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();

        const double elapsed_seconds =
            std::chrono::duration<double>(
                now - waypoint_arrival_time_
            ).count();

        const bool speech_done = speech_finished_.load();
        const bool minimum_stop_done =
            elapsed_seconds >= MIN_WAYPOINT_STOP_SECONDS;

        if (!speech_done)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "🔊 已停留 %.1f 秒，等待本地点任务和语音完整结束...",
                elapsed_seconds
            );
            return;
        }

        if (!minimum_stop_done)
        {
            ROS_INFO_THROTTLE(
                0.5,
                "⏱️ 本地点任务已完成，还需停留 %.1f 秒满足5秒规则",
                MIN_WAYPOINT_STOP_SECONDS - elapsed_seconds
            );
            return;
        }

        ROS_INFO(
            "✅ %s 本地点处理完成：累计 %.2f 秒，满足5秒规则",
            route_[current_index_].name.c_str(),
            elapsed_seconds
        );

        speech_finished_ = false;
        waypoint_timer_started_ = false;

        // -----------------------------------------------------
        // 任务一：四点全部走完 -> 直接回出发区，不充电
        // -----------------------------------------------------
        if (current_task_ == TaskType::TASK1_GUIDE)
        {
            current_index_++;

            if (current_index_ >= static_cast<int>(route_.size()))
            {
                handleTask1Complete();
            }
            else
            {
                transitionTo(State::NAVIGATING);
                sendNextGoal();
            }

            return;
        }

        // -----------------------------------------------------
        // 任务三：
        // 找到目标 -> 立即结束巡检并充电
        // 没找到 -> 下一个地点
        // 四点全走完仍没找到 -> 充电
        // -----------------------------------------------------
        if (isTask3())
        {
            if (task3_target_found_current_)
            {
                ROS_INFO("🎯 已找到指定目标，提前结束巡检");
                beginCharging();
                return;
            }

            current_index_++;

            if (current_index_ >= static_cast<int>(route_.size()))
            {
                ROS_INFO("🔎 四个地点均已巡检完成，未找到指定目标");
                beginCharging();
            }
            else
            {
                transitionTo(State::NAVIGATING);
                sendNextGoal();
            }

            return;
        }

        ROS_ERROR("❌ WAITING_SPEECH阶段没有有效任务类型");
        transitionTo(State::ERROR);
    }


    // =========================================================
    // 导航失败
    // =========================================================

    void handleNavigationFailure()
    {
        if (
            current_index_ < 0
            ||
            current_index_ >= static_cast<int>(route_.size())
        )
        {
            transitionTo(State::ERROR);
            return;
        }

        ROS_ERROR(
            "❌ 导航到 %s 失败",
            route_[current_index_].name.c_str()
        );

        setTask3Detection(false);

        current_index_++;

        if (current_index_ < static_cast<int>(route_.size()))
        {
            ROS_WARN("🔄 跳过失败目标，继续下一地点");
            transitionTo(State::NAVIGATING);
            sendNextGoal();
            return;
        }

        if (current_task_ == TaskType::TASK1_GUIDE)
        {
            ROS_WARN("⚠️ 任务一最后目标失败，本轮导览结束，直接返回出发区");
            handleTask1Complete();
            return;
        }

        if (isTask3())
        {
            ROS_WARN("⚠️ 任务三所有剩余地点结束/失败，进入充电流程");
            beginCharging();
            return;
        }

        transitionTo(State::ERROR);
    }


    // =========================================================
    // 发送目标
    // =========================================================

    void sendLobbyGoal()
    {
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

        ROS_INFO("📤 走廊目标已发布: (%.3f, %.3f)", LOBBY_X, LOBBY_Y);
    }


    void sendNextGoal()
    {
        if (
            current_index_ < 0
            ||
            current_index_ >= static_cast<int>(route_.size())
        )
        {
            ROS_ERROR("❌ 目标索引越界，无法发送导航目标");
            transitionTo(State::ERROR);
            return;
        }

        if (current_state_ != State::NAVIGATING)
        {
            transitionTo(State::NAVIGATING);
        }

        const Waypoint& wp = route_[current_index_];

        // 任务一不使用柜子专用朝向，恢复为默认0°。
        // 任务三严格使用用户确认的柜子朝向。
        double goal_angle = 0.0;

        if (isTask3())
        {
            goal_angle = wp.task3_angle;
        }

        geometry_msgs::PoseStamped goal;

        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(goal_angle / 2.0);
        goal.pose.orientation.w = std::cos(goal_angle / 2.0);

        goal_pub_.publish(goal);

        ROS_INFO("========================================");
        ROS_INFO(
            "📤 第 %d/%zu 个目标: %s (%.3f, %.3f)",
            current_index_ + 1,
            route_.size(),
            wp.name.c_str(),
            wp.x,
            wp.y
        );

        if (isTask3())
        {
            ROS_INFO(
                "🧭 任务三柜子朝向: %.1f°",
                goal_angle * 180.0 / M_PI
            );
        }
        else
        {
            ROS_INFO("🧭 任务一不使用任务三柜子朝向，使用默认0°导航朝向");
        }

        ROS_INFO("========================================");
    }


    void startCurrentTaskNavigation()
    {
        current_index_ = 0;
        task3_target_found_current_ = false;
        speech_finished_ = false;
        waypoint_timer_started_ = false;

        transitionTo(State::NAVIGATING);
        sendNextGoal();
    }


    // =========================================================
    // 任务完成与充电
    // =========================================================

    void handleTask1Complete()
    {
        ROS_INFO("========================================");
        ROS_INFO("🏁 任务一导览完成");
        ROS_INFO("🚫 任务一不执行充电");
        ROS_INFO("🏠 直接返回出发区");
        ROS_INFO("========================================");

        returnToStart();
    }


    void beginCharging()
    {
        if (!isTask3())
        {
            ROS_ERROR("❌ 非任务三禁止进入充电流程");
            transitionTo(State::ERROR);
            return;
        }

        ROS_INFO("========================================");
        ROS_INFO("🔋 任务三巡检结束，开始充电流程");
        ROS_INFO("========================================");

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        // 必须先切状态，再发命令，防止极快的完成消息被忽略
        transitionTo(State::CHARGING);
        sendChargeCommand();
    }


    void sendChargeCommand()
    {
        std_msgs::Bool msg;
        msg.data = true;

        charge_pub_.publish(msg);
        ROS_INFO("📤 已发布 /charge_command = true");
    }


    void chargeCompleteCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (current_state_ != State::CHARGING)
        {
            return;
        }

        if (msg->data)
        {
            ROS_INFO("✅ 收到充电成功消息，准备返回出发区");
            returnToStart();
        }
        else
        {
            ROS_ERROR("❌ 自动充电节点报告充电失败，进入ERROR状态");
            transitionTo(State::ERROR);
        }
    }


    void returnToStart()
    {
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

        ROS_INFO("🏠 已发送返回出发区目标: (%.2f, %.2f)", START_X, START_Y);
    }


    // =========================================================
    // 状态工具
    // =========================================================

    bool isTask3() const
    {
        return (
            current_task_ == TaskType::TASK3_FIND_PHONE
            ||
            current_task_ == TaskType::TASK3_FIND_BACKPACK
        );
    }


    void resetTaskRuntime()
    {
        current_task_ = TaskType::NONE;
        current_index_ = 0;
        task3_target_found_current_ = false;
        speech_finished_ = false;
        waypoint_timer_started_ = false;
        face_detected_ = false;
        voice_command_received_ = false;
    }


    void transitionTo(State new_state)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (current_state_ == new_state)
        {
            return;
        }

        ROS_INFO(
            "🔄 状态转换: %s -> %s",
            stateToString(current_state_).c_str(),
            stateToString(new_state).c_str()
        );

        current_state_ = new_state;
    }


    std::string stateToString(State state) const
    {
        switch (state)
        {
            case State::WAITING_START:
                return "等待启动";

            case State::GOING_TO_LOBBY:
                return "前往走廊";

            case State::WAITING_FACE:
                return "等待人脸识别";

            case State::IDLE:
                return "等待用户指令";

            case State::NAVIGATING:
                return "导航中";

            case State::TASK3_DETECTING:
                return "任务三识别中";

            case State::WAITING_SPEECH:
                return "地点任务/语音处理中";

            case State::CHARGING:
                return "任务三充电中";

            case State::RETURNING:
                return "返回出发区中";

            case State::ERROR:
                return "错误状态";

            default:
                return "未知状态";
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
        setTask3Detection(false);

        transitionTo(State::GOING_TO_LOBBY);

        if (!waitForNavGoalReady(20.0))
        {
            transitionTo(State::ERROR);
            return;
        }

        sendLobbyGoal();

        ros::Rate loop_rate(LOOP_RATE_HZ);

        int timeout_counter = 0;
        int waiting_face_counter = 0;
        State last_state = current_state_;

        while (ros::ok())
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

                    if (
                        timeout_counter
                        >
                        NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR("⏰ 前往走廊超时，重新发送目标");
                        timeout_counter = 0;
                        sendLobbyGoal();
                    }

                    break;
                }

                case State::WAITING_FACE:
                {
                    waiting_face_counter++;

                    if (
                        waiting_face_counter
                        %
                        (10 * LOOP_RATE_HZ)
                        ==
                        0
                    )
                    {
                        ROS_INFO("👤 等待人脸识别唤醒...");
                    }

                    break;
                }

                case State::IDLE:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        %
                        (IDLE_REMIND_INTERVAL * LOOP_RATE_HZ)
                        ==
                        0
                    )
                    {
                        ROS_INFO("💬 可说：参观 / 帮我找一下手机 / 帮我找一下书包");
                    }

                    break;
                }

                case State::NAVIGATING:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        >
                        NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR("⏰ 导航超时");
                        timeout_counter = 0;
                        handleNavigationFailure();
                    }

                    break;
                }

                case State::TASK3_DETECTING:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        >
                        TASK3_DETECTION_TIMEOUT_SECONDS * LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR("⏰ 任务三视觉识别超时，没有收到 /task3_detection_result");
                        setTask3Detection(false);
                        transitionTo(State::ERROR);
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
                    timeout_counter++;

                    ROS_INFO_THROTTLE(
                        10.0,
                        "🔋 等待自动充电节点完成..."
                    );

                    if (
                        timeout_counter
                        >
                        CHARGE_TIMEOUT_SECONDS * LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR("⏰ 自动充电超过 %d 秒，进入ERROR状态", CHARGE_TIMEOUT_SECONDS);
                        transitionTo(State::ERROR);
                    }

                    break;
                }

                case State::RETURNING:
                {
                    timeout_counter++;

                    ROS_INFO_THROTTLE(
                        5.0,
                        "🔄 机器人正在返回出发区..."
                    );

                    if (
                        timeout_counter
                        >
                        NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR("⏰ 返回出发区超时，重新发送返回目标");
                        timeout_counter = 0;
                        returnToStart();
                    }

                    break;
                }

                case State::ERROR:
                {
                    ROS_WARN_THROTTLE(
                        5.0,
                        "⛔ 系统处于错误状态，请查看前面的ERROR日志"
                    );

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
    }
};


int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(
        argc,
        argv,
        "task_scheduler"
    );

    ROS_INFO("智能服务机器人任务系统");

    TaskScheduler scheduler;
    scheduler.start();

    return 0;
}