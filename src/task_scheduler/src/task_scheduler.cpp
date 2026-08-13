/**
 * @file task_scheduler.cpp
 * @brief 服务机器人任务调度：任务一导览 + 任务三五点巡检
 *
 * 最终流程：
 *
 * 任务一：
 *   走廊等待人脸/语音
 *   -> 餐厅 -> 厨房 -> 客厅 -> 卧室
 *   -> 返回出发点
 *   -> 再去走廊等待下一任务
 *
 *   任务一只要求“到达位置”，不要求到点后调整最终朝向。
 *
 * 任务三：
 *   走廊等待人脸/语音
 *   -> 餐厅 -> 厨房 -> 客厅 -> 卧室 -> 走廊
 *   每个点都进行 1.5 秒多帧目标检测并播报结果
 *   即使中途已经找到目标，也绝不提前结束巡检
 *   -> 五点全部完成后充电
 *   -> 返回出发点
 *   -> 再去走廊等待下一任务
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
#include <sstream>


struct Waypoint
{
    std::string name;
    double x;
    double y;

    // 仅任务三使用的最终检测朝向
    double task3_angle;

    // 仅任务一使用的地点介绍
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
    // 路线 / 任务
    // =========================================================

    std::vector<Waypoint> route_;

    // route_ 有 5 个点，但任务一只走前 4 个
    static constexpr int TASK1_ROUTE_COUNT = 4;

    int current_index_ = 0;

    TaskType current_task_ = TaskType::NONE;
    State current_state_ = State::WAITING_START;


    // =========================================================
    // 状态变量
    // =========================================================

    std::atomic<bool> speech_finished_{false};
    std::atomic<bool> voice_command_received_{false};
    std::atomic<bool> face_detected_{false};

    // 只用于记录/日志。
    // 注意：任务三找到目标后也不会提前结束。
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

    // detector 正常约 1.5 秒返回。
    // 10 秒只是视觉链路故障保护。
    const int TASK3_DETECTION_TIMEOUT_SECONDS = 10;

    // 自动充电包含导航、AR二次定位、模拟30秒充电等。
    const int CHARGE_TIMEOUT_SECONDS = 180;

    // 正常任务三 / 返回 / 走廊导航使用的最终角度容差
    const double NORMAL_YAW_GOAL_TOLERANCE = 0.05;

    // 任务一只判断“位置到达”，不要求最终角度。
    // 任意两个平面 yaw 的最短角差不会超过 pi。
    const double TASK1_YAW_GOAL_TOLERANCE = M_PI;

    // 当前工程实际 move_base 节点名是 /move_base_node
    std::string dwa_namespace_ = "/move_base_node/DWAPlannerROS";


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
        nh_.param<std::string>(
            "dwa_namespace",
            dwa_namespace_,
            "/move_base_node/DWAPlannerROS"
        );

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/nav_goal",
            10
        );

        charge_pub_ = nh_.advertise<std_msgs::Bool>(
            "/charge_command",
            10
        );

        // latched：后启动节点也能拿到最新开关状态
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
        ROS_INFO("✅ 任务一：4点导览，位置到达即可，不要求最终朝向，不充电");
        ROS_INFO("✅ 任务三：5点全部巡检，中途找到目标也继续，最后充电");
        ROS_INFO("📍 任务一：餐厅 -> 厨房 -> 客厅 -> 卧室");
        ROS_INFO("📍 任务三：餐厅 -> 厨房 -> 客厅 -> 卧室 -> 走廊");
        ROS_INFO("========================================");
    }


    // =========================================================
    // 路线
    // =========================================================

    void initRoute()
    {
        // 完全按照用户给出的数值定义。
        // 这里不重新解释“朝北/朝南”，只使用给出的 yaw 数值。
        const double ANGLE_0 = 0.0;
        const double ANGLE_90 = -M_PI / 2.0;
        const double ANGLE_NEG_90 = M_PI / 2.0;
        const double ANGLE_180 = M_PI;

        route_ = {
            {
                "餐厅",
                2.220,
                1.034,
                ANGLE_180,
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
            },
            {
                "走廊",
                1.241,
                2.137,
                ANGLE_NEG_90,
                ""
            }
        };

        (void)ANGLE_0;
    }


    // =========================================================
    // DWA 最终角度容差控制
    //
    // 任务一：
    // yaw_goal_tolerance = pi
    // -> 到达 xy 后不再为了最终朝向原地调整
    //
    // 任务三/走廊/返回：
    // 恢复 0.05 rad
    // =========================================================

    bool setDwaYawGoalTolerance(double tolerance)
    {
        ros::param::set(
            dwa_namespace_ + "/yaw_goal_tolerance",
            tolerance
        );

        std::ostringstream cmd;

        cmd
            << "rosrun dynamic_reconfigure dynparam set "
            << dwa_namespace_
            << " yaw_goal_tolerance "
            << tolerance
            << " >/dev/null 2>&1";

        const int ret = std::system(
            cmd.str().c_str()
        );

        if (ret != 0)
        {
            ROS_WARN(
                "⚠️ 动态设置 DWA yaw_goal_tolerance 失败，"
                "namespace=%s tolerance=%.3f。"
                "请检查 dynamic_reconfigure / DWA namespace。",
                dwa_namespace_.c_str(),
                tolerance
            );

            return false;
        }

        ROS_INFO(
            "🧭 DWA yaw_goal_tolerance 已设为 %.3f rad (%.1f°)",
            tolerance,
            tolerance * 180.0 / M_PI
        );

        return true;
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

        // 同一句同时出现手机+书包时不猜测目标
        if (has_phone && has_backpack)
        {
            setVoiceListening(false);
            speakSync("请告诉我需要找手机还是书包");
            setVoiceListening(true);
            return;
        }

        // -----------------------------------------------------
        // 任务三
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

            // 任务三必须严格执行每个地点指定角度
            setDwaYawGoalTolerance(
                NORMAL_YAW_GOAL_TOLERANCE
            );

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

            // 保留：完整播完“好的，请跟我来”后再起步
            speakSync("好的，请跟我来");

            // 核心修改：
            // 任务一只要求到达 xy，不要求到点后旋转到指定 yaw。
            setDwaYawGoalTolerance(
                TASK1_YAW_GOAL_TOLERANCE
            );

            startCurrentTaskNavigation();
            return;
        }

        // 无效指令：重新做人脸唤醒
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

        const int ret = std::system(
            cmd.c_str()
        );

        if (ret != 0)
        {
            ROS_WARN(
                "⚠️ 语音播报异常，返回码: %d",
                ret
            );
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

                const int ret = std::system(
                    cmd.c_str()
                );

                if (ret == 0)
                {
                    ROS_INFO(
                        "✅ 语音完整结束: %s",
                        name.c_str()
                    );
                }
                else
                {
                    ROS_WARN(
                        "⚠️ %s 语音播报异常，返回码: %d",
                        name.c_str(),
                        ret
                    );
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
        ROS_INFO(
            "✅ 已到达走廊等待位置 (%.3f, %.3f)",
            LOBBY_X,
            LOBBY_Y
        );

        resetTaskRuntime();

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
        ROS_INFO("📍 准备再次前往走廊等待下一条任务");
        ROS_INFO("========================================");

        resetTaskRuntime();

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        transitionTo(State::GOING_TO_LOBBY);
        sendLobbyGoal();
    }


    // =========================================================
    // 到达任务地点
    // =========================================================

    void handleWaypointArrival()
    {
        const int valid_count = currentRouteCount();

        if (
            current_index_ < 0
            ||
            current_index_ >= valid_count
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

        // 到达成功这一刻立即开始算至少5秒
        waypoint_arrival_time_ =
            std::chrono::steady_clock::now();

        waypoint_timer_started_ = true;

        ROS_INFO(
            "⏱️ %s：从现在开始计算至少5秒停留时间",
            wp.name.c_str()
        );

        // -----------------------------------------------------
        // 任务一：
        // 到达位置后立即开始介绍。
        // 无额外角度调整阶段。
        // -----------------------------------------------------
        if (current_task_ == TaskType::TASK1_GUIDE)
        {
            ROS_INFO(
                "🧭 任务一到点后不调整角度，直接开始地点介绍"
            );

            transitionTo(State::WAITING_SPEECH);

            speakAsync(
                wp.description,
                wp.name
            );

            return;
        }

        // -----------------------------------------------------
        // 任务三：
        // move_base 已按照 task3_angle 完成指定姿态，
        // 然后开启检测。
        // -----------------------------------------------------
        if (isTask3())
        {
            task3_target_found_current_ = false;
            speech_finished_ = false;

            transitionTo(
                State::TASK3_DETECTING
            );

            ROS_INFO(
                "📷 %s 已按任务三指定姿态到达，开启1.5秒多帧检测",
                wp.name.c_str()
            );

            setTask3Detection(true);
            return;
        }

        ROS_ERROR(
            "❌ 到达目标点时没有有效任务类型"
        );

        transitionTo(State::ERROR);
    }


    // =========================================================
    // 任务三视觉结果
    // detector:
    // none / phone / backpack / both / error
    // =========================================================

    void task3DetectionResultCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        if (
            current_state_
            !=
            State::TASK3_DETECTING
        )
        {
            ROS_WARN_THROTTLE(
                2.0,
                "⚠️ 当前不在任务三检测阶段，忽略视觉结果: %s",
                msg->data.c_str()
            );

            return;
        }

        setTask3Detection(false);

        const std::string result =
            msg->data;

        ROS_INFO("========================================");
        ROS_INFO(
            "📷 %s 多帧识别结果: [%s]",
            route_[current_index_].name.c_str(),
            result.c_str()
        );

        if (result == "error")
        {
            ROS_ERROR(
                "❌ 任务三视觉节点没有获得足够有效图像"
            );

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

        // -----------------------------------------------------
        // 用户要找手机
        // -----------------------------------------------------
        if (
            current_task_
            ==
            TaskType::TASK3_FIND_PHONE
        )
        {
            if (has_phone)
            {
                task3_target_found_current_ = true;

                // 如果 both，也优先播目标
                speech =
                    "找到手机啦，在这里！";
            }
            else if (has_backpack)
            {
                task3_target_found_current_ = false;
                speech =
                    "我看到了书包";
            }
            else
            {
                task3_target_found_current_ = false;
                speech =
                    "这里什么都没有";
            }
        }

        // -----------------------------------------------------
        // 用户要找书包
        // -----------------------------------------------------
        else if (
            current_task_
            ==
            TaskType::TASK3_FIND_BACKPACK
        )
        {
            if (has_backpack)
            {
                task3_target_found_current_ = true;

                // 如果 both，也优先播目标
                speech =
                    "找到书包啦，在这里！";
            }
            else if (has_phone)
            {
                task3_target_found_current_ = false;
                speech =
                    "我看到了手机";
            }
            else
            {
                task3_target_found_current_ = false;
                speech =
                    "这里什么都没有";
            }
        }
        else
        {
            ROS_ERROR(
                "❌ 收到任务三视觉结果，但当前不是任务三"
            );

            transitionTo(State::ERROR);
            return;
        }

        ROS_INFO(
            "🎯 本地点是否包含用户指定目标: %s",
            task3_target_found_current_
                ? "是"
                : "否"
        );

        if (task3_target_found_current_)
        {
            ROS_INFO(
                "ℹ️ 已找到目标，但按新规则仍继续后续所有巡检点"
            );
        }

        ROS_INFO(
            "🔊 本地点将播报: %s",
            speech.c_str()
        );

        ROS_INFO("========================================");

        transitionTo(
            State::WAITING_SPEECH
        );

        speakAsync(
            speech,
            route_[current_index_].name
            +
            "巡检结果"
        );
    }


    // =========================================================
    // 5秒规则
    // =========================================================

    void checkSpeechAndProceed()
    {
        if (
            current_state_
            !=
            State::WAITING_SPEECH
        )
        {
            return;
        }

        if (!waypoint_timer_started_)
        {
            return;
        }

        const auto now =
            std::chrono::steady_clock::now();

        const double elapsed_seconds =
            std::chrono::duration<double>(
                now - waypoint_arrival_time_
            ).count();

        const bool speech_done =
            speech_finished_.load();

        const bool minimum_stop_done =
            elapsed_seconds
            >=
            MIN_WAYPOINT_STOP_SECONDS;

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
                MIN_WAYPOINT_STOP_SECONDS
                -
                elapsed_seconds
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
        // 任务一：
        // 只走前4点。
        // 卧室完成后直接返回出发点，不去第五点走廊巡检。
        // -----------------------------------------------------
        if (
            current_task_
            ==
            TaskType::TASK1_GUIDE
        )
        {
            current_index_++;

            if (
                current_index_
                >=
                TASK1_ROUTE_COUNT
            )
            {
                handleTask1Complete();
            }
            else
            {
                transitionTo(
                    State::NAVIGATING
                );

                sendNextGoal();
            }

            return;
        }


        // -----------------------------------------------------
        // 任务三：
        //
        // 无论这一点是否已经找到目标，都必须继续。
        //
        // 餐厅 -> 厨房 -> 客厅 -> 卧室 -> 走廊
        //
        // 第五点走廊完成检测+播报+5秒规则以后才充电。
        // -----------------------------------------------------
        if (isTask3())
        {
            current_index_++;

            if (
                current_index_
                >=
                static_cast<int>(
                    route_.size()
                )
            )
            {
                ROS_INFO(
                    "🏁 任务三五个地点已全部巡检完成"
                );

                beginCharging();
            }
            else
            {
                ROS_INFO(
                    "➡️ 不论是否已发现目标，继续巡检下一地点"
                );

                transitionTo(
                    State::NAVIGATING
                );

                sendNextGoal();
            }

            return;
        }

        ROS_ERROR(
            "❌ WAITING_SPEECH阶段没有有效任务类型"
        );

        transitionTo(State::ERROR);
    }


    // =========================================================
    // 导航失败
    // =========================================================

    void handleNavigationFailure()
    {
        const int valid_count =
            currentRouteCount();

        if (
            current_index_ < 0
            ||
            current_index_ >= valid_count
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

        if (current_index_ < valid_count)
        {
            ROS_WARN(
                "🔄 跳过当前失败地点，继续下一地点"
            );

            transitionTo(
                State::NAVIGATING
            );

            sendNextGoal();

            return;
        }

        if (
            current_task_
            ==
            TaskType::TASK1_GUIDE
        )
        {
            ROS_WARN(
                "⚠️ 任务一剩余地点结束/失败，本轮导览结束，返回出发区"
            );

            handleTask1Complete();
            return;
        }

        if (isTask3())
        {
            ROS_WARN(
                "⚠️ 任务三五点已完成/失败，进入充电流程"
            );

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
        // 正常走廊等待点恢复标准角度要求
        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );

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

        ROS_INFO(
            "📤 走廊等待目标已发布: (%.3f, %.3f)",
            LOBBY_X,
            LOBBY_Y
        );
    }


    void sendNextGoal()
    {
        const int valid_count =
            currentRouteCount();

        if (
            current_index_ < 0
            ||
            current_index_ >= valid_count
            ||
            current_index_
            >=
            static_cast<int>(
                route_.size()
            )
        )
        {
            ROS_ERROR(
                "❌ 目标索引越界，无法发送导航目标"
            );

            transitionTo(State::ERROR);
            return;
        }

        if (
            current_state_
            !=
            State::NAVIGATING
        )
        {
            transitionTo(
                State::NAVIGATING
            );
        }

        const Waypoint& wp =
            route_[current_index_];

        geometry_msgs::PoseStamped goal;

        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = 0.0;

        // -----------------------------------------------------
        // 任务三：严格使用用户给出的目标角度
        // -----------------------------------------------------
        if (isTask3())
        {
            const double goal_angle =
                wp.task3_angle;

            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z =
                std::sin(
                    goal_angle / 2.0
                );

            goal.pose.orientation.w =
                std::cos(
                    goal_angle / 2.0
                );

            ROS_INFO("========================================");
            ROS_INFO(
                "📤 任务三第 %d/%d 个巡检点: %s (%.3f, %.3f)",
                current_index_ + 1,
                valid_count,
                wp.name.c_str(),
                wp.x,
                wp.y
            );

            ROS_INFO(
                "🧭 任务三指定最终 yaw: %.1f°",
                goal_angle
                *
                180.0
                /
                M_PI
            );

            ROS_INFO("========================================");
        }

        // -----------------------------------------------------
        // 任务一：
        //
        // orientation 本身填单位四元数只是为了给 move_base 一个合法目标。
        // 真正“不要求最终角度”由 yaw_goal_tolerance=pi 实现。
        // -----------------------------------------------------
        else
        {
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;

            ROS_INFO("========================================");
            ROS_INFO(
                "📤 任务一第 %d/%d 个导览点: %s (%.3f, %.3f)",
                current_index_ + 1,
                valid_count,
                wp.name.c_str(),
                wp.x,
                wp.y
            );

            ROS_INFO(
                "🧭 任务一仅要求位置到达，不要求到点后调整最终朝向"
            );

            ROS_INFO("========================================");
        }

        goal_pub_.publish(goal);
    }


    void startCurrentTaskNavigation()
    {
        current_index_ = 0;

        task3_target_found_current_ = false;
        speech_finished_ = false;
        waypoint_timer_started_ = false;

        transitionTo(
            State::NAVIGATING
        );

        sendNextGoal();
    }


    // =========================================================
    // 任务完成与充电
    // =========================================================

    void handleTask1Complete()
    {
        ROS_INFO("========================================");
        ROS_INFO("🏁 任务一四点导览完成");
        ROS_INFO("🚫 任务一不去第五个走廊巡检点");
        ROS_INFO("🚫 任务一不充电");
        ROS_INFO("🏠 直接返回出发区");
        ROS_INFO("========================================");

        // 返回出发点恢复正常 yaw 容差
        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );

        returnToStart();
    }


    void beginCharging()
    {
        if (!isTask3())
        {
            ROS_ERROR(
                "❌ 非任务三禁止进入充电流程"
            );

            transitionTo(State::ERROR);
            return;
        }

        ROS_INFO("========================================");
        ROS_INFO("🔋 任务三五点巡检全部结束");
        ROS_INFO("🔋 现在开始充电流程");
        ROS_INFO("========================================");

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );

        // 先切状态再发命令
        transitionTo(
            State::CHARGING
        );

        sendChargeCommand();
    }


    void sendChargeCommand()
    {
        std_msgs::Bool msg;
        msg.data = true;

        charge_pub_.publish(msg);

        ROS_INFO(
            "📤 已发布 /charge_command = true"
        );
    }


    void chargeCompleteCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        if (
            current_state_
            !=
            State::CHARGING
        )
        {
            return;
        }

        if (msg->data)
        {
            ROS_INFO(
                "✅ 收到充电成功消息，准备返回出发区"
            );

            returnToStart();
        }
        else
        {
            ROS_ERROR(
                "❌ 自动充电节点报告充电失败，进入ERROR状态"
            );

            transitionTo(State::ERROR);
        }
    }


    void returnToStart()
    {
        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );

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

        transitionTo(
            State::RETURNING
        );

        goal_pub_.publish(home);

        ROS_INFO(
            "🏠 已发送返回出发区目标: (%.2f, %.2f)",
            START_X,
            START_Y
        );
    }


    // =========================================================
    // 状态工具
    // =========================================================

    bool isTask3() const
    {
        return (
            current_task_
            ==
            TaskType::TASK3_FIND_PHONE
            ||
            current_task_
            ==
            TaskType::TASK3_FIND_BACKPACK
        );
    }


    int currentRouteCount() const
    {
        if (
            current_task_
            ==
            TaskType::TASK1_GUIDE
        )
        {
            return TASK1_ROUTE_COUNT;
        }

        if (isTask3())
        {
            return static_cast<int>(
                route_.size()
            );
        }

        return 0;
    }


    void resetTaskRuntime()
    {
        current_task_ =
            TaskType::NONE;

        current_index_ = 0;

        task3_target_found_current_ =
            false;

        speech_finished_ =
            false;

        waypoint_timer_started_ =
            false;

        face_detected_ =
            false;

        voice_command_received_ =
            false;
    }


    void transitionTo(State new_state)
    {
        std::lock_guard<std::mutex>
            lock(state_mutex_);

        if (
            current_state_
            ==
            new_state
        )
        {
            return;
        }

        ROS_INFO(
            "🔄 状态转换: %s -> %s",
            stateToString(
                current_state_
            ).c_str(),
            stateToString(
                new_state
            ).c_str()
        );

        current_state_ =
            new_state;
    }


    std::string stateToString(
        State state
    ) const
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
        ROS_INFO(
            "▶️ 开始执行任务调度"
        );

        ros::WallDuration(
            1.0
        ).sleep();

        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);

        // 初始导航使用正常角度容差
        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );

        transitionTo(
            State::GOING_TO_LOBBY
        );

        if (
            !waitForNavGoalReady(
                20.0
            )
        )
        {
            transitionTo(
                State::ERROR
            );

            return;
        }

        sendLobbyGoal();

        ros::Rate loop_rate(
            LOOP_RATE_HZ
        );

        int timeout_counter = 0;
        int waiting_face_counter = 0;

        State last_state =
            current_state_;

        while (ros::ok())
        {
            ros::spinOnce();

            if (
                current_state_
                !=
                last_state
            )
            {
                timeout_counter = 0;
                waiting_face_counter = 0;
                last_state = current_state_;
            }

            switch (
                current_state_
            )
            {
                case State::GOING_TO_LOBBY:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        >
                        NAV_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR(
                            "⏰ 前往走廊超时，重新发送目标"
                        );

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
                        (
                            10
                            *
                            LOOP_RATE_HZ
                        )
                        ==
                        0
                    )
                    {
                        ROS_INFO(
                            "👤 等待人脸识别唤醒..."
                        );
                    }

                    break;
                }


                case State::IDLE:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        %
                        (
                            IDLE_REMIND_INTERVAL
                            *
                            LOOP_RATE_HZ
                        )
                        ==
                        0
                    )
                    {
                        ROS_INFO(
                            "💬 可说：参观 / 帮我找一下手机 / 帮我找一下书包"
                        );
                    }

                    break;
                }


                case State::NAVIGATING:
                {
                    timeout_counter++;

                    if (
                        timeout_counter
                        >
                        NAV_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR(
                            "⏰ 导航超时"
                        );

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
                        TASK3_DETECTION_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR(
                            "⏰ 任务三视觉识别超时，"
                            "没有收到 /task3_detection_result"
                        );

                        setTask3Detection(false);

                        transitionTo(
                            State::ERROR
                        );
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
                        CHARGE_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR(
                            "⏰ 自动充电超过 %d 秒，进入ERROR状态",
                            CHARGE_TIMEOUT_SECONDS
                        );

                        transitionTo(
                            State::ERROR
                        );
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
                        NAV_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        ROS_ERROR(
                            "⏰ 返回出发区超时，重新发送返回目标"
                        );

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


int main(
    int argc,
    char** argv
)
{
    setlocale(
        LC_CTYPE,
        "zh_CN.utf8"
    );

    ros::init(
        argc,
        argv,
        "task_scheduler"
    );

    ROS_INFO(
        "智能服务机器人任务系统"
    );

    TaskScheduler scheduler;

    scheduler.start();

    return 0;
}
