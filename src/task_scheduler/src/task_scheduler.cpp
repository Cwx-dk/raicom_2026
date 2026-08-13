/**
 * @file task_scheduler.cpp
 * @brief 服务机器人三任务调度：
 *        任务一导览 + 任务二菜品推荐 + 任务三五点找物巡检
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

    // 仅任务三使用
    double task3_angle;

    // 仅任务一使用
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

        TASK2_DETECTING,
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

        TASK2_RECIPE,

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

    ros::Publisher task2_detection_enable_pub_;
    ros::Publisher task3_detection_enable_pub_;

    ros::Subscriber status_sub_;
    ros::Subscriber voice_sub_;
    ros::Subscriber face_sub_;

    ros::Subscriber charge_complete_sub_;

    ros::Subscriber task2_recipe_result_sub_;
    ros::Subscriber task3_detection_result_sub_;


    // =========================================================
    // 路线
    // =========================================================

    std::vector<Waypoint> route_;

    static constexpr int TASK1_ROUTE_COUNT = 4;

    int current_index_ = 0;

    TaskType current_task_ = TaskType::NONE;
    State current_state_ = State::WAITING_START;


    // =========================================================
    // 状态
    // =========================================================

    std::atomic<bool> speech_finished_{false};
    std::atomic<bool> voice_command_received_{false};
    std::atomic<bool> face_detected_{false};

    bool task3_target_found_current_ = false;


    // =========================================================
    // 任务一 / 三的地点停留计时
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

    const int TASK3_DETECTION_TIMEOUT_SECONDS = 10;
    const int CHARGE_TIMEOUT_SECONDS = 180;

    const double NORMAL_YAW_GOAL_TOLERANCE = 0.05;

    // 任务一和任务二都只要求 XY 到达，不要求最终 yaw
    const double POSITION_ONLY_YAW_GOAL_TOLERANCE = M_PI;

    std::string dwa_namespace_ =
        "/move_base_node/DWAPlannerROS";


    // =========================================================
    // 固定坐标
    // =========================================================

    const double LOBBY_X = 1.231;
    const double LOBBY_Y = 2.142;

    const double START_X = 0.0;
    const double START_Y = 0.0;

    const double KITCHEN_X = 1.251;
    const double KITCHEN_Y = 1.013;


public:

    TaskScheduler()
    {
        nh_.param<std::string>(
            "dwa_namespace",
            dwa_namespace_,
            "/move_base_node/DWAPlannerROS"
        );


        goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(
                "/nav_goal",
                10
            );

        charge_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/charge_command",
                10
            );


        // latched 控制 Topic
        face_enable_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/face_detection_enable",
                1,
                true
            );

        voice_enable_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/voice_listen_enable",
                1,
                true
            );

        task2_detection_enable_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/task2_detection_enable",
                1,
                true
            );

        task3_detection_enable_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/task3_detection_enable",
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

        task2_recipe_result_sub_ =
            nh_.subscribe(
                "/task2_recipe_result",
                10,
                &TaskScheduler::task2RecipeResultCallback,
                this
            );

        task3_detection_result_sub_ =
            nh_.subscribe(
                "/task3_detection_result",
                10,
                &TaskScheduler::task3DetectionResultCallback,
                this
            );


        initRoute();


        ROS_INFO("========================================");
        ROS_INFO("🚀 三任务调度器已启动");
        ROS_INFO("✅ 任务一：4点导览，不要求最终角度，不充电");
        ROS_INFO("✅ 任务二：厨房现实摄像头识别3种食材 -> 推荐菜品 -> 回起点");
        ROS_INFO("✅ 任务三：5点全部巡检，中途找到目标也继续 -> 最后充电");
        ROS_INFO("💬 关键词：参观 / 做什么菜 / 手机 / 书包");
        ROS_INFO("========================================");
    }


    // =========================================================
    // 任务一 / 三路线
        // =========================================================

    void initRoute()
        {
            // 完全沿用用户给出的数值
            const double ANGLE_90 = -M_PI / 2.0;
            const double ANGLE_NEG_90 = M_PI / 2.0;
            const double ANGLE_180 = M_PI;

            route_ = {
                // {
                //     "途经点",
                //     2.361,
                //     2.180,
                //     ANGLE_180,
                //     ""
                // },
                {
                    "餐厅",
                    2.240,
                    1.034,
                    ANGLE_180,
                    "这里是餐厅"
                },
                {
                    "厨房",
                    1.261,
                    1.013,
                    ANGLE_90,
                    "这里是厨房"
                },
                {
                    "客厅",
                    1.267,
                    -0.020,
                    ANGLE_NEG_90,
                    "这里是客厅"
                },
                {
                    "卧室",
                    2.252,
                    -0.020,
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
        }


    // =========================================================
    // DWA yaw tolerance
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

        const int ret =
            std::system(
                cmd.str().c_str()
            );

        if (ret != 0)
        {
            ROS_WARN(
                "⚠️ 动态设置 DWA yaw_goal_tolerance 失败："
                "namespace=%s tolerance=%.3f",
                dwa_namespace_.c_str(),
                tolerance
            );

            return false;
        }

        ROS_INFO(
            "🧭 yaw_goal_tolerance = %.3f rad (%.1f°)",
            tolerance,
            tolerance * 180.0 / M_PI
        );

        return true;
    }


    // =========================================================
    // 控制 Topic
    // =========================================================

    void setFaceDetection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        face_enable_pub_.publish(msg);

        ROS_INFO(
            "👤 人脸识别: %s",
            enable ? "开启" : "关闭"
        );
    }


    void setVoiceListening(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        voice_enable_pub_.publish(msg);

        ROS_INFO(
            "🎤 语音监听: %s",
            enable ? "开启" : "关闭"
        );
    }


    void setTask2Detection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        task2_detection_enable_pub_.publish(msg);

        ROS_INFO(
            "🥬 任务二食材识别: %s",
            enable ? "开启" : "关闭"
        );
    }


    void setTask3Detection(bool enable)
    {
        std_msgs::Bool msg;
        msg.data = enable;
        task3_detection_enable_pub_.publish(msg);

        ROS_INFO(
            "📷 任务三目标识别: %s",
            enable ? "开启" : "关闭"
        );
    }


    // =========================================================
    // nav_goal_node ready
    // =========================================================

    bool waitForNavGoalReady(
        double timeout_seconds = 20.0
    )
    {
        ROS_INFO(
            "⏳ 等待 nav_goal_node 准备完成..."
        );

        ros::WallTime start_time =
            ros::WallTime::now();

        ros::WallRate rate(10.0);

        while (
            ros::ok()
            &&
            goal_pub_.getNumSubscribers() == 0
        )
        {
            const double elapsed =
                (
                    ros::WallTime::now()
                    -
                    start_time
                ).toSec();

            if (
                elapsed
                >=
                timeout_seconds
            )
            {
                ROS_ERROR(
                    "❌ %.1f秒内 /nav_goal 没有订阅者",
                    timeout_seconds
                );

                return false;
            }

            ROS_INFO_THROTTLE(
                2.0,
                "⏳ /nav_goal 暂无订阅者..."
            );

            rate.sleep();
        }

        if (!ros::ok())
        {
            return false;
        }

        ROS_INFO(
            "✅ nav_goal_node 已就绪"
        );

        return true;
    }


    // =========================================================
    // 人脸
    // =========================================================

    void faceCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        if (!msg->data)
        {
            return;
        }

        if (
            current_state_
            !=
            State::WAITING_FACE
        )
        {
            return;
        }


        ROS_INFO("========================================");
        ROS_INFO("👤 检测到用户人脸");


        transitionTo(
            State::IDLE
        );

        face_detected_ = true;


        // 这里只暂停人脸识别算法；人脸摄像头仍保持实时画面
        setFaceDetection(false);

        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);


        speakSync(
            "你好，需要帮助吗？"
        );


        setVoiceListening(true);

        ROS_INFO(
            "🎧 等待用户语音..."
        );

        ROS_INFO("========================================");
    }


    // =========================================================
    // 语音任务分流
    // =========================================================

    void voiceCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        const std::string text =
            msg->data;


        ROS_INFO("========================================");
        ROS_INFO(
            "🎤 收到用户语音: [%s]",
            text.c_str()
        );

        ROS_INFO(
            "📊 当前状态: [%s]",
            stateToString(
                current_state_
            ).c_str()
        );

        ROS_INFO("========================================");


        if (
            current_state_
            !=
            State::IDLE
        )
        {
            ROS_WARN(
                "⚠️ 当前不在等待命令状态，忽略"
            );

            return;
        }


        const bool has_guide =
            containsText(
                text,
                "参观"
            );

        const bool has_recipe =
            containsText(
                text,
                "做什么菜"
            );

        const bool has_phone =
            containsText(
                text,
                "手机"
            );

        const bool has_backpack =
            containsText(
                text,
                "书包"
            );


        // -----------------------------------------------------
        // 任务二：推荐做什么菜
        // -----------------------------------------------------

        if (has_recipe)
        {
            if (
                voice_command_received_
                .exchange(true)
            )
            {
                return;
            }


            current_task_ =
                TaskType::TASK2_RECIPE;


            setVoiceListening(false);
            setFaceDetection(false);
            setTask2Detection(false);
            setTask3Detection(false);


            ROS_INFO(
                "✅ 触发任务二：菜品推荐"
            );


            // 三个任务都必须先说
            speakSync(
                "好的，请跟我来"
            );


            // 任务二只要求到达厨房 XY
            setDwaYawGoalTolerance(
                POSITION_ONLY_YAW_GOAL_TOLERANCE
            );


            startTask2Navigation();

            return;
        }


        // -----------------------------------------------------
        // 手机+书包同时出现时避免猜测
        // -----------------------------------------------------

        if (
            has_phone
            &&
            has_backpack
        )
        {
            setVoiceListening(false);

            speakSync(
                "请告诉我需要找手机还是书包"
            );

            setVoiceListening(true);

            return;
        }


        // -----------------------------------------------------
        // 任务三
        // -----------------------------------------------------

        if (
            has_phone
            ||
            has_backpack
        )
        {
            if (
                voice_command_received_
                .exchange(true)
            )
            {
                return;
            }


            setVoiceListening(false);
            setFaceDetection(false);
            setTask2Detection(false);
            setTask3Detection(false);


            if (has_phone)
            {
                current_task_ =
                    TaskType::TASK3_FIND_PHONE;

                ROS_INFO(
                    "✅ 触发任务三：寻找手机"
                );
            }
            else
            {
                current_task_ =
                    TaskType::TASK3_FIND_BACKPACK;

                ROS_INFO(
                    "✅ 触发任务三：寻找书包"
                );
            }


            // 修复：
            // 任务三以前缺少这一句
            speakSync(
                "好的，请跟我来"
            );


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
            if (
                voice_command_received_
                .exchange(true)
            )
            {
                return;
            }


            current_task_ =
                TaskType::TASK1_GUIDE;


            setVoiceListening(false);
            setFaceDetection(false);
            setTask2Detection(false);
            setTask3Detection(false);


            ROS_INFO(
                "✅ 触发任务一：参观"
            );


            speakSync(
                "好的，请跟我来"
            );


            setDwaYawGoalTolerance(
                POSITION_ONLY_YAW_GOAL_TOLERANCE
            );


            startCurrentTaskNavigation();

            return;
        }


        // -----------------------------------------------------
        // 无效语音 -> 回到人脸唤醒
        // -----------------------------------------------------

        ROS_WARN(
            "⚠️ 不是有效任务关键词: [%s]",
            text.c_str()
        );


        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);

        face_detected_ = false;
        voice_command_received_ = false;

        current_task_ =
            TaskType::NONE;


        transitionTo(
            State::WAITING_FACE
        );

        setFaceDetection(true);
    }


    bool containsText(
        const std::string& text,
        const std::string& keyword
    )
    {
        return (
            text.find(keyword)
            !=
            std::string::npos
        );
    }


    // =========================================================
    // TTS
    // =========================================================

    void speakSync(
        const std::string& text
    )
    {
        ROS_INFO(
            "🔊 语音播报: %s",
            text.c_str()
        );


        std::string cmd =
            "/home/reicom2025/.local/bin/edge-playback "
            "--voice zh-CN-YunxiNeural "
            "--text \"" + text + "\" "
            "2>/dev/null";


        const int ret =
            std::system(
                cmd.c_str()
            );


        if (ret != 0)
        {
            ROS_WARN(
                "⚠️ TTS返回码: %d",
                ret
            );
        }
    }


    void speakAsync(
        const std::string& text,
        const std::string& name
    )
    {
        speech_finished_ =
            false;


        std::thread(
            [
                this,
                text,
                name
            ]()
            {
                ROS_INFO(
                    "🔊 异步播报: %s",
                    text.c_str()
                );


                std::string cmd =
                    "/home/reicom2025/.local/bin/edge-playback "
                    "--voice zh-CN-YunxiNeural "
                    "--text \"" + text + "\" "
                    "2>/dev/null";


                const int ret =
                    std::system(
                        cmd.c_str()
                    );


                if (ret != 0)
                {
                    ROS_WARN(
                        "⚠️ %s TTS异常: %d",
                        name.c_str(),
                        ret
                    );
                }


                speech_finished_ =
                    true;
            }
        ).detach();
    }


    // =========================================================
    // 导航反馈
    // =========================================================

    void statusCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        if (
            current_state_
            !=
            State::NAVIGATING
            &&
            current_state_
            !=
            State::GOING_TO_LOBBY
            &&
            current_state_
            !=
            State::RETURNING
        )
        {
            return;
        }


        ROS_INFO(
            "📩 导航反馈: %s",
            msg->data
                ? "成功"
                : "失败"
        );


        if (msg->data)
        {
            if (
                current_state_
                ==
                State::GOING_TO_LOBBY
            )
            {
                handleLobbyArrival();
                return;
            }


            if (
                current_state_
                ==
                State::NAVIGATING
            )
            {
                handleWaypointArrival();
                return;
            }


            if (
                current_state_
                ==
                State::RETURNING
            )
            {
                handleStartArrival();
                return;
            }
        }
        else
        {
            if (
                current_state_
                ==
                State::GOING_TO_LOBBY
            )
            {
                ROS_ERROR(
                    "❌ 前往走廊失败，1秒后重试"
                );

                ros::WallDuration(
                    1.0
                ).sleep();

                sendLobbyGoal();

                return;
            }


            if (
                current_state_
                ==
                State::NAVIGATING
            )
            {
                handleNavigationFailure();
                return;
            }


            if (
                current_state_
                ==
                State::RETURNING
            )
            {
                ROS_ERROR(
                    "❌ 返回出发点失败"
                );

                transitionTo(
                    State::ERROR
                );

                return;
            }
        }
    }


    // =========================================================
    // 走廊 / 出发点
    // =========================================================

    void handleLobbyArrival()
    {
        ROS_INFO("========================================");
        ROS_INFO(
            "✅ 已到达走廊等待位置 (%.3f, %.3f)",
            LOBBY_X,
            LOBBY_Y
        );


        resetTaskRuntime();


        setTask2Detection(false);
        setTask3Detection(false);

        setVoiceListening(false);


        transitionTo(
            State::WAITING_FACE
        );


        // 到走廊后重新开启人脸识别算法；摄像头本身一直保持打开
        setFaceDetection(true);


        ROS_INFO(
            "👤 开始等待人脸..."
        );

        ROS_INFO("========================================");
    }


    void handleStartArrival()
    {
        ROS_INFO("========================================");
        ROS_INFO(
            "✅ 已返回出发点"
        );

        ROS_INFO(
            "📍 现在再次前往走廊等待下一条命令"
        );

        ROS_INFO("========================================");


        resetTaskRuntime();


        setFaceDetection(false);
        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);


        transitionTo(
            State::GOING_TO_LOBBY
        );


        sendLobbyGoal();
    }


    // =========================================================
    // 到达任务地点
    // =========================================================

    void handleWaypointArrival()
    {
        // -----------------------------------------------------
        // 任务二只有厨房一个目标点
        // -----------------------------------------------------

        if (
            current_task_
            ==
            TaskType::TASK2_RECIPE
        )
        {
            handleTask2KitchenArrival();
            return;
        }


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
                "❌ 目标点索引越界"
            );

            transitionTo(
                State::ERROR
            );

            return;
        }


        const Waypoint& wp =
            route_[
                current_index_
            ];


        ROS_INFO("========================================");
        ROS_INFO(
            "✅ 到达: %s",
            wp.name.c_str()
        );


        waypoint_arrival_time_ =
            std::chrono::steady_clock::now();

        waypoint_timer_started_ =
            true;


        // -----------------------------------------------------
        // 任务一
        // -----------------------------------------------------

        if (
            current_task_
            ==
            TaskType::TASK1_GUIDE
        )
        {
            ROS_INFO(
                "🧭 任务一：位置到达即可，不调整角度"
            );


            transitionTo(
                State::WAITING_SPEECH
            );


            speakAsync(
                wp.description,
                wp.name
            );

            return;
        }


        // -----------------------------------------------------
        // 任务三
        // -----------------------------------------------------

        if (isTask3())
        {
            task3_target_found_current_ =
                false;

            speech_finished_ =
                false;


            transitionTo(
                State::TASK3_DETECTING
            );


            ROS_INFO(
                "📷 %s 开始任务三识别",
                wp.name.c_str()
            );


            setTask3Detection(true);

            return;
        }


        ROS_ERROR(
            "❌ 未知任务类型"
        );

        transitionTo(
            State::ERROR
        );
    }


    // =========================================================
    // 任务二：到厨房
    // =========================================================

    void handleTask2KitchenArrival()
    {
        ROS_INFO("========================================");
        ROS_INFO(
            "✅ 任务二已到达厨房 (%.3f, %.3f)",
            KITCHEN_X,
            KITCHEN_Y
        );

        ROS_INFO(
            "🧭 任务二不要求最终角度"
        );

        ROS_INFO(
            "📷 人脸识别算法保持暂停；独立食材摄像头保持实时画面，现在开启食材YOLO"
        );

        ROS_INFO(
            "🥬 将持续识别，直到同一帧出现至少3个不同食材类别"
        );

        ROS_INFO("========================================");


        setFaceDetection(false);
        setVoiceListening(false);
        setTask3Detection(false);


        transitionTo(
            State::TASK2_DETECTING
        );


        setTask2Detection(true);
    }


    // =========================================================
    // 任务二 detector 返回的是“最终完整播报文本”
    // =========================================================

    void task2RecipeResultCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        if (
            current_state_
            !=
            State::TASK2_DETECTING
            ||
            current_task_
            !=
            TaskType::TASK2_RECIPE
        )
        {
            ROS_WARN_THROTTLE(
                2.0,
                "⚠️ 当前不在任务二识别阶段，忽略结果"
            );

            return;
        }


        const std::string speech =
            msg->data;


        if (speech.empty())
        {
            return;
        }


        setTask2Detection(false);


        ROS_INFO("========================================");
        ROS_INFO(
            "🥬 任务二识别完成"
        );

        ROS_INFO(
            "🔊 推荐播报: %s",
            speech.c_str()
        );

        ROS_INFO("========================================");


        // 完整播完菜品推荐后才返回
        speakSync(
            speech
        );


        returnToStart();
    }


    // =========================================================
    // 任务三 detector 结果
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
            return;
        }


        setTask3Detection(false);


        const std::string result =
            msg->data;


        if (
            result
            ==
            "error"
        )
        {
            ROS_ERROR(
                "❌ 任务三视觉识别异常"
            );

            transitionTo(
                State::ERROR
            );

            return;
        }


        const bool has_phone =
            (
                result == "phone"
                ||
                result == "both"
            );

        const bool has_backpack =
            (
                result == "backpack"
                ||
                result == "both"
            );


        std::string speech;


        if (
            current_task_
            ==
            TaskType::TASK3_FIND_PHONE
        )
        {
            if (has_phone)
            {
                task3_target_found_current_ =
                    true;

                speech =
                    "找到手机啦，在这里！";
            }
            else if (has_backpack)
            {
                task3_target_found_current_ =
                    false;

                speech =
                    "我看到了书包";
            }
            else
            {
                task3_target_found_current_ =
                    false;

                speech =
                    "这里什么都没有";
            }
        }
        else if (
            current_task_
            ==
            TaskType::TASK3_FIND_BACKPACK
        )
        {
            if (has_backpack)
            {
                task3_target_found_current_ =
                    true;

                speech =
                    "找到书包啦，在这里！";
            }
            else if (has_phone)
            {
                task3_target_found_current_ =
                    false;

                speech =
                    "我看到了手机";
            }
            else
            {
                task3_target_found_current_ =
                    false;

                speech =
                    "这里什么都没有";
            }
        }
        else
        {
            transitionTo(
                State::ERROR
            );

            return;
        }


        if (
            task3_target_found_current_
        )
        {
            ROS_INFO(
                "ℹ️ 本点已找到目标，但仍继续剩余巡检点"
            );
        }


        transitionTo(
            State::WAITING_SPEECH
        );


        speakAsync(
            speech,
            route_[
                current_index_
            ].name
            +
            "巡检结果"
        );
    }


    // =========================================================
    // 任务一 / 三的 5 秒规则
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


        const double elapsed_seconds =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                waypoint_arrival_time_
            ).count();


        if (!speech_finished_.load())
        {
            ROS_INFO_THROTTLE(
                1.0,
                "🔊 等待语音结束..."
            );

            return;
        }


        if (
            elapsed_seconds
            <
            MIN_WAYPOINT_STOP_SECONDS
        )
        {
            ROS_INFO_THROTTLE(
                0.5,
                "⏱️ 还需停留 %.1f 秒",
                MIN_WAYPOINT_STOP_SECONDS
                -
                elapsed_seconds
            );

            return;
        }


        speech_finished_ =
            false;

        waypoint_timer_started_ =
            false;


        // -----------------------------------------------------
        // 任务一
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
        // 任务三
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
                beginCharging();
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


        transitionTo(
            State::ERROR
        );
    }


    // =========================================================
    // 导航失败
    // =========================================================

    void handleNavigationFailure()
    {
        // 任务二只去厨房。
        // 不跳过厨房，直接重试。
        if (
            current_task_
            ==
            TaskType::TASK2_RECIPE
        )
        {
            ROS_ERROR(
                "❌ 任务二前往厨房失败，1秒后重试"
            );

            ros::WallDuration(
                1.0
            ).sleep();


            sendTask2KitchenGoal();

            return;
        }


        const int valid_count =
            currentRouteCount();


        if (
            current_index_ < 0
            ||
            current_index_
            >=
            valid_count
        )
        {
            transitionTo(
                State::ERROR
            );

            return;
        }


        ROS_ERROR(
            "❌ 导航到 %s 失败",
            route_[
                current_index_
            ].name.c_str()
        );


        setTask3Detection(false);


        current_index_++;


        if (
            current_index_
            <
            valid_count
        )
        {
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
            handleTask1Complete();
            return;
        }


        if (isTask3())
        {
            beginCharging();
            return;
        }


        transitionTo(
            State::ERROR
        );
    }


    // =========================================================
    // 导航目标
    // =========================================================

    void sendLobbyGoal()
    {
        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );


        geometry_msgs::PoseStamped goal;

        goal.header.frame_id =
            "map";

        goal.header.stamp =
            ros::Time::now();

        goal.pose.position.x =
            LOBBY_X;

        goal.pose.position.y =
            LOBBY_Y;

        goal.pose.orientation.z =
            0.0;

        goal.pose.orientation.w =
            1.0;


        goal_pub_.publish(
            goal
        );


        ROS_INFO(
            "📤 前往走廊: (%.3f, %.3f)",
            LOBBY_X,
            LOBBY_Y
        );
    }


    // ---------------------------------------------------------
    // 任务二厨房
    // ---------------------------------------------------------

    void sendTask2KitchenGoal()
    {
        setDwaYawGoalTolerance(
            POSITION_ONLY_YAW_GOAL_TOLERANCE
        );


        geometry_msgs::PoseStamped goal;

        goal.header.frame_id =
            "map";

        goal.header.stamp =
            ros::Time::now();

        goal.pose.position.x =
            KITCHEN_X;

        goal.pose.position.y =
            KITCHEN_Y;

        // 给合法四元数即可。
        // yaw_goal_tolerance=pi 保证到点后不为了角度调整。
        goal.pose.orientation.z =
            0.0;

        goal.pose.orientation.w =
            1.0;


        goal_pub_.publish(
            goal
        );


        ROS_INFO(
            "📤 任务二前往厨房: (%.3f, %.3f)，不要求最终角度",
            KITCHEN_X,
            KITCHEN_Y
        );
    }


    // ---------------------------------------------------------
    // 任务一 / 三
    // ---------------------------------------------------------

    void sendNextGoal()
    {
        const int valid_count =
            currentRouteCount();


        if (
            current_index_ < 0
            ||
            current_index_
            >=
            valid_count
            ||
            current_index_
            >=
            static_cast<int>(
                route_.size()
            )
        )
        {
            transitionTo(
                State::ERROR
            );

            return;
        }


        const Waypoint& wp =
            route_[
                current_index_
            ];


        geometry_msgs::PoseStamped goal;

        goal.header.frame_id =
            "map";

        goal.header.stamp =
            ros::Time::now();

        goal.pose.position.x =
            wp.x;

        goal.pose.position.y =
            wp.y;


        if (isTask3())
        {
            const double yaw =
                wp.task3_angle;


            goal.pose.orientation.z =
                std::sin(
                    yaw / 2.0
                );

            goal.pose.orientation.w =
                std::cos(
                    yaw / 2.0
                );


            ROS_INFO(
                "📤 任务三 %d/%d：%s (%.3f, %.3f), yaw=%.1f°",
                current_index_ + 1,
                valid_count,
                wp.name.c_str(),
                wp.x,
                wp.y,
                yaw * 180.0 / M_PI
            );
        }
        else
        {
            goal.pose.orientation.z =
                0.0;

            goal.pose.orientation.w =
                1.0;


            ROS_INFO(
                "📤 任务一 %d/%d：%s (%.3f, %.3f)，不要求最终角度",
                current_index_ + 1,
                valid_count,
                wp.name.c_str(),
                wp.x,
                wp.y
            );
        }


        goal_pub_.publish(
            goal
        );
    }


    void startCurrentTaskNavigation()
    {
        current_index_ = 0;

        task3_target_found_current_ =
            false;

        speech_finished_ =
            false;

        waypoint_timer_started_ =
            false;


        transitionTo(
            State::NAVIGATING
        );


        sendNextGoal();
    }


    void startTask2Navigation()
    {
        current_index_ = 0;

        speech_finished_ =
            false;

        waypoint_timer_started_ =
            false;


        transitionTo(
            State::NAVIGATING
        );


        sendTask2KitchenGoal();
    }


    // =========================================================
    // 任务完成
    // =========================================================

    void handleTask1Complete()
    {
        ROS_INFO(
            "🏁 任务一完成 -> 返回出发点"
        );


        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );


        returnToStart();
    }


    void beginCharging()
    {
        if (!isTask3())
        {
            transitionTo(
                State::ERROR
            );

            return;
        }


        ROS_INFO(
            "🏁 任务三5个地点全部巡检完成 -> 开始充电"
        );


        setFaceDetection(false);
        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);


        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );


        transitionTo(
            State::CHARGING
        );


        std_msgs::Bool msg;
        msg.data = true;

        charge_pub_.publish(
            msg
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
            returnToStart();
        }
        else
        {
            transitionTo(
                State::ERROR
            );
        }
    }


    void returnToStart()
    {
        setFaceDetection(false);
        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);


        setDwaYawGoalTolerance(
            NORMAL_YAW_GOAL_TOLERANCE
        );


        geometry_msgs::PoseStamped home;

        home.header.frame_id =
            "map";

        home.header.stamp =
            ros::Time::now();

        home.pose.position.x =
            START_X;

        home.pose.position.y =
            START_Y;

        home.pose.orientation.z =
            0.0;

        home.pose.orientation.w =
            1.0;


        transitionTo(
            State::RETURNING
        );


        goal_pub_.publish(
            home
        );


        ROS_INFO(
            "🏠 返回出发点: (%.2f, %.2f)",
            START_X,
            START_Y
        );
    }


    // =========================================================
    // 工具
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

        current_index_ =
            0;

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


    void transitionTo(
        State new_state
    )
    {
        std::lock_guard<std::mutex>
            lock(
                state_mutex_
            );


        if (
            current_state_
            ==
            new_state
        )
        {
            return;
        }


        ROS_INFO(
            "🔄 %s -> %s",
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
                return "等待人脸";

            case State::IDLE:
                return "等待语音命令";

            case State::NAVIGATING:
                return "导航中";

            case State::TASK2_DETECTING:
                return "任务二食材识别中";

            case State::TASK3_DETECTING:
                return "任务三识别中";

            case State::WAITING_SPEECH:
                return "地点语音处理中";

            case State::CHARGING:
                return "充电中";

            case State::RETURNING:
                return "返回出发点";

            case State::ERROR:
                return "错误";

            default:
                return "未知";
        }
    }


    // =========================================================
    // 主循环
    // =========================================================

    void start()
    {
        ros::WallDuration(
            1.0
        ).sleep();


        setFaceDetection(false);
        setVoiceListening(false);
        setTask2Detection(false);
        setTask3Detection(false);


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

                last_state =
                    current_state_;
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
                            "👤 等待人脸识别..."
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
                            "💬 可说：参观 / 给我推荐一下今天适合做什么菜 / 找手机 / 找书包"
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
                        timeout_counter = 0;

                        handleNavigationFailure();
                    }

                    break;
                }


                case State::TASK2_DETECTING:
                {
                    // 按要求：
                    // 不设置短时间结束条件；
                    // 直到现实摄像头同一帧检测出 >=3 个不同食材类别。
                    ROS_INFO_THROTTLE(
                        10.0,
                        "🥬 任务二持续识别中，等待至少3个不同食材..."
                    );

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
                        setTask3Detection(
                            false
                        );

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


                    if (
                        timeout_counter
                        >
                        CHARGE_TIMEOUT_SECONDS
                        *
                        LOOP_RATE_HZ
                    )
                    {
                        transitionTo(
                            State::ERROR
                        );
                    }

                    break;
                }


                case State::RETURNING:
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
                        timeout_counter = 0;

                        returnToStart();
                    }

                    break;
                }


                case State::ERROR:
                {
                    ROS_WARN_THROTTLE(
                        5.0,
                        "⛔ ERROR，请检查前面的日志"
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
        "智能服务机器人三任务系统"
    );


    TaskScheduler scheduler;

    scheduler.start();


    return 0;
}
