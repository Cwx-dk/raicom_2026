/**
 * @file task_scheduler.cpp
 * @brief 智能导览机器人 - 语音交互导航系统
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
        CHARGING,
        RETURNING,
        TASK_COMPLETE,
        ERROR
    };


    // =========================================================
    // ROS
    // =========================================================

    ros::NodeHandle nh_;

    ros::Publisher goal_pub_;
    ros::Publisher charge_pub_;

    ros::Publisher face_enable_pub_;
    ros::Publisher voice_enable_pub_;

    ros::Subscriber status_sub_;
    ros::Subscriber voice_sub_;
    ros::Subscriber face_sub_;
    ros::Subscriber charge_complete_sub_;


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
        CHARGE_TIMEOUT_SECONDS = 60;


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


        // 状态控制Topic使用latched publisher

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


        ROS_INFO(
            "🚀 智能导览机器人已启动"
        );

        ROS_INFO(
            "📍 共加载 %zu 个参观目标点",
            route_.size()
        );

        ROS_INFO(
            "📊 初始状态: %s",
            stateToString(
                current_state_
            ).c_str()
        );
    }


    // =========================================================
    // 路线
    // =========================================================

    void initRoute()
    {
        // 定义角度常量（弧度）
        const double ANGLE_0 = 0.0;           // 0度（朝东）
        const double ANGLE_90 = M_PI / 2.0;   // 90度（朝北）
        const double ANGLE_NEG_90 = -M_PI / 2.0;  // -90度（朝南）
        
        route_ = {

            {
                "餐厅",
                2.220,
                1.034,
                ANGLE_0,        // 0度（朝东）
                "这里是餐厅"
            },

            {
                "厨房",
                1.251,
                1.013,
                ANGLE_90,       // 90度（朝北）
                "这里是厨房"
            },

            {
                "客厅",
                1.267,
                -0.028,
                ANGLE_NEG_90,   // -90度（朝南）
                "这里是客厅"
            },

            {
                "卧室",
                2.252,
                -0.056,
                ANGLE_NEG_90,   // -90度（朝南）
                "这里是卧室"
            }
        };
    }


    // =========================================================
    // 人脸启停
    // =========================================================

    void setFaceDetection(
        bool enable
    )
    {
        std_msgs::Bool msg;

        msg.data = enable;

        face_enable_pub_.publish(
            msg
        );


        ROS_INFO(
            "👤 人脸检测控制: %s",
            enable
                ? "开启"
                : "关闭"
        );
    }


    // =========================================================
    // 用户语音监听启停
    // =========================================================

    void setVoiceListening(
        bool enable
    )
    {
        std_msgs::Bool msg;

        msg.data = enable;

        voice_enable_pub_.publish(
            msg
        );


        ROS_INFO(
            "🎤 语音监听控制: %s",
            enable
                ? "开启"
                : "关闭"
        );
    }


    // =========================================================
    // 等待nav_goal_node准备完成
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

        ros::WallRate rate(
            10.0
        );


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
                    "❌ 等待 nav_goal_node 超时！"
                    " %.1f 秒内 /nav_goal 没有订阅者",
                    timeout_seconds
                );

                return false;
            }


            ROS_INFO_THROTTLE(
                2.0,
                "⏳ /nav_goal 暂无订阅者，"
                "继续等待 nav_goal_node..."
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
    // 人脸回调
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
            != State::WAITING_FACE
        )
        {
            return;
        }


        ROS_INFO(
            "========================================"
        );

        ROS_INFO(
            "👤 检测到用户人脸"
        );


        // =====================================================
        // 第一时间退出 WAITING_FACE
        //
        // 即使还有残余face消息，
        // 也不能重复触发欢迎流程。
        // =====================================================

        transitionTo(
            State::IDLE
        );


        face_detected_ =
            true;


        // 人脸阶段结束
        setFaceDetection(
            false
        );


        // =====================================================
        // 欢迎语期间绝对不能听用户语音
        // =====================================================

        setVoiceListening(
            false
        );


        // =====================================================
        // 同步完整播放欢迎语
        // =====================================================

        speakSync(
            "你好，需要帮助吗？"
        );


        ROS_INFO(
            "🔊 欢迎语播放完成"
        );


        // =====================================================
        // 欢迎语完整结束后才开启Pulse
        // =====================================================

        setVoiceListening(
            true
        );


        ROS_INFO(
            "🎧 现在开始等待用户语音..."
        );

        ROS_INFO(
            "========================================"
        );
    }


    // =========================================================
    // 用户语音回调
    // =========================================================

    void voiceCallback(
        const std_msgs::String::ConstPtr& msg
    )
    {
        const std::string recognized_text =
            msg->data;


        ROS_INFO(
            "========================================"
        );

        ROS_INFO(
            "🎤 收到用户语音: [%s]",
            recognized_text.c_str()
        );

        ROS_INFO(
            "📊 当前状态: [%s]",
            stateToString(
                current_state_
            ).c_str()
        );

        ROS_INFO(
            "========================================"
        );


        // 只有欢迎语结束后才接受指令
        if (
            current_state_
            != State::IDLE
        )
        {
            ROS_WARN(
                "⚠️ 当前不在等待用户指令状态，"
                "忽略本次语音"
            );

            return;
        }


        // =====================================================
        // 成功识别成文字，但是不包含“参观”
        //
        // 按你的要求：
        //
        // 关闭语音
        // → 重新做人脸
        // → 人脸成功
        // → 再说欢迎语
        // → 再开语音
        // =====================================================

        if (
            !containsKeyword(
                recognized_text,
                start_keywords_
            )
        )
        {
            ROS_WARN(
                "⚠️ 当前文字不是有效导览命令: [%s]",
                recognized_text.c_str()
            );


            setVoiceListening(
                false
            );


            face_detected_ =
                false;

            voice_command_received_ =
                false;


            transitionTo(
                State::WAITING_FACE
            );


            setFaceDetection(
                true
            );


            ROS_INFO(
                "👤 指令中没有“参观”，"
                "重新进入人脸等待阶段"
            );


            return;
        }


        // =====================================================
        // 包含“参观”
        // =====================================================

        if (
            voice_command_received_
            .exchange(true)
        )
        {
            ROS_WARN(
                "⚠️ 导览任务已经触发，忽略重复命令"
            );

            return;
        }


        ROS_INFO(
            "✅ 检测到任务一导览指令"
        );


        // 用户交互完成
        setVoiceListening(
            false
        );

        setFaceDetection(
            false
        );


        // =====================================================
        // 按你确认的要求：
        //
        // “好的，请跟我来”
        // 必须完整生成并完整播放。
        //
        // 不进行异步起步。
        // =====================================================

        speakSync(
            "好的，请跟我来"
        );


        // 完整播完以后再开始任务一
        startNavigation();
    }


    // =========================================================
    // 关键词判断
    // =========================================================

    bool containsKeyword(
        const std::string& text,
        const std::vector<std::string>& keywords
    )
    {
        for (
            const auto& keyword :
            keywords
        )
        {
            if (
                text.find(keyword)
                != std::string::npos
            )
            {
                ROS_INFO(
                    "🔍 匹配到关键词: [%s]",
                    keyword.c_str()
                );


                return true;
            }
        }


        return false;
    }


    // =========================================================
    // 同步TTS
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
            "--text \"" +
            text +
            "\" "
            "2>/dev/null";


        const int ret =
            system(
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


    // =========================================================
    // 地点介绍异步TTS
    // =========================================================

    void speakAsync(
        const std::string& text,
        const std::string& name
    )
    {
        speech_finished_ =
            false;


        ROS_INFO(
            "🎤 启动语音播报: %s",
            name.c_str()
        );


        std::thread(
            [this, text, name]()
            {
                ROS_INFO(
                    "🔊 开始播报 %s 的介绍",
                    name.c_str()
                );

                ROS_INFO(
                    "📝 内容: %s",
                    text.c_str()
                );


                std::string cmd =
                    "/home/reicom2025/.local/bin/edge-playback "
                    "--voice zh-CN-YunxiNeural "
                    "--text \"" +
                    text +
                    "\" "
                    "2>/dev/null";


                const int ret =
                    system(
                        cmd.c_str()
                    );


                if (ret == 0)
                {
                    ROS_INFO(
                        "✅ 语音播报完整结束: %s",
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


                // =================================================
                // 只有edge-playback完整退出后，
                // 才认为：
                //
                // TTS生成 + 完整语音播放
                //
                // 全部结束。
                // =================================================

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
            != State::NAVIGATING
            &&
            current_state_
            != State::GOING_TO_LOBBY
            &&
            current_state_
            != State::RETURNING
        )
        {
            return;
        }


        ROS_INFO(
            "📩 收到导航反馈: %s",
            msg->data
                ? "✅ 成功"
                : "❌ 失败"
        );


        // =====================================================
        // 导航成功
        // =====================================================

        if (msg->data)
        {
            // -------------------------------------------------
            // 到达走廊
            // -------------------------------------------------

            if (
                current_state_
                == State::GOING_TO_LOBBY
            )
            {
                ROS_INFO(
                    "========================================"
                );

                ROS_INFO(
                    "✅ 已到达走廊位置 (%.3f, %.3f)",
                    LOBBY_X,
                    LOBBY_Y
                );


                face_detected_ =
                    false;

                voice_command_received_ =
                    false;


                // 走廊阶段先关闭语音
                setVoiceListening(
                    false
                );


                // 必须先改变状态
                transitionTo(
                    State::WAITING_FACE
                );


                // 再打开人脸
                setFaceDetection(
                    true
                );


                ROS_INFO(
                    "👤 开始等待用户人脸..."
                );

                ROS_INFO(
                    "========================================"
                );


                return;
            }


            // -------------------------------------------------
            // 到达任务中的参观点
            // -------------------------------------------------

            if (
                current_state_
                == State::NAVIGATING
            )
            {
                handleNavigationSuccess();

                return;
            }


            // -------------------------------------------------
            // 返回出发区完成
            // -------------------------------------------------

            if (
                current_state_
                == State::RETURNING
            )
            {
                ROS_INFO(
                    "========================================"
                );

                ROS_INFO(
                    "✅ 已返回出发区"
                );

                ROS_INFO(
                    "📍 准备再次前往走廊"
                );

                ROS_INFO(
                    "========================================"
                );


                current_index_ =
                    0;

                speech_finished_ =
                    false;

                waypoint_timer_started_ =
                    false;

                face_detected_ =
                    false;

                voice_command_received_ =
                    false;


                // 返回走廊途中都不要感知交互
                setFaceDetection(
                    false
                );

                setVoiceListening(
                    false
                );


                transitionTo(
                    State::GOING_TO_LOBBY
                );


                sendLobbyGoal();


                return;
            }
        }


        // =====================================================
        // 导航失败
        // =====================================================

        else
        {
            if (
                current_state_
                == State::GOING_TO_LOBBY
            )
            {
                ROS_ERROR(
                    "❌ 前往走廊失败，1秒后重新尝试"
                );


                ros::WallDuration(
                    1.0
                ).sleep();


                sendLobbyGoal();


                return;
            }


            if (
                current_state_
                == State::NAVIGATING
            )
            {
                handleNavigationFailure();

                return;
            }


            if (
                current_state_
                == State::RETURNING
            )
            {
                ROS_ERROR(
                    "❌ 返回出发区失败"
                );


                transitionTo(
                    State::ERROR
                );


                return;
            }
        }
    }


    // =========================================================
    // 成功到达某个参观点
    // =========================================================

    void handleNavigationSuccess()
    {
        if (
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


        const std::string description =
            route_[current_index_]
                .description;

        const std::string name =
            route_[current_index_]
                .name;


        ROS_INFO(
            "========================================"
        );

        ROS_INFO(
            "✅ 成功到达目标点: %s",
            name.c_str()
        );


        // =====================================================
        // 关键：导航成功这一刻立即开始计算5秒
        //
        // TTS生成时间也包含在这5秒之中。
        // =====================================================

        waypoint_arrival_time_ =
            std::chrono::steady_clock::now();


        waypoint_timer_started_ =
            true;


        ROS_INFO(
            "⏱️ %s 到达定位点，"
            "开始计算至少5秒停留时间",
            name.c_str()
        );


        transitionTo(
            State::WAITING_SPEECH
        );


        // =====================================================
        // 同时启动：
        //
        // TTS生成 + 语音完整播放
        //
        // 与5秒计时并行。
        // =====================================================

        speakAsync(
            description,
            name
        );
    }


    // =========================================================
    // 导航失败
    // =========================================================

    void handleNavigationFailure()
    {
        if (
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


        ROS_ERROR(
            "❌ 导航到 %s 失败",
            route_[current_index_]
                .name.c_str()
        );


        if (
            current_index_
            <
            static_cast<int>(
                route_.size()
            ) - 1
        )
        {
            ROS_WARN(
                "🔄 跳过失败目标，继续下一目标"
            );


            current_index_++;


            transitionTo(
                State::NAVIGATING
            );


            sendNextGoal();
        }

        else
        {
            ROS_WARN(
                "⚠️ 最后一个目标失败，本轮导览结束"
            );


            handleTaskComplete();
        }
    }


    // =========================================================
    // 发送走廊目标
    // =========================================================

    void sendLobbyGoal()
    {
        ROS_INFO(
            "📍 导航到走廊位置 (%.2f, %.2f)",
            LOBBY_X,
            LOBBY_Y
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

        goal.pose.position.z =
            0.0;


        goal.pose.orientation.x =
            0.0;

        goal.pose.orientation.y =
            0.0;

        goal.pose.orientation.z =
            0.0;

        goal.pose.orientation.w =
            1.0;


        goal_pub_.publish(
            goal
        );


        ROS_INFO(
            "📤 走廊目标已发布到 /nav_goal"
        );
    }


    // =========================================================
    // 发送下一个任务目标
    // =========================================================

    void sendNextGoal()
    {
        if (
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


            transitionTo(
                State::ERROR
            );


            return;
        }


        if (
            current_state_
            != State::NAVIGATING
        )
        {
            transitionTo(
                State::NAVIGATING
            );
        }


        Waypoint& wp =
            route_[current_index_];


        ROS_INFO(
            "🔄 发送第 %d/%zu 个目标: %s (%.2f, %.2f)",
            current_index_ + 1,
            route_.size(),
            wp.name.c_str(),
            wp.x,
            wp.y
        );


        geometry_msgs::PoseStamped goal;


        goal.header.frame_id =
            "map";

        goal.header.stamp =
            ros::Time::now();


        goal.pose.position.x =
            wp.x;

        goal.pose.position.y =
            wp.y;

        goal.pose.position.z =
            0.0;


        goal.pose.orientation.x =
            0.0;

        goal.pose.orientation.y =
            0.0;

        goal.pose.orientation.z =
            std::sin(
                wp.angle / 2.0
            );

        goal.pose.orientation.w =
            std::cos(
                wp.angle / 2.0
            );


        goal_pub_.publish(
            goal
        );


        ROS_INFO(
            "📤 目标已发布到 /nav_goal，等待导航反馈..."
        );
    }


    // =========================================================
    // 开始任务一
    // =========================================================

    void startNavigation()
    {
        ROS_INFO(
            "🚀 开始导览任务"
        );


        current_index_ =
            0;


        transitionTo(
            State::NAVIGATING
        );


        sendNextGoal();
    }


    // =========================================================
    // 充电相关保留
    // =========================================================

    void chargeCompleteCallback(
        const std_msgs::Bool::ConstPtr& msg
    )
    {
        if (
            msg->data
            &&
            current_state_
            == State::CHARGING
        )
        {
            ROS_INFO(
                "🔋 收到充电完成消息"
            );


            charge_completed_ =
                true;
        }
    }


    void sendChargeCommand()
    {
        ROS_INFO(
            "🔋 发布充电指令..."
        );


        std_msgs::Bool msg;

        msg.data =
            true;


        charge_pub_.publish(
            msg
        );
    }


    // =========================================================
    // 任务一完成
    // =========================================================

// =========================================================
// 任务一完成 -> 先去充电
// =========================================================

    void handleTaskComplete()
    {
        ROS_INFO(
            "========================================"
        );

        ROS_INFO(
            "🏁 任务一导览完成"
        );

        ROS_INFO(
            "✅ 餐厅、厨房、客厅、卧室全部参观完成"
        );

        ROS_INFO(
            "🔋 准备前往充电桩充电"
        );

        ROS_INFO(
            "========================================"
        );

        // 语音提示去充电
        speakSync(
            "参观结束，现在去充电"
        );

        // 发送充电指令
        sendChargeCommand();

        // 进入充电状态
        transitionTo(
            State::CHARGING
        );

        charge_completed_ =
            false;

        // 等待充电完成
        waitForChargeComplete();
    }


    // =========================================================
    // 返回出发区
    // =========================================================
// =========================================================
// 等待充电完成
// =========================================================

    void waitForChargeComplete()
    {
        ROS_INFO(
            "⏳ 等待充电完成..."
        );

        // 设置充电超时计时器
        int charge_timeout_counter = 0;
        ros::Rate loop_rate(LOOP_RATE_HZ);

        while (
            ros::ok()
            &&
            !charge_completed_.load()
        )
        {
            ros::spinOnce();

            charge_timeout_counter++;

            if (
                charge_timeout_counter
                >
                CHARGE_TIMEOUT_SECONDS
                *
                LOOP_RATE_HZ
            )
            {
                ROS_ERROR(
                    "⏰ 充电超时！(%d秒)，跳过充电直接返回起点",
                    CHARGE_TIMEOUT_SECONDS
                );

                break;
            }

            // 每10秒打印一次等待信息
            if (
                charge_timeout_counter
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
                    "⏳ 等待充电完成... 已等待 %d 秒",
                    charge_timeout_counter / LOOP_RATE_HZ
                );
            }

            loop_rate.sleep();
        }

        if (
            charge_completed_.load()
        )
        {
            ROS_INFO(
                "✅ 充电完成，开始返回起点"
            );
        }

        // 充电完成后返回起点
        returnToStart();
    }

// =========================================================
// 返回出发区
// =========================================================

    void returnToStart()
    {
        ROS_INFO(
            "========================================"
        );

        ROS_INFO(
            "🏠 返回出发区 (%.2f, %.2f)",
            START_X,
            START_Y
        );

        ROS_INFO(
            "========================================"
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

        home.pose.position.z =
            0.0;

        home.pose.orientation.x =
            0.0;

        home.pose.orientation.y =
            0.0;

        home.pose.orientation.z =
            0.0;

        home.pose.orientation.w =
            1.0;

        // 必须先改变状态
        transitionTo(
            State::RETURNING
        );

        // 再发目标
        goal_pub_.publish(
            home
        );

        ROS_INFO(
            "📤 已发送返回出发区命令"
        );

        ROS_INFO(
            "🔄 正在返回出发区..."
        );

        // 等待导航完成后再结束
        // 或设置超时后自动结束
        int return_timeout = 0;

        while (
            ros::ok()
            &&
            current_state_
            == State::RETURNING
        )
        {
            ros::spinOnce();
            ros::WallDuration(0.1).sleep();

            return_timeout++;

            if (
                return_timeout
                >
                NAV_TIMEOUT_SECONDS
                *
                10
            )
            {
                ROS_ERROR(
                    "⏰ 返回出发区超时，强制结束"
                );

                break;
            }
        }

        // 到达起点后播报结束语
        speakSync(
            "已回到起点，感谢参观"
        );

        transitionTo(
            State::TASK_COMPLETE
        );

        all_done_ =
            true;

        ROS_INFO(
            "✅ 已回到起点，任务全部完成"
        );
    }

    // =========================================================
    // 地点5秒原则
    // =========================================================

    void checkSpeechAndProceed()
    {
        if (
            current_state_
            != State::WAITING_SPEECH
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
                now
                -
                waypoint_arrival_time_
            ).count();


        // TTS生成 + 语音播放是否完整结束
        const bool speech_done =
            speech_finished_.load();


        // 从导航成功到达开始是否已经5秒
        const bool minimum_stop_done =
            elapsed_seconds
            >=
            MIN_WAYPOINT_STOP_SECONDS;


        // =====================================================
        // 情况1：
        // 已经5秒甚至更久，但语音仍未完整结束
        //
        // 必须继续等。
        // =====================================================

        if (!speech_done)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "🔊 已停留 %.1f 秒，"
                "等待地点介绍完整播放结束...",
                elapsed_seconds
            );


            return;
        }


        // =====================================================
        // 情况2：
        // 语音完整结束，但是还不到5秒
        //
        // 必须补足5秒。
        // =====================================================

        if (!minimum_stop_done)
        {
            const double remaining =
                MIN_WAYPOINT_STOP_SECONDS
                -
                elapsed_seconds;


            ROS_INFO_THROTTLE(
                0.5,
                "⏱️ 语音已完整播放，"
                "继续停留 %.1f 秒以满足5秒要求",
                remaining
            );


            return;
        }


        // =====================================================
        // 情况3：
        //
        // 语音已经完整结束
        // AND
        // 从到达该点开始已经 >= 5秒
        //
        // 才允许下一步。
        //
        // 实际停留时间：
        //
        // max(
        //     5秒,
        //     TTS生成时间 + 完整播放时间
        // )
        // =====================================================

        ROS_INFO(
            "✅ %s 停留完成：累计 %.2f 秒，"
            "且介绍已完整播放",
            route_[current_index_]
                .name.c_str(),
            elapsed_seconds
        );


        speech_finished_ =
            false;

        waypoint_timer_started_ =
            false;


        current_index_++;


        if (
            current_index_
            >=
            static_cast<int>(
                route_.size()
            )
        )
        {
            handleTaskComplete();
        }

        else
        {
            transitionTo(
                State::NAVIGATING
            );


            sendNextGoal();
        }
    }


    // =========================================================
    // 错误处理
    // =========================================================

    void handleError()
    {
        ROS_ERROR(
            "🚨 系统进入错误状态"
        );
    }


    // =========================================================
    // 状态转换
    // =========================================================

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
            == new_state
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


    // =========================================================
    // 状态名称
    // =========================================================

    std::string stateToString(
        State state
    )
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
                return "空闲(等待指令)";

            case State::NAVIGATING:
                return "导航中";

            case State::WAITING_SPEECH:
                return "语音播报中";

            case State::CHARGING:
                return "充电中";

            case State::RETURNING:
                return "返回起点中";

            case State::TASK_COMPLETE:
                return "任务完成";

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


        // 初始状态：
        // face OFF
        // voice OFF

        setFaceDetection(
            false
        );

        setVoiceListening(
            false
        );


        // =====================================================
        // 开始去走廊
        // =====================================================

        ROS_INFO(
            "📍 开始导航到走廊位置"
        );


        transitionTo(
            State::GOING_TO_LOBBY
        );


        // =====================================================
        // 防止第一条/nav_goal发布过早
        // =====================================================

        if (
            !waitForNavGoalReady(
                20.0
            )
        )
        {
            ROS_ERROR(
                "❌ nav_goal_node 未就绪，无法开始任务"
            );


            transitionTo(
                State::ERROR
            );


            return;
        }


        // 只发送一次
        sendLobbyGoal();


        // =====================================================
        // 主循环
        // =====================================================

        ros::Rate loop_rate(
            LOOP_RATE_HZ
        );


        int timeout_counter =
            0;

        int waiting_face_counter =
            0;


        State last_state =
            current_state_;


        while (
            ros::ok()
            &&
            !all_done_
        )
        {
            ros::spinOnce();


            if (
                current_state_
                != last_state
            )
            {
                timeout_counter =
                    0;

                waiting_face_counter =
                    0;

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
                        ROS_ERROR(
                            "⏰ 前往走廊超时"
                        );


                        timeout_counter =
                            0;


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
                            "💬 请说带有“参观”的指令"
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


                        timeout_counter =
                            0;


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
                    ROS_INFO_THROTTLE(
                        5.0,
                        "🔋 正在充电中..."
                    );
                    
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
                            "⏰ 返回出发区超时"
                        );


                        timeout_counter =
                            0;


                        returnToStart();
                    }


                    break;
                }


                case State::TASK_COMPLETE:
                {
                    all_done_ =
                        true;

                    break;
                }


                case State::ERROR:
                {
                    ROS_WARN_THROTTLE(
                        5.0,
                        "⛔ 系统处于错误状态"
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


        ROS_INFO(
            "🛑 调度器结束运行"
        );
    }
};


// =============================================================
// main
// =============================================================

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
        "智能导览机器人系统"
    );


    TaskScheduler scheduler;


    scheduler.start();


    return 0;
}