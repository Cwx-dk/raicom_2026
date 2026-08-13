/**
 * @file task_scheduler.cpp
 * @brief 服务机器人任务调度：任务一导览 + 任务二推荐菜 + 任务三目标寻找
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
#include <fstream>
#include <sstream>
#include <jsoncpp/json/json.h>

#include <set>        // 用于 matchRecipe 中的 std::set
#include <algorithm>  // 用于 std::find


// =========================================================
// Recipe 结构体（菜谱）
// =========================================================

struct Recipe
{
    std::string name;
    std::vector<std::string> ingredients;
    std::string steps;
};


// =========================================================
// Waypoint 结构体（目标点）
// =========================================================

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


// =========================================================
// TaskScheduler 类
// =========================================================

class TaskScheduler
{
private:

    // =========================================================
    // 状态枚举
    // =========================================================

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
        TASK2_RECOMMEND_FOOD,
        TASK3_FIND_PHONE,
        TASK3_FIND_BACKPACK
    };


    // =========================================================
    // ROS 通信
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

    // 任务三：当前地点是否找到目标
    bool task3_target_found_current_ = false;

    // =========================================================
    // 任务二专用变量
    // =========================================================

    // 菜谱数据库
    std::vector<Recipe> recipes_;

    // 识别的食材列表
    std::vector<std::string> detected_ingredients_;

    // 选中的3种食材
    std::vector<std::string> selected_ingredients_;

    // 匹配到的菜名
    std::string matched_dish_name_;

    // 匹配到的做法
    std::string matched_recipe_;

    // 识别重试计数
    int detection_retry_count_ = 0;

    // 最大重试次数
    const int MAX_DETECTION_RETRIES = 3;


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

    // 任务三检测超时（10秒作为视觉链路故障保护）
    const int TASK3_DETECTION_TIMEOUT_SECONDS = 10;

    // 充电超时
    const int CHARGE_TIMEOUT_SECONDS = 180;


    // =========================================================
    // 坐标
    // =========================================================

    const double LOBBY_X = 1.241;
    const double LOBBY_Y = 2.137;

    const double START_X = 0.0;
    const double START_Y = 0.0;


public:

    // =========================================================
    // 构造函数
    // =========================================================

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

        // 这些控制 Topic 使用 latched publisher
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

        // 加载菜谱
        std::string recipe_path;
        nh_.param<std::string>("recipe_file", recipe_path, "/home/reicom2025/recipes.json");

        if (!loadRecipes(recipe_path))
        {
            ROS_WARN("⚠️ 菜谱文件加载失败，使用默认硬编码菜谱");
            loadDefaultRecipes();
        }

        ROS_INFO("========================================");
        ROS_INFO("🚀 任务调度器已启动");
        ROS_INFO("✅ 任务一：导览，不充电");
        ROS_INFO("✅ 任务二：推荐菜（加载 %zu 道菜谱）", recipes_.size());
        ROS_INFO("✅ 任务三：找手机/书包，完成后充电");
        ROS_INFO("📍 共加载 %zu 个目标点", route_.size());
        ROS_INFO("========================================");
    }


    // =========================================================
    // 路线初始化
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
    // 菜谱加载
    // =========================================================

    bool loadRecipes(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            ROS_ERROR("❌ 无法打开菜谱文件: %s", filepath.c_str());
            return false;
        }

        Json::Value root;
        Json::Reader reader;

        if (!reader.parse(file, root))
        {
            ROS_ERROR("❌ JSON 解析失败: %s", reader.getFormattedErrorMessages().c_str());
            return false;
        }

        const Json::Value& recipe_array = root["recipes"];
        for (int i = 0; i < (int)recipe_array.size(); ++i)
        {
            Recipe recipe;
            recipe.name = recipe_array[i]["name"].asString();
            recipe.steps = recipe_array[i]["steps"].asString();

            const Json::Value& ingredients = recipe_array[i]["ingredients"];
            for (int j = 0; j < (int)ingredients.size(); ++j)
            {
                recipe.ingredients.push_back(ingredients[j].asString());
            }

            recipes_.push_back(recipe);
        }

        ROS_INFO("✅ 成功加载 %zu 道菜谱", recipes_.size());
        return true;
    }


    void loadDefaultRecipes()
    {
        Recipe r;

        // 默认硬编码菜谱（保证至少有菜可用）
        r.name = "西红柿炒鸡蛋";
        r.ingredients = {"西红柿", "鸡蛋", "青椒"};
        r.steps = "1. 西红柿切块，鸡蛋打散加盐。\n2. 热油炒鸡蛋至凝固盛出。\n3. 炒西红柿至软烂，加入鸡蛋和青椒翻炒。\n4. 加盐调味，出锅。";
        recipes_.push_back(r);

        r.name = "青椒土豆丝";
        r.ingredients = {"青椒", "土豆", "鸡蛋"};
        r.steps = "1. 土豆切丝泡水去淀粉，青椒切丝。\n2. 热油爆香，放入土豆丝翻炒至半透明。\n3. 加入青椒丝翻炒，加盐调味出锅。";
        recipes_.push_back(r);

        r.name = "黄瓜炒鸡蛋";
        r.ingredients = {"黄瓜", "鸡蛋", "虾仁"};
        r.steps = "1. 黄瓜切片，鸡蛋打散，虾仁去虾线。\n2. 热油炒虾仁至变色盛出。\n3. 炒鸡蛋至凝固，加入黄瓜和虾仁翻炒。\n4. 加盐调味，出锅。";
        recipes_.push_back(r);

        r.name = "白菜炖豆腐";
        r.ingredients = {"白菜", "豆腐", "猪肉"};
        r.steps = "1. 白菜切块，豆腐切块，猪肉切片。\n2. 热油煎豆腐至金黄盛出。\n3. 炒猪肉至变色，放入白菜翻炒，加水炖煮。\n4. 放入豆腐炖5分钟，加盐调味出锅。";
        recipes_.push_back(r);

        ROS_INFO("✅ 加载 %zu 道默认硬编码菜谱", recipes_.size());
    }


    // =========================================================
    // 菜谱匹配算法
    // =========================================================

    bool matchRecipe(
        const std::vector<std::string>& detected,
        std::vector<std::string>& selected,
        std::string& dish_name,
        std::string& recipe_steps
    )
    {
        if (detected.empty() || recipes_.empty())
        {
            return false;
        }

        // 将识别的食材转为集合，方便查找
        std::set<std::string> detected_set(detected.begin(), detected.end());

        int best_match_count = 0;
        int best_recipe_index = -1;

        // 遍历所有菜谱，计算匹配度
        for (int i = 0; i < (int)recipes_.size(); ++i)
        {
            const Recipe& recipe = recipes_[i];
            int match_count = 0;

            for (const std::string& ingredient : recipe.ingredients)
            {
                if (detected_set.count(ingredient) > 0)
                {
                    match_count++;
                }
            }

            // 检查是否所有食材都匹配上了
            if (match_count == (int)recipe.ingredients.size())
            {
                // 完全匹配，直接选择
                selected = recipe.ingredients;
                dish_name = recipe.name;
                recipe_steps = recipe.steps;
                return true;
            }

            // 记录最高匹配度
            if (match_count > best_match_count)
            {
                best_match_count = match_count;
                best_recipe_index = i;
            }
        }

        // 没有完全匹配，选择匹配度最高的（至少匹配2种食材）
        if (best_recipe_index >= 0 && best_match_count >= 2)
        {
            const Recipe& best = recipes_[best_recipe_index];

            // 选中的食材：从检测到的中取菜谱需要的食材
            for (const std::string& ingredient : best.ingredients)
            {
                if (detected_set.count(ingredient) > 0)
                {
                    selected.push_back(ingredient);
                }
            }

            // 如果选中的少于3种，补一些检测到的其他食材
            for (const std::string& ingredient : detected)
            {
                if (std::find(selected.begin(), selected.end(), ingredient) == selected.end())
                {
                    selected.push_back(ingredient);
                    if ((int)selected.size() >= 3) break;
                }
            }

            dish_name = best.name;
            recipe_steps = best.steps;
            return true;
        }

        return false;
    }


    // =========================================================
    // 工具函数
    // =========================================================

    std::vector<std::string> parseDetectedIngredients(const std::string& result)
    {
        std::vector<std::string> ingredients;
        std::stringstream ss(result);
        std::string item;

        while (std::getline(ss, item, ','))
        {
            // 去除首尾空格
            item.erase(0, item.find_first_not_of(" \t"));
            item.erase(item.find_last_not_of(" \t") + 1);

            if (!item.empty())
            {
                ingredients.push_back(item);
            }
        }

        return ingredients;
    }


    std::string joinStrings(const std::vector<std::string>& vec, const std::string& delimiter)
    {
        if (vec.empty()) return "";

        std::string result = vec[0];
        for (int i = 1; i < (int)vec.size(); ++i)
        {
            result += delimiter + vec[i];
        }
        return result;
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
            "📷 任务三/任务二 检测控制: %s",
            enable ? "开启" : "关闭"
        );
    }


    bool waitForNavGoalReady(double timeout_seconds = 20.0)
    {
        ROS_INFO("⏳ 等待 nav_goal_node 准备完成...");

        ros::WallTime start_time = ros::WallTime::now();
        ros::WallRate rate(10.0);

        while (ros::ok() && goal_pub_.getNumSubscribers() == 0)
        {
            const double elapsed = (ros::WallTime::now() - start_time).toSec();

            if (elapsed >= timeout_seconds)
            {
                ROS_ERROR(
                    "❌ 等待 nav_goal_node 超时，%.1f 秒内 /nav_goal 没有订阅者",
                    timeout_seconds
                );
                return false;
            }

            ROS_INFO_THROTTLE(2.0, "⏳ /nav_goal 暂无订阅者，继续等待...");
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
    // 人脸回调
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

        speakSync("你好，需要帮助吗？");

        setVoiceListening(true);

        ROS_INFO("🎧 现在开始等待用户语音...");
        ROS_INFO("========================================");
    }


    // =========================================================
    // 语音回调
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
        const bool has_recommend = containsText(text, "推荐菜");
        const bool has_phone = containsText(text, "手机");
        const bool has_backpack = containsText(text, "书包");

        // =========================================================
        // 任务二：推荐菜
        // =========================================================
        if (has_recommend)
        {
            if (voice_command_received_.exchange(true))
            {
                ROS_WARN("⚠️ 当前任务已经触发，忽略重复命令");
                return;
            }

            current_task_ = TaskType::TASK2_RECOMMEND_FOOD;

            setVoiceListening(false);
            setFaceDetection(false);
            setTask3Detection(false);

            ROS_INFO("✅ 触发任务二：推荐菜");

            speakSync("好的，带您去厨房看看有什么食材");

            startCurrentTaskNavigation();
            return;
        }

        // =========================================================
        // 任务三：手机/书包（优先匹配）
        // =========================================================
        if (has_phone && has_backpack)
        {
            setVoiceListening(false);
            speakSync("请告诉我需要找手机还是书包");
            setVoiceListening(true);
            return;
        }

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
                speakSync("好的，请跟我来");
                ROS_INFO("✅ 触发任务三：寻找手机");
            }
            else
            {
                current_task_ = TaskType::TASK3_FIND_BACKPACK;
                speakSync("好的，请跟我来");
                ROS_INFO("✅ 触发任务三：寻找书包");
            }

            startCurrentTaskNavigation();
            return;
        }

        // =========================================================
        // 任务一：导览
        // =========================================================
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

            speakSync("好的，请跟我来");

            startCurrentTaskNavigation();
            return;
        }

        // =========================================================
        // 无效指令：重新走人脸唤醒流程
        // =========================================================
        ROS_WARN("⚠️ 当前文字不是有效任务命令: [%s]", text.c_str());

        setVoiceListening(false);
        setTask3Detection(false);

        face_detected_ = false;
        voice_command_received_ = false;
        current_task_ = TaskType::NONE;

        transitionTo(State::WAITING_FACE);
        setFaceDetection(true);
    }


    bool containsText(const std::string& text, const std::string& keyword)
    {
        return text.find(keyword) != std::string::npos;
    }


    // =========================================================
    // TTS 语音播报
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


    void speakAsync(const std::string& text, const std::string& name)
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
    // 导航状态回调
    // =========================================================

    void statusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (current_state_ != State::NAVIGATING
            && current_state_ != State::GOING_TO_LOBBY
            && current_state_ != State::RETURNING)
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
    // 到达目标点处理
    // =========================================================

    void handleWaypointArrival()
    {
        if (current_index_ < 0 || current_index_ >= static_cast<int>(route_.size()))
        {
            ROS_ERROR("❌ 目标点索引越界");
            transitionTo(State::ERROR);
            return;
        }

        const Waypoint& wp = route_[current_index_];

        ROS_INFO("========================================");
        ROS_INFO("✅ 成功到达目标点: %s", wp.name.c_str());

        waypoint_arrival_time_ = std::chrono::steady_clock::now();
        waypoint_timer_started_ = true;

        ROS_INFO("⏱️ %s：从现在开始计算至少5秒停留时间", wp.name.c_str());

        // =========================================================
        // 任务一：导览播报
        // =========================================================
        if (current_task_ == TaskType::TASK1_GUIDE)
        {
            transitionTo(State::WAITING_SPEECH);
            speakAsync(wp.description, wp.name);
            return;
        }

        // =========================================================
        // 任务二：推荐菜（到达厨房后开始识别）
        // =========================================================
        if (current_task_ == TaskType::TASK2_RECOMMEND_FOOD)
        {
            // 重置识别状态
            detected_ingredients_.clear();
            selected_ingredients_.clear();
            matched_dish_name_.clear();
            matched_recipe_.clear();
            detection_retry_count_ = 0;

            transitionTo(State::TASK2_DETECTING);

            ROS_INFO("🍅 机器人已到达厨房，开始识别食材");
            setTask3Detection(true);
            return;
        }

        // =========================================================
        // 任务三：找手机/书包
        // =========================================================
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
    // 视觉检测结果回调（任务二 + 任务三共用）
    // =========================================================

    void task3DetectionResultCallback(const std_msgs::String::ConstPtr& msg)
    {
        // 任务二：推荐菜
        if (current_state_ == State::TASK2_DETECTING)
        {
            handleTask2DetectionResult(msg);
            return;
        }

        // 任务三：找手机/书包
        if (current_state_ == State::TASK3_DETECTING)
        {
            handleTask3DetectionResult(msg);
            return;
        }

        ROS_WARN_THROTTLE(
            2.0,
            "⚠️ 收到视觉结果但当前不在检测状态: %s",
            msg->data.c_str()
        );
    }


    // =========================================================
    // 任务二：食材识别结果处理
    // =========================================================

    void handleTask2DetectionResult(const std_msgs::String::ConstPtr& msg)
    {
        setTask3Detection(false);

        const std::string result = msg->data;

        ROS_INFO("========================================");
        ROS_INFO("🍅 任务二食材识别结果: [%s]", result.c_str());
        ROS_INFO("========================================");

        // result 格式: "apple,banana,tomato" 或 "none" 或 "error"
        if (result == "error")
        {
            ROS_ERROR("❌ 食材识别失败");
            handleTask2RetryOrFail();
            return;
        }

        if (result == "none" || result.empty())
        {
            ROS_WARN("⚠️ 未识别到任何食材");
            handleTask2RetryOrFail();
            return;
        }

        // 解析识别的食材列表
        std::vector<std::string> detected = parseDetectedIngredients(result);

        if (detected.empty())
        {
            ROS_WARN("⚠️ 解析食材列表为空");
            handleTask2RetryOrFail();
            return;
        }

        ROS_INFO("📋 识别到的食材: %s", joinStrings(detected, ", ").c_str());

        // 尝试匹配菜谱
        if (matchRecipe(detected, selected_ingredients_, matched_dish_name_, matched_recipe_))
        {
            ROS_INFO("🍽️ 匹配到菜谱: %s", matched_dish_name_.c_str());
            ROS_INFO("📝 使用食材: %s", joinStrings(selected_ingredients_, ", ").c_str());

            // 构造播报文本
            std::string speech = "我推荐 ";
            speech += matched_dish_name_;
            speech += "，需要 ";
            speech += joinStrings(selected_ingredients_, "、");
            speech += "。做法是：";
            speech += matched_recipe_;

            transitionTo(State::WAITING_SPEECH);
            speakAsync(speech, "推荐菜");
            return;
        }

        // 没有匹配到合适的菜谱
        ROS_WARN("⚠️ 无法根据识别到的食材匹配菜谱");
        handleTask2RetryOrFail();
    }


    void handleTask2RetryOrFail()
    {
        detection_retry_count_++;

        if (detection_retry_count_ < MAX_DETECTION_RETRIES)
        {
            ROS_INFO("🔄 第 %d 次重试识别食材...", detection_retry_count_);
            ros::Duration(1.0).sleep();

            transitionTo(State::TASK2_DETECTING);
            setTask3Detection(true);
        }
        else
        {
            ROS_ERROR("❌ 食材识别失败 %d 次，放弃任务二", MAX_DETECTION_RETRIES);

            speakSync("抱歉，没有找到合适的食材，无法推荐菜品");

            // 返回出发区
            returnToStart();
        }
    }


    // =========================================================
    // 任务三：手机/书包识别结果处理
    // =========================================================

    void handleTask3DetectionResult(const std_msgs::String::ConstPtr& msg)
    {
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

        const bool has_phone = (result == "phone" || result == "both");
        const bool has_backpack = (result == "backpack" || result == "both");

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

        ROS_INFO("🎯 当前地点是否找到用户目标: %s", task3_target_found_current_ ? "是" : "否");
        ROS_INFO("🔊 本地点将播报: %s", speech.c_str());
        ROS_INFO("========================================");

        transitionTo(State::WAITING_SPEECH);
        speakAsync(speech, route_[current_index_].name + "巡检结果");
    }


    // =========================================================
    // 5秒规则：任务一、二、三共用
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
        const double elapsed_seconds = std::chrono::duration<double>(now - waypoint_arrival_time_).count();

        const bool speech_done = speech_finished_.load();
        const bool minimum_stop_done = elapsed_seconds >= MIN_WAYPOINT_STOP_SECONDS;

        if (!speech_done)
        {
            ROS_INFO_THROTTLE(1.0, "🔊 已停留 %.1f 秒，等待本地点任务和语音完整结束...", elapsed_seconds);
            return;
        }

        if (!minimum_stop_done)
        {
            ROS_INFO_THROTTLE(0.5, "⏱️ 本地点任务已完成，还需停留 %.1f 秒满足5秒规则", MIN_WAYPOINT_STOP_SECONDS - elapsed_seconds);
            return;
        }

        ROS_INFO("✅ %s 本地点处理完成：累计 %.2f 秒，满足5秒规则", route_[current_index_].name.c_str(), elapsed_seconds);

        speech_finished_ = false;
        waypoint_timer_started_ = false;

        // =========================================================
        // 任务一：导览
        // =========================================================
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

        // =========================================================
        // 任务二：推荐菜（播报完成后直接返回出发区）
        // =========================================================
        if (current_task_ == TaskType::TASK2_RECOMMEND_FOOD)
        {
            ROS_INFO("🍽️ 任务二推荐菜完成，返回出发区");
            returnToStart();
            return;
        }

        // =========================================================
        // 任务三：找手机/书包
        // =========================================================
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
    // 导航失败处理
    // =========================================================

    void handleNavigationFailure()
    {
        if (current_index_ < 0 || current_index_ >= static_cast<int>(route_.size()))
        {
            transitionTo(State::ERROR);
            return;
        }

        ROS_ERROR("❌ 导航到 %s 失败", route_[current_index_].name.c_str());

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

        if (current_task_ == TaskType::TASK2_RECOMMEND_FOOD)
        {
            ROS_WARN("⚠️ 任务二导航失败，直接返回出发区");
            returnToStart();
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
        if (current_index_ < 0 || current_index_ >= static_cast<int>(route_.size()))
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

        // 任务一/任务二使用默认0°朝向，任务三使用柜子朝向
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
        ROS_INFO("📤 第 %d/%zu 个目标: %s (%.3f, %.3f)", current_index_ + 1, route_.size(), wp.name.c_str(), wp.x, wp.y);

        if (isTask3())
        {
            ROS_INFO("🧭 任务三柜子朝向: %.1f°", goal_angle * 180.0 / M_PI);
        }
        else
        {
            ROS_INFO("🧭 任务一/任务二使用默认0°导航朝向");
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
        return (current_task_ == TaskType::TASK3_FIND_PHONE
                || current_task_ == TaskType::TASK3_FIND_BACKPACK);
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

        ROS_INFO("🔄 状态转换: %s -> %s", stateToString(current_state_).c_str(), stateToString(new_state).c_str());

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
            case State::TASK2_DETECTING:
                return "任务二识别中";
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

                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ)
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
                        ROS_INFO("💬 可说：参观 / 推荐菜 / 帮我找一下手机 / 帮我找一下书包");
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

                case State::TASK2_DETECTING:
                {
                    timeout_counter++;

                    if (timeout_counter > TASK3_DETECTION_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 任务二视觉识别超时，没有收到 /task3_detection_result");
                        setTask3Detection(false);
                        handleTask2RetryOrFail();
                    }
                    break;
                }

                case State::TASK3_DETECTING:
                {
                    timeout_counter++;

                    if (timeout_counter > TASK3_DETECTION_TIMEOUT_SECONDS * LOOP_RATE_HZ)
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

                    ROS_INFO_THROTTLE(10.0, "🔋 等待自动充电节点完成...");

                    if (timeout_counter > CHARGE_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 自动充电超过 %d 秒，进入ERROR状态", CHARGE_TIMEOUT_SECONDS);
                        transitionTo(State::ERROR);
                    }
                    break;
                }

                case State::RETURNING:
                {
                    timeout_counter++;

                    ROS_INFO_THROTTLE(5.0, "🔄 机器人正在返回出发区...");

                    if (timeout_counter > NAV_TIMEOUT_SECONDS * LOOP_RATE_HZ)
                    {
                        ROS_ERROR("⏰ 返回出发区超时，重新发送返回目标");
                        timeout_counter = 0;
                        returnToStart();
                    }
                    break;
                }

                case State::ERROR:
                {
                    ROS_WARN_THROTTLE(5.0, "⛔ 系统处于错误状态，请查看前面的ERROR日志");
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


// =========================================================
// main
// =========================================================

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "task_scheduler");

    ROS_INFO("智能服务机器人任务系统");

    TaskScheduler scheduler;
    scheduler.start();

    return 0;
}