#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
任务二：第二台现实摄像头实时预览 + 14类食材识别 + 菜品推荐

重要行为：
1. 节点一启动，就立刻创建 "Task2 Food Detection" 窗口。
2. 第二台现实摄像头从 /dev/video4、/dev/video5 中自动选择可正常读彩色帧的节点。
3. 摄像头一旦正常打开，在节点整个生命周期内保持打开。
4. 未执行任务二时：
      摄像头画面持续显示，YOLO OFF。
5. 收到 /task2_detection_enable=true：
      开启 best2.pt 识别。
6. 跨帧累计不同食材类别；同类别只保留本轮最高置信度。
7. 累计 >= 3 个不同类别后，取置信度最高的三个不同类别。
8. 匹配 recipes.json 并发布 /task2_recipe_result。
9. 完成后只关闭 YOLO 检测状态，摄像头画面继续显示。
10. 只有节点真正 shutdown 时才 release 食材摄像头。

为了保证“启动时立即看到摄像头画面”：
- YOLO 模型放到后台线程加载；
- 主线程优先打开窗口并持续显示摄像头；
- 模型尚未加载好时也不会阻塞摄像头预览。
"""

import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
import torch

from std_msgs.msg import Bool, String
from ultralytics import YOLO


class Task2FoodDetector:

    WINDOW_NAME = "Task2 Food Detection"

    def __init__(self):

        rospy.init_node(
            "task2_food_detector",
            anonymous=False
        )

        # =====================================================
        # 参数
        # =====================================================

        self.model_path = rospy.get_param(
            "~model_path",
            "/home/reicom2025/bobac3_ws/src/model/best2.pt"
        )

        self.recipes_path = rospy.get_param(
            "~recipes_path",
            "/home/reicom2025/bobac3_ws/src/task_scheduler/src/recipes.json"
        )

        # 第二个物理现实摄像头。
        # 完全不碰人脸摄像头的 /dev/video0 ~ /dev/video3。
        self.camera_devices = rospy.get_param(
            "~camera_devices",
            ["/dev/video4", "/dev/video5"]
        )

        self.conf = float(
            rospy.get_param(
                "~conf",
                0.25
            )
        )

        self.imgsz = int(
            rospy.get_param(
                "~imgsz",
                640
            )
        )

        self.show_window = bool(
            rospy.get_param(
                "~show_window",
                True
            )
        )

        self.camera_width = int(
            rospy.get_param(
                "~camera_width",
                640
            )
        )

        self.camera_height = int(
            rospy.get_param(
                "~camera_height",
                480
            )
        )

        self.camera_fps = int(
            rospy.get_param(
                "~camera_fps",
                30
            )
        )

        # =====================================================
        # 14类映射
        # 必须与 best2.pt 的训练顺序一致
        # =====================================================

        self.class_names_cn = {
            0: "上海青",
            1: "玉米",
            2: "虾仁",
            3: "黄瓜",
            4: "猪肉",
            5: "茄子",
            6: "土豆",
            7: "鸡蛋",
            8: "西红柿",
            9: "三文鱼",
            10: "青椒",
            11: "豆腐",
            12: "香蕉",
            13: "苹果",
        }

        # =====================================================
        # 任务状态
        # =====================================================

        self.active = False
        self.result_sent = False

        # 本轮每个类别见过的最高置信度结果
        self.session_best_by_class = {}

        # =====================================================
        # 菜谱
        # =====================================================

        self.recipe_map = {}
        self.fallback_rules = {}

        self.load_recipes()

        # =====================================================
        # 摄像头状态
        # =====================================================

        self.cap = None
        self.camera_device_in_use = None

        self.last_camera_retry_time = 0.0
        self.camera_retry_interval = 1.0

        # =====================================================
        # 模型状态
        #
        # 模型异步加载，避免阻塞启动画面。
        # =====================================================

        self.model = None
        self.model_ready = False
        self.model_loading = True
        self.model_error = ""

        if torch.cuda.is_available():

            self.device = 0

        else:

            self.device = "cpu"

        # =====================================================
        # ROS
        # =====================================================

        self.result_pub = rospy.Publisher(
            "/task2_recipe_result",
            String,
            queue_size=10
        )

        self.control_sub = rospy.Subscriber(
            "/task2_detection_enable",
            Bool,
            self.control_callback,
            queue_size=1
        )

        rospy.on_shutdown(
            self.shutdown
        )

        # =====================================================
        # 关键：
        # 先创建窗口，再后台加载YOLO。
        # =====================================================

        if self.show_window:

            cv2.namedWindow(
                self.WINDOW_NAME,
                cv2.WINDOW_NORMAL
            )

            cv2.resizeWindow(
                self.WINDOW_NAME,
                self.camera_width,
                self.camera_height
            )

            # 先显示一个启动画面，保证 launch 一启动窗口就出现
            boot = np.zeros(
                (
                    self.camera_height,
                    self.camera_width,
                    3
                ),
                dtype=np.uint8
            )

            cv2.putText(
                boot,
                "TASK2 CAMERA STARTING...",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2
            )

            cv2.putText(
                boot,
                "Food camera: /dev/video4 or /dev/video5",
                (30, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1
            )

            cv2.imshow(
                self.WINDOW_NAME,
                boot
            )

            # 让 Linux GUI 真正创建窗口
            cv2.waitKey(
                1
            )

        # 优先打开食材摄像头
        self.try_open_food_camera()

        # YOLO 后台加载
        self.model_thread = threading.Thread(
            target=self.load_model_background,
            daemon=True
        )

        self.model_thread.start()

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "✅ task2_food_detector 已启动"
        )

        rospy.loginfo(
            "📷 第二台现实摄像头候选: %s",
            str(
                self.camera_devices
            )
        )

        rospy.loginfo(
            "📷 当前食材摄像头: %s",
            str(
                self.camera_device_in_use
            )
        )

        rospy.loginfo(
            "📺 窗口启动即显示；YOLO未开启时也持续显示实时画面"
        )

        rospy.loginfo(
            "🤖 best2.pt 正在后台加载，不阻塞摄像头预览"
        )

        rospy.loginfo(
            "========================================"
        )


    # =========================================================
    # YOLO后台加载
    # =========================================================

    def load_model_background(
        self
    ):

        try:

            rospy.loginfo(
                "🥬 后台加载任务二模型: %s",
                self.model_path
            )

            model = YOLO(
                self.model_path
            )

            rospy.loginfo(
                "🏷️ best2.pt model.names = %s",
                str(
                    model.names
                )
            )

            # 预热一次，避免真正任务开始时第一帧特别慢
            dummy = np.zeros(
                (
                    480,
                    640,
                    3
                ),
                dtype=np.uint8
            )

            model.predict(
                source=dummy,
                conf=self.conf,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False
            )

            # 所有加载/预热都成功后再赋给主线程
            self.model = model

            self.model_ready = True
            self.model_loading = False

            if self.device == 0:

                rospy.loginfo(
                    "✅ Task2 YOLO模型加载完成，GPU: %s",
                    torch.cuda.get_device_name(0)
                )

            else:

                rospy.loginfo(
                    "✅ Task2 YOLO模型加载完成，使用CPU"
                )

        except Exception as e:

            self.model_error = repr(
                e
            )

            self.model_ready = False
            self.model_loading = False

            rospy.logerr(
                "❌ Task2 YOLO模型加载失败: %s",
                self.model_error
            )


    # =========================================================
    # 菜谱名称统一
    # =========================================================

    def normalize_ingredient(
        self,
        name
    ):

        aliases = {
            "番茄": "西红柿",
            "tomato": "西红柿",

            "虾": "虾仁",
            "shrimp": "虾仁",

            "白菜": "上海青",
            "青菜": "上海青",
            "leaves": "上海青",
            "shanghaiqing": "上海青",

            "corn": "玉米",
            "cucumber": "黄瓜",
            "meat": "猪肉",
            "pork": "猪肉",
            "eggplant": "茄子",
            "potato": "土豆",
            "egg": "鸡蛋",
            "salmon": "三文鱼",
            "green pepper": "青椒",
            "greenpepper": "青椒",
            "tofu": "豆腐",
            "banana": "香蕉",
            "apple": "苹果",
        }

        text = str(
            name
        ).strip()

        lower = text.lower()

        return aliases.get(
            lower,
            aliases.get(
                text,
                text
            )
        )


    # =========================================================
    # 菜谱
    # =========================================================

    def load_recipes(
        self
    ):

        if not os.path.exists(
            self.recipes_path
        ):

            rospy.logerr(
                "❌ recipes.json 不存在: %s",
                self.recipes_path
            )

            return

        try:

            with open(
                self.recipes_path,
                "r",
                encoding="utf-8"
            ) as f:

                data = json.load(
                    f
                )

        except Exception as e:

            rospy.logerr(
                "❌ recipes.json 加载失败: %r",
                e
            )

            return

        for item in data.get(
            "recipes",
            []
        ):

            ingredients = [
                self.normalize_ingredient(
                    x
                )
                for x in item.get(
                    "ingredients",
                    []
                )
            ]

            ingredients = list(
                dict.fromkeys(
                    ingredients
                )
            )

            if len(
                ingredients
            ) != 3:

                continue

            key = tuple(
                sorted(
                    ingredients
                )
            )

            self.recipe_map[
                key
            ] = item

        self.fallback_rules = data.get(
            "fallback_rules",
            {}
        )

        rospy.loginfo(
            "📖 已加载 %d 个三食材菜谱",
            len(
                self.recipe_map
            )
        )


    # =========================================================
    # 摄像头
    # =========================================================

    def try_open_food_camera(
        self
    ):

        # 已正常打开则永远不重复打开
        if (
            self.cap is not None
            and
            self.cap.isOpened()
        ):

            return True

        now = time.monotonic()

        if (
            now
            -
            self.last_camera_retry_time
            <
            self.camera_retry_interval
        ):

            return False

        self.last_camera_retry_time = now

        for device in self.camera_devices:

            if not os.path.exists(
                device
            ):

                rospy.logwarn_throttle(
                    2.0,
                    "⚠️ 食材摄像头候选不存在: %s",
                    device
                )

                continue

            rospy.loginfo(
                "📷 尝试任务二食材摄像头: %s",
                device
            )

            cap = cv2.VideoCapture(
                device,
                cv2.CAP_V4L2
            )

            if not cap.isOpened():

                cap.release()

                continue

            cap.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                self.camera_width
            )

            cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                self.camera_height
            )

            cap.set(
                cv2.CAP_PROP_FPS,
                self.camera_fps
            )

            valid = False

            for _ in range(
                20
            ):

                ret, frame = cap.read()

                if (
                    ret
                    and
                    frame is not None
                    and
                    frame.size > 0
                ):

                    valid = True

                    break

                time.sleep(
                    0.03
                )

            if not valid:

                cap.release()

                rospy.logwarn(
                    "⚠️ %s 可打开但不能正常读取彩色画面",
                    device
                )

                continue

            self.cap = cap
            self.camera_device_in_use = device

            rospy.loginfo(
                "✅ 任务二食材摄像头 = %s",
                device
            )

            rospy.loginfo(
                "📐 实际分辨率 = %dx%d",
                int(
                    cap.get(
                        cv2.CAP_PROP_FRAME_WIDTH
                    )
                ),
                int(
                    cap.get(
                        cv2.CAP_PROP_FRAME_HEIGHT
                    )
                )
            )

            return True

        return False


    # =========================================================
    # scheduler控制
    #
    # 这里只启停YOLO，不开关摄像头。
    # =========================================================

    def control_callback(
        self,
        msg
    ):

        if msg.data:

            if self.active:

                return

            self.active = True

            self.result_sent = False

            self.session_best_by_class = {}

            rospy.loginfo(
                "========================================"
            )

            rospy.loginfo(
                "🥬 任务二 YOLO = ON"
            )

            rospy.loginfo(
                "📷 食材摄像头保持实时显示: %s",
                str(
                    self.camera_device_in_use
                )
            )

            rospy.loginfo(
                "🎯 持续累计，直到 >=3 个不同食材类别"
            )

            rospy.loginfo(
                "========================================"
            )

        else:

            if self.active:

                rospy.loginfo(
                    "🥬 任务二 YOLO = OFF；摄像头继续显示"
                )

            self.active = False

            self.result_sent = False

            self.session_best_by_class = {}


    # =========================================================
    # YOLO单帧
    # =========================================================

    def infer_frame(
        self,
        frame
    ):

        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False
        )

        result = results[
            0
        ]

        # 当前帧同类去重：
        # 一个 class 只保存置信度最高的检测框
        best_by_class = {}

        if result.boxes is not None:

            for box in result.boxes:

                class_id = int(
                    box.cls[
                        0
                    ].item()
                )

                confidence = float(
                    box.conf[
                        0
                    ].item()
                )

                if (
                    class_id
                    not in
                    self.class_names_cn
                ):

                    continue

                old = best_by_class.get(
                    class_id
                )

                if (
                    old is None
                    or
                    confidence
                    >
                    old[
                        "confidence"
                    ]
                ):

                    best_by_class[
                        class_id
                    ] = {
                        "class_id":
                            class_id,

                        "name":
                            self.class_names_cn[
                                class_id
                            ],

                        "confidence":
                            confidence,

                        "bbox":
                            [
                                int(
                                    v
                                )
                                for v in
                                box.xyxy[
                                    0
                                ]
                                .detach()
                                .cpu()
                                .tolist()
                            ]
                    }

        unique_detections = sorted(
            best_by_class.values(),
            key=lambda item:
                item[
                    "confidence"
                ],
            reverse=True
        )

        return unique_detections


    # =========================================================
    # 菜谱匹配
    # =========================================================

    def choose_recipe(
        self,
        top3_names
    ):

        normalized = [
            self.normalize_ingredient(
                x
            )
            for x in top3_names
        ]

        key = tuple(
            sorted(
                normalized
            )
        )

        item = self.recipe_map.get(
            key
        )

        if item is not None:

            return item.get(
                "name",
                "家常三鲜菜"
            )

        fruits = {
            "苹果",
            "香蕉"
        }

        seafood = {
            "虾仁",
            "三文鱼"
        }

        proteins = {
            "猪肉",
            "鸡蛋",
            "豆腐",
            "虾仁",
            "三文鱼"
        }

        selected = set(
            normalized
        )

        if len(
            selected
            &
            fruits
        ) >= 2:

            return self.fallback_rules.get(
                "fruit",
                "三鲜果蔬沙拉"
            )

        if selected & seafood:

            return self.fallback_rules.get(
                "seafood",
                "三鲜海味什锦"
            )

        if selected & proteins:

            return self.fallback_rules.get(
                "protein",
                "家常三鲜小炒"
            )

        return self.fallback_rules.get(
            "vegetable",
            "田园三鲜素炒"
        )


    # =========================================================
    # 成功
    # =========================================================

    def finish_success(
        self,
        top3
    ):

        if self.result_sent:

            return

        self.result_sent = True

        names = [
            item[
                "name"
            ]
            for item in top3
        ]

        recipe_name = self.choose_recipe(
            names
        )

        speech = (
            "里面有{}，{}，{}，可以做一个{}。"
        ).format(
            names[
                0
            ],
            names[
                1
            ],
            names[
                2
            ],
            recipe_name
        )

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "🥬 采用置信度最高的三个不同食材："
        )

        for item in top3:

            rospy.loginfo(
                "  %s : %.3f",
                item[
                    "name"
                ],
                item[
                    "confidence"
                ]
            )

        rospy.loginfo(
            "🍳 推荐: %s",
            recipe_name
        )

        rospy.loginfo(
            "🔊 %s",
            speech
        )

        rospy.loginfo(
            "========================================"
        )

        # 只关本轮YOLO
        # 摄像头绝不关闭
        self.active = False

        self.result_pub.publish(
            String(
                data=speech
            )
        )


    # =========================================================
    # 画检测框
    # =========================================================

    def draw_detections(
        self,
        frame,
        unique_detections,
        session_unique_count
    ):

        output = frame.copy()

        for item in unique_detections:

            x1, y1, x2, y2 = item[
                "bbox"
            ]

            cv2.rectangle(
                output,
                (
                    x1,
                    y1
                ),
                (
                    x2,
                    y2
                ),
                (
                    0,
                    255,
                    0
                ),
                2
            )

            label = "ID{} {:.2f}".format(
                item[
                    "class_id"
                ],
                item[
                    "confidence"
                ]
            )

            cv2.putText(
                output,
                label,
                (
                    x1,
                    max(
                        20,
                        y1 - 6
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (
                    0,
                    255,
                    0
                ),
                2
            )

        cv2.putText(
            output,
            "TASK2 YOLO ON",
            (
                10,
                30
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (
                0,
                255,
                0
            ),
            2
        )

        cv2.putText(
            output,
            "Current unique: {}  Session unique: {}".format(
                len(
                    unique_detections
                ),
                session_unique_count
            ),
            (
                10,
                60
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (
                0,
                255,
                255
            ),
            2
        )

        return output


    # =========================================================
    # 普通实时预览状态
    # =========================================================

    def draw_preview_status(
        self,
        frame
    ):

        output = frame.copy()

        cv2.putText(
            output,
            "TASK2 CAMERA LIVE - YOLO OFF",
            (
                10,
                30
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (
                0,
                255,
                255
            ),
            2
        )

        if self.model_ready:

            text = "MODEL READY"

            color = (
                0,
                255,
                0
            )

        elif self.model_loading:

            text = "MODEL LOADING..."

            color = (
                0,
                255,
                255
            )

        else:

            text = "MODEL ERROR"

            color = (
                0,
                0,
                255
            )

        cv2.putText(
            output,
            text,
            (
                10,
                60
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2
        )

        return output


    # =========================================================
    # 无摄像头时也保持窗口存在
    # =========================================================

    def show_waiting_camera(
        self
    ):

        if not self.show_window:

            return

        blank = np.zeros(
            (
                self.camera_height,
                self.camera_width,
                3
            ),
            dtype=np.uint8
        )

        cv2.putText(
            blank,
            "WAITING FOOD CAMERA...",
            (
                30,
                60
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (
                0,
                255,
                255
            ),
            2
        )

        cv2.putText(
            blank,
            "/dev/video4  /dev/video5",
            (
                30,
                100
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (
                255,
                255,
                255
            ),
            2
        )

        cv2.imshow(
            self.WINDOW_NAME,
            blank
        )

        cv2.waitKey(
            1
        )


    # =========================================================
    # 主循环
    # =========================================================

    def run(
        self
    ):

        rate = rospy.Rate(
            20
        )

        while not rospy.is_shutdown():

            # -------------------------------------------------
            # 摄像头还没打开时：
            # 窗口仍然保持存在，并不断重试 /dev/video4/5。
            # -------------------------------------------------

            if (
                self.cap is None
                or
                not self.cap.isOpened()
            ):

                self.show_waiting_camera()

                self.try_open_food_camera()

                rate.sleep()

                continue


            ret, frame = self.cap.read()


            if (
                not ret
                or
                frame is None
            ):

                rospy.logwarn_throttle(
                    2.0,
                    "⚠️ 食材摄像头读帧失败: %s",
                    str(
                        self.camera_device_in_use
                    )
                )

                # 正常任务状态下不 release。
                # 保持设备句柄，下一轮继续读。
                rate.sleep()

                continue


            # -------------------------------------------------
            # 未执行任务二：
            # 实时显示第二摄像头，但不跑YOLO。
            # -------------------------------------------------

            if not self.active:

                if self.show_window:

                    preview = (
                        self.draw_preview_status(
                            frame
                        )
                    )

                    cv2.imshow(
                        self.WINDOW_NAME,
                        preview
                    )

                    cv2.waitKey(
                        1
                    )

                rate.sleep()

                continue


            # -------------------------------------------------
            # 任务二已触发，但模型仍在后台加载
            # -------------------------------------------------

            if not self.model_ready:

                if self.show_window:

                    waiting = frame.copy()

                    if self.model_loading:

                        text = (
                            "TASK2 ACTIVE - WAITING MODEL..."
                        )

                    else:

                        text = (
                            "TASK2 MODEL ERROR"
                        )

                    cv2.putText(
                        waiting,
                        text,
                        (
                            10,
                            30
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (
                            0,
                            255,
                            255
                        ),
                        2
                    )

                    cv2.imshow(
                        self.WINDOW_NAME,
                        waiting
                    )

                    cv2.waitKey(
                        1
                    )

                rate.sleep()

                continue


            # -------------------------------------------------
            # YOLO识别
            # -------------------------------------------------

            try:

                unique_detections = (
                    self.infer_frame(
                        frame
                    )
                )

            except Exception as e:

                rospy.logerr_throttle(
                    2.0,
                    "❌ Task2 YOLO推理失败: %r",
                    e
                )

                rate.sleep()

                continue


            # -------------------------------------------------
            # 跨帧累计不同类别
            # -------------------------------------------------

            for item in unique_detections:

                class_id = item[
                    "class_id"
                ]

                old = (
                    self.session_best_by_class
                    .get(
                        class_id
                    )
                )

                if (
                    old is None
                    or
                    item[
                        "confidence"
                    ]
                    >
                    old[
                        "confidence"
                    ]
                ):

                    self.session_best_by_class[
                        class_id
                    ] = item


            session_unique = sorted(
                self.session_best_by_class.values(),
                key=lambda item:
                    item[
                        "confidence"
                    ],
                reverse=True
            )


            rospy.loginfo_throttle(
                1.0,
                "🥬 当前帧不同类别=%d，本轮累计不同类别=%d",
                len(
                    unique_detections
                ),
                len(
                    session_unique
                )
            )


            if self.show_window:

                display = (
                    self.draw_detections(
                        frame,
                        unique_detections,
                        len(
                            session_unique
                        )
                    )
                )

                cv2.imshow(
                    self.WINDOW_NAME,
                    display
                )

                cv2.waitKey(
                    1
                )


            if len(
                session_unique
            ) >= 3:

                top3 = session_unique[
                    :3
                ]

                self.finish_success(
                    top3
                )


            rate.sleep()


    # =========================================================
    # 只有节点真正退出才关闭食材摄像头
    # =========================================================

    def shutdown(
        self
    ):

        self.active = False

        if self.cap is not None:

            try:

                self.cap.release()

            except Exception:

                pass

            self.cap = None

        try:

            cv2.destroyAllWindows()

        except Exception:

            pass

        rospy.loginfo(
            "🛑 task2_food_detector 已关闭"
        )


if __name__ == "__main__":

    node = None

    try:

        node = Task2FoodDetector()

        node.run()

    except rospy.ROSInterruptException:

        pass

    finally:

        if node is not None:

            node.shutdown()
