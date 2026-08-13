#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file task2_detector.py
@brief 任务二：食材识别节点
       1. 订阅 /task3_detection_enable 控制信号（与调度器共用）
       2. 到达厨房后，识别摄像头画面中的食材（13类）
       3. 1.5秒检测窗口，多帧统计
       4. 发布逗号分隔的食材名称列表到 /task3_detection_result
       5. 发布调试画面到 /task2_vision/debug_image
"""

import time
import threading
import json
import sys

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class Task2Detector:

    def __init__(self):

        rospy.init_node(
            "task2_detector",
            anonymous=False
        )

        # =====================================================
        # 参数
        # =====================================================

        self.model_path = rospy.get_param(
            "~model_path",
            "/home/reicom2025/bobac3_ws/src/model/best.pt"
        )

        # 使用仿真机器人的 Berxel 摄像头
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/berxel_base/color/image_raw"
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

        self.max_fps = float(
            rospy.get_param(
                "~max_fps",
                10.0
            )
        )

        # 有效检测窗口（与调度器 TASK2_DETECTING 匹配）
        self.detect_seconds = float(
            rospy.get_param(
                "~detect_seconds",
                1.5
            )
        )

        # 某食材至少命中多少帧才算存在
        self.min_hit_frames = int(
            rospy.get_param(
                "~min_hit_frames",
                3
            )
        )

        # 是否显示 OpenCV 实时窗口
        self.show_window = bool(
            rospy.get_param(
                "~show_window",
                True
            )
        )

        # 内部保护超时
        self.internal_timeout = float(
            rospy.get_param(
                "~internal_timeout",
                8.0
            )
        )

        # =====================================================
        # 模型
        # =====================================================

        self.bridge = CvBridge()

        rospy.loginfo(
            "🤖 正在加载任务二模型: %s",
            self.model_path
        )

        self.model = YOLO(
            self.model_path
        )

        # =====================================================
        # 食材类别映射（14类）
        # =====================================================

        # 按你的需求：只要识别到的食材名称
        # 如果模型类别名称与下面匹配，自动映射
        # 如果不对应，需要手动设置映射
        self.ingredient_class_names = {
            # 假设你的模型类别名称（根据训练时的名称）
            # 如果实际名称不同，请修改这里
            "banana": "香蕉",
            "apple": "苹果",
            "salmon": "三文鱼",
            "potato": "土豆",
            "tomato": "西红柿",
            "shrimp": "虾",
            "cucumber": "黄瓜",
            "corn": "玉米",
            "green pepper": "青椒",
            "leaves": "白菜",
            "tofu": "豆腐",
            "egg": "鸡蛋",
            "meat": "猪肉",
            # 如果还有第14类，在这里添加
        }

        # 反向映射：中文名 -> 英文名（用于输出）
        self.cn_to_en = {v: k for k, v in self.ingredient_class_names.items()}

        # 所有可识别的食材中文名列表
        self.valid_ingredients = set(self.ingredient_class_names.values())

        # 类别ID -> 中文名映射（从模型加载后构建）
        self.id_to_cn_name = {}

        self._build_class_mapping()

        # =====================================================
        # 摄像头帧缓存
        # =====================================================

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.latest_seq = 0
        self.last_processed_seq = -1

        # =====================================================
        # YOLO预热
        # =====================================================

        self.model_warmed_up = False

        # =====================================================
        # 一轮检测状态
        # =====================================================

        self.active = False
        self.session_request_time = None
        self.detection_start_time = None

        # 统计字典：食材名 -> 命中帧数
        self.hit_counts = {}

        # 已识别的食材集合
        self.detected_ingredients = set()

        self.processed_frames = 0

        # 最近结果
        self.last_result = ""
        self.last_result_time = 0.0

        self.shutdown_done = False

        # =====================================================
        # ROS Publisher
        # =====================================================

        # 调试画面（与任务三共用话题名，方便 rqt 查看）
        self.debug_pub = rospy.Publisher(
            "/task2_vision/debug_image",
            Image,
            queue_size=1
        )

        # 每一帧的详细识别信息
        self.frame_result_pub = rospy.Publisher(
            "/task2_vision/frame_detections",
            String,
            queue_size=10
        )

        # 最终结果：逗号分隔的食材名，如 "西红柿,鸡蛋,青椒"
        # 或 "none" / "error"
        self.result_pub = rospy.Publisher(
            "/task3_detection_result",  # 与调度器订阅的话题一致
            String,
            queue_size=10
        )

        # =====================================================
        # ROS Subscriber
        # =====================================================

        self.control_sub = rospy.Subscriber(
            "/task3_detection_enable",  # 与调度器发布的话题一致
            Bool,
            self.control_callback,
            queue_size=1
        )

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.on_shutdown(
            self.shutdown
        )

        # =====================================================
        # 启动日志
        # =====================================================

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "✅ task2_detector 已启动（食材识别）"
        )

        rospy.loginfo(
            "🤖 模型: %s",
            self.model_path
        )

        rospy.loginfo(
            "📷 机器人摄像头: %s",
            self.image_topic
        )

        rospy.loginfo(
            "🏷️ 可识别食材: %s",
            ", ".join(sorted(self.valid_ingredients))
        )

        rospy.loginfo(
            "⏱️ 有效检测窗口: %.1f 秒",
            self.detect_seconds
        )

        rospy.loginfo(
            "🧮 至少命中 %d 帧才算存在",
            self.min_hit_frames
        )

        rospy.loginfo(
            "📡 发布话题: /task3_detection_result"
        )

        rospy.loginfo(
            "📡 调试话题: /task2_vision/debug_image"
        )

        rospy.loginfo(
            "========================================"
        )


    # =========================================================
    # 构建类别映射
    # =========================================================

    def _build_class_mapping(self):
        """从模型类别名称构建 ID -> 中文名 映射"""
        # 注意：需要先加载模型才能访问 model.names
        # 但模型在 __init__ 中已加载，可以在 __init__ 中延迟调用
        pass

    def _ensure_class_mapping(self):
        """确保映射已构建（延迟初始化）"""
        if self.id_to_cn_name:
            return

        if not hasattr(self.model, 'names'):
            rospy.logwarn("⚠️ 模型没有 names 属性，无法自动映射类别")
            return

        names = self.model.names
        if isinstance(names, dict):
            items = names.items()
        else:
            items = enumerate(names)

        for class_id, class_name in items:
            class_name_str = str(class_name).strip().lower()
            # 尝试匹配
            if class_name_str in self.ingredient_class_names:
                cn_name = self.ingredient_class_names[class_name_str]
                self.id_to_cn_name[class_id] = cn_name
                rospy.logdebug(
                    "📌 类别 %d (%s) -> %s",
                    class_id,
                    class_name_str,
                    cn_name
                )

        if not self.id_to_cn_name:
            rospy.logwarn(
                "⚠️ 未能自动匹配任何食材类别，请检查 model.names 与 ingredient_class_names 是否匹配"
            )
            rospy.logwarn(
                "📋 model.names = %s",
                str(names)
            )


    # =========================================================
    # 摄像头回调
    # =========================================================

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            rospy.logerr_throttle(
                2.0,
                "❌ CvBridge 转换失败: %s",
                str(e)
            )
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.latest_header = msg.header
            self.latest_seq += 1


    # =========================================================
    # 控制回调（与调度器同步）
    # =========================================================

    def control_callback(self, msg):
        if msg.data:
            if self.active:
                return

            self.active = True
            self.session_request_time = time.monotonic()
            self.detection_start_time = None
            self.hit_counts = {}
            self.detected_ingredients = set()
            self.processed_frames = 0

            with self.frame_lock:
                self.last_processed_seq = self.latest_seq

            rospy.loginfo(
                "========================================"
            )
            rospy.loginfo(
                "🍅 收到任务二检测命令（食材识别）"
            )
            rospy.loginfo(
                "⏳ 等待第一帧YOLO推理..."
            )
            rospy.loginfo(
                "========================================"
            )

        else:
            if self.active:
                rospy.loginfo(
                    "🍅 收到停止命令，取消当前检测"
                )
            self.active = False
            self.session_request_time = None
            self.detection_start_time = None


    # =========================================================
    # 获取最新帧
    # =========================================================

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None, None, -1
            return (
                self.latest_frame.copy(),
                self.latest_header,
                self.latest_seq
            )


    # =========================================================
    # YOLO 推理
    # =========================================================

    def infer(self, frame):
        """执行推理，返回 (标注图, 识别的食材名列表, 详细信息)"""
        # 确保映射已构建
        self._ensure_class_mapping()

        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False
        )

        result = results[0]

        detected = []
        details = []

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())

                # 获取食材中文名
                if class_id in self.id_to_cn_name:
                    ingredient = self.id_to_cn_name[class_id]
                else:
                    # 如果类别未映射，尝试用模型原始名称
                    raw_name = self.model.names[class_id] if class_id in self.model.names else str(class_id)
                    ingredient = raw_name
                    rospy.logdebug(
                        "⚠️ 未映射类别 %d (%s)",
                        class_id,
                        raw_name
                    )

                # 只记录有效食材（在 valid_ingredients 中）
                if ingredient in self.valid_ingredients:
                    detected.append(ingredient)
                    details.append({
                        "class_id": class_id,
                        "name": ingredient,
                        "confidence": round(confidence, 3),
                        "bbox": [
                            round(float(v), 1)
                            for v in box.xyxy[0].cpu().tolist()
                        ]
                    })

        # 去重（同一帧中同种食材只算一次）
        detected_unique = list(set(detected))

        # 标注图
        annotated = result.plot()

        return annotated, detected_unique, details


    # =========================================================
    # 模型预热
    # =========================================================

    def warmup_model(self, frame):
        if self.model_warmed_up:
            return True

        try:
            rospy.loginfo(
                "🔥 正在使用机器人摄像头预热YOLO模型..."
            )
            self.model.predict(
                source=frame,
                conf=self.conf,
                imgsz=self.imgsz,
                verbose=False
            )
            self.model_warmed_up = True
            rospy.loginfo(
                "✅ YOLO模型预热完成"
            )
            return True
        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "⚠️ 预热失败: %s",
                str(e)
            )
            return False


    # =========================================================
    # 结束检测会话
    # =========================================================

    def finish_session(self):
        if self.processed_frames < self.min_hit_frames:
            final_result = "error"
            rospy.logwarn(
                "⚠️ 有效帧数不足 %d 帧，结果为 error",
                self.min_hit_frames
            )
        else:
            # 收集命中的食材
            found_ingredients = []
            for ingredient, count in self.hit_counts.items():
                if count >= self.min_hit_frames:
                    found_ingredients.append(ingredient)

            if found_ingredients:
                final_result = ",".join(sorted(found_ingredients))
            else:
                final_result = "none"

        rospy.loginfo(
            "========================================"
        )
        rospy.loginfo(
            "🍅 任务二识别窗口结束"
        )
        rospy.loginfo(
            "📊 处理帧数: %d",
            self.processed_frames
        )
        rospy.loginfo(
            "📊 各食材命中帧数:"
        )
        for ingredient, count in sorted(self.hit_counts.items()):
            rospy.loginfo(
                "   %s: %d 帧",
                ingredient,
                count
            )
        rospy.loginfo(
            "📤 最终结果: [%s]",
            final_result
        )
        rospy.loginfo(
            "========================================"
        )

        self.last_result = final_result
        self.last_result_time = time.monotonic()

        self.active = False
        self.session_request_time = None
        self.detection_start_time = None

        self.result_pub.publish(
            String(
                data=final_result
            )
        )


    # =========================================================
    # 超时错误
    # =========================================================

    def finish_with_error(self, reason):
        rospy.logerr(
            "❌ 任务二视觉链路异常: %s",
            reason
        )

        self.last_result = "error"
        self.last_result_time = time.monotonic()

        self.active = False
        self.session_request_time = None
        self.detection_start_time = None

        self.result_pub.publish(
            String(
                data="error"
            )
        )


    # =========================================================
    # 绘制调试信息
    # =========================================================

    def draw_status(self, frame):
        output = frame.copy()

        if self.active:
            if self.detection_start_time is None:
                text = "TASK2: WAITING FIRST YOLO FRAME"
            else:
                elapsed = time.monotonic() - self.detection_start_time
                hit_str = ", ".join(
                    [
                        "{}:{}".format(k, v)
                        for k, v in sorted(self.hit_counts.items())
                    ]
                ) if self.hit_counts else "无"
                text = "TASK2: {:.1f}/{:.1f}s  食材: {}".format(
                    min(elapsed, self.detect_seconds),
                    self.detect_seconds,
                    hit_str
                )
        else:
            text = "TASK2: 空闲（等待命令）"

        cv2.putText(
            output,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )

        if self.last_result and time.monotonic() - self.last_result_time < 2.0:
            result_text = "结果: {}".format(self.last_result)
            cv2.putText(
                output,
                result_text,
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (0, 255, 0),
                2
            )

        return output


    # =========================================================
    # 发布调试图像
    # =========================================================

    def publish_debug(self, frame, header):
        if self.debug_pub.get_num_connections() <= 0:
            return

        try:
            debug_frame = np.asarray(frame)

            if debug_frame.dtype != np.uint8:
                debug_frame = np.clip(debug_frame, 0, 255).astype(np.uint8)

            if len(debug_frame.shape) == 2:
                debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_GRAY2BGR)
            elif len(debug_frame.shape) == 3 and debug_frame.shape[2] == 4:
                debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_BGRA2BGR)

            debug_frame = np.ascontiguousarray(debug_frame)
            msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")

            if header is not None:
                msg.header = header

            self.debug_pub.publish(msg)

        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "发布 debug_image 失败: %s",
                str(e)
            )


    # =========================================================
    # 主循环
    # =========================================================

    def run(self):
        rate = rospy.Rate(self.max_fps)

        while not rospy.is_shutdown():
            frame, header, seq = self.get_latest_frame()

            if frame is None:
                rospy.loginfo_throttle(
                    2.0,
                    "⏳ 等待机器人摄像头图像: %s",
                    self.image_topic
                )
                rate.sleep()
                continue

            # 预热
            if not self.model_warmed_up:
                self.warmup_model(frame)

            display = frame.copy()

            # =====================================================
            # 任务二检测状态
            # =====================================================

            if self.active:
                # 超时保护（等待第一帧YOLO）
                if (self.session_request_time is not None
                        and self.detection_start_time is None
                        and time.monotonic() - self.session_request_time > self.internal_timeout):
                    self.finish_with_error(
                        "超过 {:.1f} 秒仍没有第一帧有效YOLO推理".format(
                            self.internal_timeout
                        )
                    )
                    rate.sleep()
                    continue

                # 只对新帧推理
                if seq != self.last_processed_seq:
                    self.last_processed_seq = seq

                    try:
                        annotated, detected, details = self.infer(frame)

                        # 第一帧推理成功才计时
                        if self.detection_start_time is None:
                            self.detection_start_time = time.monotonic()
                            rospy.loginfo(
                                "✅ 第一帧YOLO推理成功，开始 %.1f 秒检测窗口",
                                self.detect_seconds
                            )

                        display = annotated
                        self.processed_frames += 1

                        # 统计命中
                        for ingredient in detected:
                            if ingredient in self.hit_counts:
                                self.hit_counts[ingredient] += 1
                            else:
                                self.hit_counts[ingredient] = 1

                        # 发布帧详情
                        self.frame_result_pub.publish(
                            String(
                                data=json.dumps(
                                    {
                                        "detected": detected,
                                        "details": details
                                    },
                                    ensure_ascii=False
                                )
                            )
                        )

                    except Exception as e:
                        rospy.logerr_throttle(
                            1.0,
                            "❌ YOLO推理失败: %s",
                            str(e)
                        )

                # 判断是否结束
                if (self.active
                        and self.detection_start_time is not None):
                    elapsed = time.monotonic() - self.detection_start_time

                    if (elapsed >= self.detect_seconds
                            and self.processed_frames >= self.min_hit_frames):
                        self.finish_session()

            # =====================================================
            # 绘制并发布
            # =====================================================

            display = self.draw_status(display)
            self.publish_debug(display, header)

            if self.show_window:
                cv2.imshow(
                    "Task2 Ingredient Detector",
                    display
                )
                key = cv2.waitKey(1) & 0xFF
                if key == ord('e'):
                    rospy.signal_shutdown("User pressed E")
                    break

            rate.sleep()


    # =========================================================
    # 关闭
    # =========================================================

    def shutdown(self):
        if self.shutdown_done:
            return

        self.shutdown_done = True
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        rospy.loginfo(
            "🛑 task2_detector 已关闭"
        )


# =============================================================
# main
# =============================================================

if __name__ == "__main__":

    node = None

    try:
        node = Task2Detector()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()