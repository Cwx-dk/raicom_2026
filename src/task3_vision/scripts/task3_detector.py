#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import json

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class Task3Detector:

    def __init__(self):

        rospy.init_node(
            "task3_detector",
            anonymous=False
        )

        # =====================================================
        # 参数
        # =====================================================

        self.model_path = rospy.get_param(
            "~model_path",
            "/home/reicom2025/bobac3_ws/src/model/best.pt"
        )

        # 明确使用仿真机器人的 Berxel 摄像头
        # 不使用电脑摄像头，不使用 cv2.VideoCapture(0)
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/top_camera/image_raw"
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

        # 有效检测窗口：
        # 从“第一帧 YOLO 成功完成”开始计算，而不是从收到控制命令开始
        self.detect_seconds = float(
            rospy.get_param(
                "~detect_seconds",
                1.5
            )
        )

        # 某类别至少命中多少帧才算存在
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

        # detector 自己的保护超时。
        # 正常仍然只识别 1.5 秒；
        # 只有第一帧推理一直无法成功时才触发 error。
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
            "🤖 正在加载任务三模型: %s",
            self.model_path
        )

        self.model = YOLO(
            self.model_path
        )

        self.phone_ids = set()
        self.backpack_ids = set()

        self._resolve_class_ids()

        # =====================================================
        # 摄像头帧缓存
        # =====================================================

        self.frame_lock = threading.Lock()

        self.latest_frame = None
        self.latest_header = None

        # 每收到一个新的 ROS 图像就 +1
        self.latest_seq = 0

        # 防止重复推理同一张图
        self.last_processed_seq = -1

        # =====================================================
        # YOLO预热
        # =====================================================

        self.model_warmed_up = False

        # =====================================================
        # 一轮任务三检测状态
        # =====================================================

        self.active = False

        # 收到 scheduler 开启命令的真实时间
        self.session_request_time = None

        # 第一帧 YOLO 真正成功之后才赋值
        # 1.5 秒从这里开始计算
        self.detection_start_time = None

        self.phone_hit_frames = 0
        self.backpack_hit_frames = 0
        self.processed_frames = 0

        # 最近一次识别结果，用于画面显示
        self.last_result = ""
        self.last_result_time = 0.0

        # 避免 shutdown 重复打印
        self.shutdown_done = False

        # =====================================================
        # ROS Publisher
        # =====================================================

        # 带状态/识别框的调试画面
        self.debug_pub = rospy.Publisher(
            "/task3_vision/debug_image",
            Image,
            queue_size=1
        )

        # 每一帧的详细识别信息
        self.frame_result_pub = rospy.Publisher(
            "/task3_vision/frame_detections",
            String,
            queue_size=10
        )

        # 一轮 1.5 秒最终结果：
        # none / phone / backpack / both / error
        self.result_pub = rospy.Publisher(
            "/task3_detection_result",
            String,
            queue_size=10
        )

        # =====================================================
        # ROS Subscriber
        # =====================================================

        self.control_sub = rospy.Subscriber(
            "/task3_detection_enable",
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
            "✅ task3_detector 已启动"
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
            "🏷️ 模型类别: %s",
            str(self.model.names)
        )

        rospy.loginfo(
            "📱 phone class ids: %s",
            sorted(self.phone_ids)
        )

        rospy.loginfo(
            "🎒 backpack class ids: %s",
            sorted(self.backpack_ids)
        )

        rospy.loginfo(
            "⏱️ 有效检测窗口: %.1f 秒",
            self.detect_seconds
        )

        rospy.loginfo(
            "🧮 至少命中 %d 帧才判定目标存在",
            self.min_hit_frames
        )

        rospy.loginfo(
            "📺 OpenCV实时窗口: %s",
            "开启" if self.show_window else "关闭"
        )

        rospy.loginfo(
            "📡 ROS调试图像: /task3_vision/debug_image"
        )

        rospy.loginfo(
            "========================================"
        )


    # =========================================================
    # 自动解析手机/书包类别 ID
    # =========================================================

    def _resolve_class_ids(self):

        phone_aliases = {
            "phone",
            "cell phone",
            "cellphone",
            "mobile phone",
            "mobile",
            "手机"
        }

        backpack_aliases = {
            "backpack",
            "bag",
            "bookbag",
            "schoolbag",
            "school bag",
            "书包"
        }

        if isinstance(
            self.model.names,
            dict
        ):
            items = self.model.names.items()

        else:
            items = enumerate(
                self.model.names
            )

        for class_id, class_name in items:

            name = str(
                class_name
            ).strip().lower()

            if name in phone_aliases:
                self.phone_ids.add(
                    int(class_id)
                )

            if name in backpack_aliases:
                self.backpack_ids.add(
                    int(class_id)
                )

        # 如果自动类别名匹配失败，可以从 launch 手动指定
        manual_phone_id = int(
            rospy.get_param(
                "~phone_class_id",
                -1
            )
        )

        manual_backpack_id = int(
            rospy.get_param(
                "~backpack_class_id",
                -1
            )
        )

        if manual_phone_id >= 0:
            self.phone_ids = {
                manual_phone_id
            }

        if manual_backpack_id >= 0:
            self.backpack_ids = {
                manual_backpack_id
            }

        if (
            not self.phone_ids
            or
            not self.backpack_ids
        ):

            rospy.logwarn(
                "⚠️ 未能完整自动匹配 phone/backpack 类别。"
                "请查看启动日志里的 model.names；"
                "必要时在 test.launch 手动设置类别 ID。"
            )


    # =========================================================
    # 机器人摄像头回调
    # =========================================================

    def image_callback(
        self,
        msg
    ):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except CvBridgeError as e:

            rospy.logerr_throttle(
                2.0,
                "❌ CvBridge 图像转换失败: type=%s repr=%r",
                type(e).__name__,
                e
            )

            return

        with self.frame_lock:

            self.latest_frame = frame.copy()

            self.latest_header = msg.header

            self.latest_seq += 1


    # =========================================================
    # scheduler 控制是否执行任务三检测
    # =========================================================

    def control_callback(
        self,
        msg
    ):

        if msg.data:

            # 已经在识别时忽略重复 true
            if self.active:
                return

            self.active = True

            self.session_request_time = (
                time.monotonic()
            )

            # 关键：
            # 现在还没有开始 1.5 秒有效识别窗口
            self.detection_start_time = None

            self.phone_hit_frames = 0
            self.backpack_hit_frames = 0
            self.processed_frames = 0

            # 只处理收到命令之后的新图像
            with self.frame_lock:
                self.last_processed_seq = (
                    self.latest_seq
                )

            rospy.loginfo(
                "========================================"
            )

            rospy.loginfo(
                "📷 收到任务三检测命令"
            )

            rospy.loginfo(
                "⏳ 等待收到命令后的第一张新图像并完成YOLO推理..."
            )

            rospy.loginfo(
                "========================================"
            )

        else:

            if self.active:

                rospy.loginfo(
                    "📷 收到关闭命令，停止当前任务三检测"
                )

            self.active = False
            self.session_request_time = None
            self.detection_start_time = None


    # =========================================================
    # 取得最新帧
    # =========================================================

    def get_latest_frame(
        self
    ):

        with self.frame_lock:

            if self.latest_frame is None:

                return (
                    None,
                    None,
                    -1
                )

            return (
                self.latest_frame.copy(),
                self.latest_header,
                self.latest_seq
            )


    # =========================================================
    # 单帧 YOLO
    # =========================================================

    def infer(
        self,
        frame
    ):

        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False
        )

        result = results[0]

        phone = False
        backpack = False

        details = []

        if result.boxes is not None:

            for box in result.boxes:

                class_id = int(
                    box.cls[0].item()
                )

                confidence = float(
                    box.conf[0].item()
                )

                class_name = str(
                    self.model.names[
                        class_id
                    ]
                )

                if class_id in self.phone_ids:
                    phone = True

                if class_id in self.backpack_ids:
                    backpack = True

                if (
                    class_id in self.phone_ids
                    or
                    class_id in self.backpack_ids
                ):

                    details.append(
                        {
                            "class_id":
                                class_id,

                            "class_name":
                                class_name,

                            "confidence":
                                round(
                                    confidence,
                                    3
                                ),

                            "bbox":
                                [
                                    round(
                                        float(v),
                                        1
                                    )
                                    for v in
                                    box.xyxy[
                                        0
                                    ].cpu().tolist()
                                ]
                        }
                    )

        # Ultralytics 自动画识别框
        annotated = result.plot()

        return (
            annotated,
            phone,
            backpack,
            details
        )


    # =========================================================
    # 模型预热
    #
    # 只执行一次，不计入任务三的1.5秒窗口。
    # =========================================================

    def warmup_model(
        self,
        frame
    ):

        if self.model_warmed_up:
            return True

        try:

            rospy.loginfo(
                "🔥 正在使用机器人摄像头图像预热YOLO模型..."
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
                "⚠️ YOLO模型预热失败: type=%s repr=%r",
                type(e).__name__,
                e
            )

            return False


    # =========================================================
    # 结束一轮多帧识别
    # =========================================================

    def finish_session(
        self
    ):

        # 正常情况下走到这里应该已经 >= min_hit_frames
        if (
            self.processed_frames
            <
            self.min_hit_frames
        ):

            final_result = "error"

        else:

            phone = (
                self.phone_hit_frames
                >=
                self.min_hit_frames
            )

            backpack = (
                self.backpack_hit_frames
                >=
                self.min_hit_frames
            )

            if phone and backpack:
                final_result = "both"

            elif phone:
                final_result = "phone"

            elif backpack:
                final_result = "backpack"

            else:
                final_result = "none"

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "📊 任务三识别窗口结束"
        )

        rospy.loginfo(
            "处理帧数: %d",
            self.processed_frames
        )

        rospy.loginfo(
            "手机命中帧: %d",
            self.phone_hit_frames
        )

        rospy.loginfo(
            "书包命中帧: %d",
            self.backpack_hit_frames
        )

        rospy.loginfo(
            "最终结果: %s",
            final_result
        )

        rospy.loginfo(
            "========================================"
        )

        # 先保存状态，再发布结果
        self.last_result = final_result

        self.last_result_time = (
            time.monotonic()
        )

        self.active = False

        self.session_request_time = None
        self.detection_start_time = None

        self.result_pub.publish(
            String(
                data=final_result
            )
        )


    # =========================================================
    # detector 内部故障保护
    # =========================================================

    def finish_with_error(
        self,
        reason
    ):

        rospy.logerr(
            "❌ 任务三视觉链路异常: %s",
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
    # 调试画面文字
    # =========================================================

    def draw_status(
        self,
        frame
    ):

        output = frame.copy()

        if self.active:

            if self.detection_start_time is None:

                text = (
                    "TASK3 WAITING FIRST YOLO FRAME"
                )

            else:

                elapsed = (
                    time.monotonic()
                    -
                    self.detection_start_time
                )

                text = (
                    "DETECTING "
                    "{:.1f}/{:.1f}s "
                    "phone:{} backpack:{}"
                ).format(
                    min(
                        elapsed,
                        self.detect_seconds
                    ),
                    self.detect_seconds,
                    self.phone_hit_frames,
                    self.backpack_hit_frames
                )

        else:

            text = (
                "TASK3 CAMERA READY - detection OFF"
            )

        cv2.putText(
            output,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2
        )

        if (
            self.last_result
            and
            time.monotonic()
            -
            self.last_result_time
            <
            2.0
        ):

            cv2.putText(
                output,
                "RESULT: {}".format(
                    self.last_result
                ),
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (0, 255, 0),
                2
            )

        return output


    # =========================================================
    # 发布 /task3_vision/debug_image
    # =========================================================

    def publish_debug(
        self,
        frame,
        header
    ):

        # 没有 rqt_image_view 等订阅者时就不做 ROS 图像编码，
        # 可减少CPU占用，也避免不必要的 debug_image 报错。
        if (
            self.debug_pub.get_num_connections()
            <=
            0
        ):
            return

        try:

            debug_frame = np.asarray(
                frame
            )

            # 统一处理为三通道 uint8 BGR
            if debug_frame.dtype != np.uint8:

                debug_frame = np.clip(
                    debug_frame,
                    0,
                    255
                ).astype(
                    np.uint8
                )

            if len(
                debug_frame.shape
            ) == 2:

                debug_frame = cv2.cvtColor(
                    debug_frame,
                    cv2.COLOR_GRAY2BGR
                )

            elif (
                len(debug_frame.shape) == 3
                and
                debug_frame.shape[2] == 4
            ):

                debug_frame = cv2.cvtColor(
                    debug_frame,
                    cv2.COLOR_BGRA2BGR
                )

            elif (
                len(debug_frame.shape) != 3
                or
                debug_frame.shape[2] != 3
            ):

                rospy.logwarn_throttle(
                    2.0,
                    "⚠️ debug_image图像形状异常: %s",
                    str(debug_frame.shape)
                )

                return

            debug_frame = (
                np.ascontiguousarray(
                    debug_frame
                )
            )

            msg = (
                self.bridge
                .cv2_to_imgmsg(
                    debug_frame,
                    encoding="bgr8"
                )
            )

            if header is not None:
                msg.header = header

            self.debug_pub.publish(
                msg
            )

        except Exception as e:

            rospy.logwarn_throttle(
                2.0,
                "发布 debug_image 失败: type=%s repr=%r",
                type(e).__name__,
                e
            )


    # =========================================================
    # 主循环
    # =========================================================

    def run(
        self
    ):

        rate = rospy.Rate(
            self.max_fps
        )

        while not rospy.is_shutdown():

            frame, header, seq = (
                self.get_latest_frame()
            )

            if frame is None:

                rospy.loginfo_throttle(
                    2.0,
                    "⏳ 等待机器人摄像头图像: %s",
                    self.image_topic
                )

                rate.sleep()

                continue

            # =================================================
            # 启动后预热一次模型
            # =================================================

            if not self.model_warmed_up:

                self.warmup_model(
                    frame
                )

            # 默认画原始机器人视角
            display = frame.copy()

            # =================================================
            # 任务三检测阶段
            # =================================================

            if self.active:

                # ---------------------------------------------
                # 如果长时间连第一帧有效YOLO都没有，
                # 主动返回error，不让scheduler无期限等待。
                # ---------------------------------------------

                if (
                    self.session_request_time
                    is not None
                    and
                    self.detection_start_time
                    is None
                    and
                    time.monotonic()
                    -
                    self.session_request_time
                    >
                    self.internal_timeout
                ):

                    self.finish_with_error(
                        "超过 {:.1f} 秒仍没有第一帧有效YOLO推理".format(
                            self.internal_timeout
                        )
                    )

                # ---------------------------------------------
                # 只对新的机器人摄像头帧执行YOLO
                # ---------------------------------------------

                elif (
                    seq
                    !=
                    self.last_processed_seq
                ):

                    self.last_processed_seq = seq

                    try:

                        (
                            annotated,
                            phone,
                            backpack,
                            details
                        ) = self.infer(
                            frame
                        )

                        # 第一帧真正推理成功后才开始1.5秒计时
                        if (
                            self.detection_start_time
                            is None
                        ):

                            self.detection_start_time = (
                                time.monotonic()
                            )

                            rospy.loginfo(
                                "✅ 第一帧YOLO推理成功，"
                                "现在正式开始 %.1f 秒有效检测窗口",
                                self.detect_seconds
                            )

                        display = annotated

                        self.processed_frames += 1

                        if phone:
                            self.phone_hit_frames += 1

                        if backpack:
                            self.backpack_hit_frames += 1

                        self.frame_result_pub.publish(
                            String(
                                data=json.dumps(
                                    {
                                        "phone":
                                            phone,

                                        "backpack":
                                            backpack,

                                        "detections":
                                            details
                                    },
                                    ensure_ascii=False
                                )
                            )
                        )

                    except Exception as e:

                        rospy.logerr_throttle(
                            1.0,
                            "❌ YOLO推理失败: type=%s repr=%r",
                            type(e).__name__,
                            e
                        )

                # ---------------------------------------------
                # 是否结束当前检测窗口
                # ---------------------------------------------

                if (
                    self.active
                    and
                    self.detection_start_time
                    is not None
                ):

                    elapsed = (
                        time.monotonic()
                        -
                        self.detection_start_time
                    )

                    # 必须同时满足：
                    # 1) 有效检测时间 >= 1.5秒
                    # 2) 至少处理 min_hit_frames 张有效图像
                    if (
                        elapsed
                        >=
                        self.detect_seconds
                        and
                        self.processed_frames
                        >=
                        self.min_hit_frames
                    ):

                        self.finish_session()

            # =================================================
            # 画状态
            # =================================================

            display = self.draw_status(
                display
            )

            # ROS debug topic 按需发布
            self.publish_debug(
                display,
                header
            )

            # =================================================
            # 本机实时窗口
            # =================================================

            if self.show_window:

                cv2.imshow(
                    "Task3 Robot Camera",
                    display
                )

                key = (
                    cv2.waitKey(1)
                    &
                    0xFF
                )

                if key == ord("e"):

                    rospy.signal_shutdown(
                        "User pressed E"
                    )

                    break

            rate.sleep()


    # =========================================================
    # 关闭
    # =========================================================

    def shutdown(
        self
    ):

        if self.shutdown_done:
            return

        self.shutdown_done = True

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        rospy.loginfo(
            "🛑 task3_detector 已关闭"
        )


if __name__ == "__main__":

    node = None

    try:

        node = Task3Detector()

        node.run()

    except rospy.ROSInterruptException:

        pass

    finally:

        if node is not None:

            node.shutdown()