#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人脸识别节点：保持“识别算法启停”和“摄像头开关”完全分离。

最终行为：
1. 节点启动时打开人脸摄像头，并在整个节点生命周期内保持打开。
2. /face_detection_enable=true：
   执行 YOLO + InsightFace 人脸识别。
3. 检测到人脸后：
   发布 /face_detected=true，并停止继续进行人脸识别算法。
4. /face_detection_enable=false：
   只停止人脸识别算法；摄像头仍持续读取并显示实时画面。
5. 只有整个节点真正退出时才 release 人脸摄像头。
6. 任务二使用另一台独立现实摄像头，本节点完全不管理任务二摄像头。
"""

import os
import time
import pickle
import warnings

import cv2
import numpy as np
import rospy
from PIL import Image, ImageDraw, ImageFont
from std_msgs.msg import String, Bool
from ultralytics import YOLO
import insightface

warnings.filterwarnings("ignore")


class FaceRecognizerNode:

    def __init__(self):

        rospy.init_node(
            "face_recognizer_node",
            anonymous=False,
            disable_signals=True
        )

        self.package_path = (
            "/home/reicom2025/bobac3_ws/src/yolo_face"
        )

        model_dir = os.path.join(
            self.package_path,
            "model"
        )

        # =====================================================
        # 参数
        # =====================================================

        self.camera_index = int(
            rospy.get_param(
                "~camera_index",
                0
            )
        )

        # =====================================================
        # 模型
        # =====================================================

        rospy.loginfo(
            "👤 加载人脸YOLO..."
        )

        self.detector = YOLO(
            os.path.join(
                model_dir,
                "face_yolov8n.pt"
            )
        )

        rospy.loginfo(
            "👤 加载InsightFace..."
        )

        self.recognizer = (
            insightface
            .app
            .FaceAnalysis(
                name="buffalo_l"
            )
        )

        self.recognizer.prepare(
            ctx_id=-1,
            det_size=(320, 320)
        )

        db_path = os.path.join(
            model_dir,
            "face_database.pkl"
        )

        with open(
            db_path,
            "rb"
        ) as f:

            self.face_database = (
                pickle.load(f)
            )

        rospy.loginfo(
            "✅ 已加载 %d 个人脸身份",
            len(
                self.face_database
            )
        )

        # =====================================================
        # ROS
        # =====================================================

        self.face_pub = rospy.Publisher(
            "/recognized_face",
            String,
            queue_size=10
        )

        self.face_detected_pub = rospy.Publisher(
            "/face_detected",
            Bool,
            queue_size=10
        )

        self.face_detection_enabled = False

        self.control_sub = rospy.Subscriber(
            "/face_detection_enable",
            Bool,
            self.face_detection_control_callback,
            queue_size=1
        )

        # =====================================================
        # 人脸现实摄像头
        #
        # 关键：
        # 节点启动时打开一次。
        # 之后不随 /face_detection_enable 开关而关闭。
        # =====================================================

        self.cap = None
        self.last_camera_open_try = 0.0

        if not self.open_camera():
            raise RuntimeError(
                "无法打开人脸识别现实摄像头 index={}".format(
                    self.camera_index
                )
            )

        # =====================================================
        # 状态
        # =====================================================

        self.last_processed_frame = None

        self.last_result = {
            "name": "Unknown",
            "confidence": 0.0,
            "bbox": None
        }

        self.last_publish_time = 0.0
        self.publish_cooldown = 2.0

        self.threshold = 0.25

        self.frame_count = 0
        self.process_interval = 2

        self.load_chinese_font()

        rospy.on_shutdown(
            self.shutdown
        )

        rospy.loginfo(
            "✅ face_node 已启动；人脸摄像头保持实时画面，识别算法默认关闭"
        )


    # =========================================================
    # 摄像头
    # =========================================================

    def open_camera(
        self
    ):

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
            self.last_camera_open_try
            <
            0.5
        ):
            return False

        self.last_camera_open_try = now

        rospy.loginfo(
            "📷 人脸节点打开现实摄像头 index=%d",
            self.camera_index
        )

        cap = cv2.VideoCapture(
            self.camera_index,
            cv2.CAP_V4L2
        )

        if not cap.isOpened():

            cap.release()

            cap = cv2.VideoCapture(
                self.camera_index
            )

        if not cap.isOpened():

            cap.release()

            rospy.logwarn_throttle(
                2.0,
                "⚠️ 人脸节点暂时无法打开现实摄像头"
            )

            return False

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            640
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            480
        )

        cap.set(
            cv2.CAP_PROP_FPS,
            30
        )

        self.cap = cap

        rospy.loginfo(
            "✅ 人脸节点获得现实摄像头"
        )

        return True


    def close_camera(
        self
    ):

        if self.cap is not None:

            try:
                self.cap.release()
            except Exception:
                pass

            self.cap = None

            rospy.loginfo(
                "🔓 人脸节点已释放现实摄像头"
            )

        try:
            cv2.destroyWindow(
                "Face Recognition"
            )
        except Exception:
            pass


    # =========================================================
    # 控制
    # =========================================================

    def face_detection_control_callback(
        self,
        msg
    ):

        self.face_detection_enabled = (
            msg.data
        )

        if msg.data:

            self.last_publish_time = 0.0

            rospy.loginfo(
                "👤 收到命令：开启人脸检测"
            )

        else:

            # 这里只暂停识别算法。
            # 不 release，不关闭窗口。
            self.last_processed_frame = None

            rospy.loginfo(
                "⏸️ 收到命令：暂停人脸识别；摄像头画面继续实时显示"
            )


    # =========================================================
    # 字体
    # =========================================================

    def load_chinese_font(
        self
    ):

        font_paths = [
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]

        self.font = None

        for path in font_paths:

            if not os.path.exists(
                path
            ):
                continue

            try:

                self.font = (
                    ImageFont
                    .truetype(
                        path,
                        24
                    )
                )

                return

            except Exception:

                pass

        self.font = (
            ImageFont
            .load_default()
        )


    # =========================================================
    # 人脸匹配
    # =========================================================

    def match_face(
        self,
        embedding
    ):

        best_name = "Unknown"
        best_score = -1.0

        for (
            name,
            db_embedding
        ) in self.face_database.items():

            score = np.dot(
                embedding,
                db_embedding
            )

            if score > best_score:

                best_score = score
                best_name = name

        if best_score < self.threshold:

            return (
                "Unknown",
                best_score
            )

        return (
            best_name,
            best_score
        )


    def draw_chinese_text(
        self,
        frame,
        text,
        pos,
        color
    ):

        frame_rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        pil_img = Image.fromarray(
            frame_rgb
        )

        draw = ImageDraw.Draw(
            pil_img
        )

        draw.text(
            pos,
            text,
            font=self.font,
            fill=color
        )

        return cv2.cvtColor(
            np.array(
                pil_img
            ),
            cv2.COLOR_RGB2BGR
        )


    # =========================================================
    # 单帧人脸
    # =========================================================

    def process_frame(
        self,
        frame
    ):

        if not self.face_detection_enabled:

            return frame.copy()

        height, width = (
            frame.shape[:2]
        )

        if width > 640:

            scale = (
                640.0
                /
                width
            )

            process_frame = cv2.resize(
                frame,
                (
                    640,
                    int(
                        height
                        *
                        scale
                    )
                )
            )

        else:

            process_frame = frame.copy()


        results = self.detector(
            process_frame,
            conf=0.2,
            imgsz=320,
            verbose=False
        )


        detected_name = "Unknown"
        detected_confidence = 0.0
        face_found = False


        for result in results:

            if (
                result.boxes
                is
                None
            ):
                continue

            boxes = result.boxes

            areas = []

            for box in boxes:

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[
                        0
                    ].tolist()
                )

                areas.append(
                    (
                        x2 - x1
                    )
                    *
                    (
                        y2 - y1
                    )
                )


            if not areas:
                continue


            face_found = True

            max_idx = int(
                np.argmax(
                    areas
                )
            )

            box = boxes[
                max_idx
            ]

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[
                    0
                ].tolist()
            )


            face_img = process_frame[
                y1:y2,
                x1:x2
            ]


            if face_img.size <= 0:
                continue


            faces = self.recognizer.get(
                face_img
            )


            if len(faces) <= 0:
                continue


            embedding = (
                faces[
                    0
                ]
                .normed_embedding
            )


            (
                detected_name,
                detected_confidence
            ) = self.match_face(
                embedding
            )


            self.last_result = {
                "name":
                    detected_name,

                "confidence":
                    detected_confidence,

                "bbox":
                    (
                        x1,
                        y1,
                        x2,
                        y2
                    )
            }


            color = (
                (0, 255, 0)
                if
                detected_name
                !=
                "Unknown"
                else
                (0, 0, 255)
            )


            cv2.rectangle(
                process_frame,
                (x1, y1),
                (x2, y2),
                color,
                2
            )


            label = (
                "{} ({:.2f})".format(
                    detected_name,
                    detected_confidence
                )
                if
                detected_name
                !=
                "Unknown"
                else
                "Unknown"
            )


            process_frame = (
                self.draw_chinese_text(
                    process_frame,
                    label,
                    (
                        x1,
                        max(
                            0,
                            y1 - 25
                        )
                    ),
                    color
                )
            )


        current_time = (
            time.time()
        )


        if (
            face_found
            and
            self.face_detection_enabled
            and
            current_time
            -
            self.last_publish_time
            >
            self.publish_cooldown
        ):

            # 一轮只触发一次
            self.face_detection_enabled = False


            self.face_detected_pub.publish(
                Bool(
                    data=True
                )
            )


            rospy.loginfo(
                "👤 检测到人脸，发布 /face_detected=true"
            )


            if (
                detected_name
                !=
                "Unknown"
            ):

                msg = "{}|{:.3f}".format(
                    detected_name,
                    detected_confidence
                )

                self.face_pub.publish(
                    String(
                        data=msg
                    )
                )


            self.last_publish_time = (
                current_time
            )


        return process_frame


    # =========================================================
    # 主循环
    # =========================================================

    def run(
        self
    ):

        fps_counter = 0
        fps_time = time.time()
        current_fps = 0

        rospy.loginfo(
            "🔄 开始人脸摄像头实时循环..."
        )

        while not rospy.is_shutdown():

            # -------------------------------------------------
            # 摄像头始终保持打开、始终读帧。
            # /face_detection_enable 只决定是否运行识别算法。
            # -------------------------------------------------

            if (
                self.cap is None
                or
                not self.cap.isOpened()
            ):

                # 仅用于设备异常恢复；正常任务切换不会走这里。
                if not self.open_camera():

                    rospy.logwarn_throttle(
                        2.0,
                        "⚠️ 人脸摄像头当前不可用，继续等待..."
                    )

                    time.sleep(
                        0.1
                    )

                    continue


            ret, frame = (
                self.cap.read()
            )


            if not ret:

                rospy.logwarn_throttle(
                    2.0,
                    "⚠️ 人脸摄像头读取失败，摄像头不关闭，继续重试..."
                )

                time.sleep(
                    0.1
                )

                continue


            self.frame_count += 1
            fps_counter += 1


            if (
                time.time()
                -
                fps_time
                >
                1.0
            ):

                current_fps = (
                    fps_counter
                )

                fps_counter = 0
                fps_time = time.time()


            # -------------------------------------------------
            # 开启状态：跑人脸识别
            # 关闭状态：不跑算法，只显示原始实时画面
            # -------------------------------------------------

            if self.face_detection_enabled:

                if (
                    self.frame_count
                    %
                    self.process_interval
                    ==
                    0
                ):

                    processed_frame = (
                        self.process_frame(
                            frame
                        )
                    )

                    self.last_processed_frame = (
                        processed_frame.copy()
                    )

                else:

                    if (
                        self.last_processed_frame
                        is not None
                    ):

                        processed_frame = (
                            self.last_processed_frame.copy()
                        )

                    else:

                        processed_frame = (
                            frame.copy()
                        )

            else:

                processed_frame = (
                    frame.copy()
                )

                # 防止暂停后仍显示上一轮人脸框
                self.last_processed_frame = None


            cv2.putText(
                processed_frame,
                "FPS: {}".format(
                    current_fps
                ),
                (
                    10,
                    30
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (
                    0,
                    255,
                    255
                ),
                2
            )


            if self.face_detection_enabled:

                mode_text = (
                    "Face Detection: ON"
                )

                mode_color = (
                    0,
                    255,
                    0
                )

            else:

                mode_text = (
                    "Face Detection: OFF - Camera Live"
                )

                mode_color = (
                    0,
                    255,
                    255
                )


            cv2.putText(
                processed_frame,
                mode_text,
                (
                    10,
                    60
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                mode_color,
                2
            )


            cv2.imshow(
                "Face Recognition",
                cv2.resize(
                    processed_frame,
                    (
                        640,
                        480
                    )
                )
            )


            key = (
                cv2.waitKey(
                    1
                )
                &
                0xFF
            )


            if key == ord("q"):

                rospy.signal_shutdown(
                    "User pressed q"
                )

                break


        self.shutdown()


    def shutdown(
        self
    ):

        self.face_detection_enabled = False

        self.close_camera()

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        rospy.loginfo(
            "🛑 face_node 已关闭"
        )


if __name__ == "__main__":

    try:

        node = FaceRecognizerNode()

        node.run()

    except Exception as e:

        rospy.logerr(
            "❌ face_node异常: %r",
            e
        )

        raise
