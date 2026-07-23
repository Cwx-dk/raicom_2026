#!/usr/bin/env python3
import sys
import rospy
import cv2
import pickle
import os
import numpy as np
import threading
import queue
import time
from ultralytics import YOLO
import insightface
from std_msgs.msg import String
from PIL import Image, ImageDraw, ImageFont

class FaceRecognizerNode:
    def __init__(self):
        rospy.init_node('face_recognizer_node', anonymous=True, disable_signals=True)
        package_path = '/home/cde/bobac4/src/yolo_face'
        model_dir = os.path.join(package_path, 'model')

        # 加载 YOLO 模型
        print("加载 YOLO...", flush=True)
        det_model_path = os.path.join(model_dir, 'face_yolov8n.pt')
        self.detector = YOLO(det_model_path)

        # 加载 InsightFace 识别器
        print("加载 InsightFace...", flush=True)
        self.recognizer = insightface.app.FaceAnalysis(name='buffalo_l')
        #self.recognizer = insightface.app.FaceAnalysis(name='antelope') 
        self.recognizer.prepare(ctx_id=-1)

        # 加载人脸数据库
        print("加载数据库...", flush=True)
        db_path = os.path.join(model_dir, 'face_database.pkl')
        with open(db_path, 'rb') as f:
            self.face_database = pickle.load(f)
        rospy.loginfo(f"已加载 {len(self.face_database)} 个身份")

        # ROS 通信
        self.face_pub = rospy.Publisher("/recognized_face", String, queue_size=10)

        # 摄像头
        print("摄像头初始化...", flush=True)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头！")
            exit(1)

        # ===== 状态管理 =====
        self.last_processed_frame = None
        self.current_name = "Unknown"
        self.current_confidence = 0.0
        self.last_published_name = None
        self.last_publish_time = 0
        self.publish_cooldown = 0.5

        # ===== 识别结果缓存（用于显示） =====
        self.last_result = {
            'name': 'Unknown',
            'confidence': 0.0,
            'bbox': None
        }

        # 加载中文字体
        self.load_chinese_font()
        self.threshold = 0.25
        
        rospy.loginfo("✅ 人脸识别节点初始化完成")
        self.run()

    def load_chinese_font(self):
        font_paths = [
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
        ]
        self.font = None
        for path in font_paths:
            if os.path.exists(path):
                try:
                    self.font = ImageFont.truetype(path, 24)
                    return
                except:
                    continue
        self.font = ImageFont.load_default()

    def match_face(self, embedding):
        best_name = "Unknown"
        best_score = -1
        for name, db_embedding in self.face_database.items():
            score = np.dot(embedding, db_embedding)
            if score > best_score:
                best_score = score
                best_name = name
        if best_score < self.threshold:
            return "Unknown", best_score
        return best_name, best_score

    def draw_chinese_text(self, frame, text, pos, color):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(frame_rgb)
        draw = ImageDraw.Draw(pil_img)
        draw.text(pos, text, font=self.font, fill=color)
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    def process_frame(self, frame):
        """同步处理单帧（稳定可靠）"""
        results = self.detector(frame, conf=0.2, verbose=False)
        detected_name = "Unknown"
        detected_confidence = 0.0

        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    face_img = frame[y1:y2, x1:x2]
                    if face_img.size == 0:
                        continue

                    faces = self.recognizer.get(face_img)
                    if len(faces) > 0:
                        embedding = faces[0].normed_embedding
                        detected_name, detected_confidence = self.match_face(embedding)
                        
                        # 更新最新结果
                        self.last_result['name'] = detected_name
                        self.last_result['confidence'] = detected_confidence
                        self.last_result['bbox'] = (x1, y1, x2, y2)
                        
                        # 绘制
                        color = (0, 255, 0) if detected_name != "Unknown" else (0, 0, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        label = f"{detected_name} ({detected_confidence:.2f})" if detected_name != "Unknown" else "Unknown"
                        frame = self.draw_chinese_text(frame, label, (x1, y1-25), color)
                        break

        # 发布结果
        # if detected_name != "Unknown" and detected_name != self.last_published_name:
        #     current_time = time.time()
        #     if current_time - self.last_publish_time > self.publish_cooldown:
        #         self.face_pub.publish(f"{detected_name}|{detected_confidence:.3f}")
        #         rospy.loginfo(f"🔔 识别到：{detected_name} (置信度: {detected_confidence:.2f})")
        #         self.last_published_name = detected_name
        #         self.last_publish_time = current_time

        # 发布结果
        if detected_name != "Unknown":
            current_time = time.time()
            if current_time - self.last_publish_time > 2.0:
                msg = f"{detected_name}|{detected_confidence:.3f}"
                self.face_pub.publish(msg)
                rospy.loginfo(f"🔔 发布: {msg}")
                self.last_publish_time = current_time

        return frame

    def run(self):
        """主循环"""
        frame_count = 0
        
        # ============================================================
        # ✅ 调整这里控制识别频率
        # ============================================================
        PROCESS_INTERVAL = 2  # 每2帧处理一次

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame_count += 1
            if frame_count % PROCESS_INTERVAL == 0:
                # 处理帧（同步识别，稳定可靠）
                frame = self.process_frame(frame)
                self.last_processed_frame = frame.copy()
            else:
                # 使用上一次处理的结果帧
                if self.last_processed_frame is not None:
                    frame = self.last_processed_frame.copy()

            display_frame = cv2.resize(frame, (640, 480))
            cv2.imshow("Face Recognition", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = FaceRecognizerNode()
    except Exception as e:
        print(f"❌ 初始化失败: {e}")