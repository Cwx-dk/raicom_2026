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
import warnings
warnings.filterwarnings('ignore')

class FaceRecognizerNode:
    def __init__(self):
        rospy.init_node('face_recognizer_node', anonymous=True, disable_signals=True)
        package_path = '/home/reicom2025/bobac3_ws/src/yolo_face'
        model_dir = os.path.join(package_path, 'model')

        # 加载 YOLO 模型
        print("加载 YOLO...", flush=True)
        det_model_path = os.path.join(model_dir, 'face_yolov8n.pt')
        self.detector = YOLO(det_model_path)

        # 加载 InsightFace 识别器
        print("加载 InsightFace...", flush=True)
        self.recognizer = insightface.app.FaceAnalysis(name='buffalo_l')
        # 使用 CPU，检测尺寸减小以提高速度
        self.recognizer.prepare(ctx_id=-1, det_size=(320, 320))
        
        # 加载人脸数据库
        print("加载数据库...", flush=True)
        db_path = os.path.join(model_dir, 'face_database.pkl')
        with open(db_path, 'rb') as f:
            self.face_database = pickle.load(f)
        rospy.loginfo(f"已加载 {len(self.face_database)} 个身份")

        # ROS 通信
        self.face_pub = rospy.Publisher("/recognized_face", String, queue_size=10)

        # 摄像头 - 降低分辨率提高速度
        print("摄像头初始化...", flush=True)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头！")
            exit(1)
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # ===== 状态管理 =====
        self.last_processed_frame = None
        self.current_name = "Unknown"
        self.current_confidence = 0.0
        self.last_published_name = None
        self.last_publish_time = 0
        self.publish_cooldown = 2.0  # 发布间隔（秒）

        # ===== 识别结果缓存（用于显示） =====
        self.last_result = {
            'name': 'Unknown',
            'confidence': 0.0,
            'bbox': None
        }

        # 加载中文字体
        self.load_chinese_font()
        self.threshold = 0.25
        
        # 帧率控制
        self.frame_count = 0
        self.process_interval = 2  # 每2帧处理一次（可调整：1-5）
        
        rospy.loginfo("✅ 人脸识别节点初始化完成")
        self.run()

    def load_chinese_font(self):
        font_paths = [
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
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
        # 缩小图像以提高处理速度
        height, width = frame.shape[:2]
        if width > 640:
            scale = 640 / width
            new_width = 640
            new_height = int(height * scale)
            process_frame = cv2.resize(frame, (new_width, new_height))
        else:
            process_frame = frame.copy()
        
        # YOLO检测 - 使用更小的输入尺寸
        results = self.detector(process_frame, conf=0.2, imgsz=320, verbose=False)
        detected_name = "Unknown"
        detected_confidence = 0.0

        for result in results:
            if result.boxes is not None:
                # 按面积排序，只处理最大的人脸
                boxes = result.boxes
                areas = []
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    areas.append((x2 - x1) * (y2 - y1))
                
                if areas:
                    # 只处理最大的人脸
                    max_idx = np.argmax(areas)
                    box = boxes[max_idx]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    face_img = process_frame[y1:y2, x1:x2]
                    if face_img.size > 0:
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
                            cv2.rectangle(process_frame, (x1, y1), (x2, y2), color, 2)
                            label = f"{detected_name} ({detected_confidence:.2f})" if detected_name != "Unknown" else "Unknown"
                            process_frame = self.draw_chinese_text(process_frame, label, (x1, y1-25), color)

        # 发布结果（带冷却时间）
        if detected_name != "Unknown":
            current_time = time.time()
            if current_time - self.last_publish_time > self.publish_cooldown:
                msg = f"{detected_name}|{detected_confidence:.3f}"
                self.face_pub.publish(msg)
                rospy.loginfo(f"🔔 发布: {msg}")
                self.last_publish_time = current_time

        return process_frame

    def run(self):
        """主循环"""
        frame_count = 0
        fps_counter = 0
        fps_time = time.time()
        current_fps = 0
        
        rospy.loginfo("🔄 开始人脸识别循环...")
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("摄像头读取失败，重试...")
                time.sleep(0.1)
                continue

            frame_count += 1
            fps_counter += 1
            
            # 计算FPS
            if time.time() - fps_time > 1.0:
                current_fps = fps_counter
                fps_counter = 0
                fps_time = time.time()
                # 可选：显示FPS
                # rospy.loginfo(f"FPS: {current_fps}")
            
            # 按间隔处理帧
            if frame_count % self.process_interval == 0:
                # 处理帧
                processed_frame = self.process_frame(frame)
                self.last_processed_frame = processed_frame.copy()
            else:
                # 使用上一次处理的结果帧
                if self.last_processed_frame is not None:
                    processed_frame = self.last_processed_frame.copy()
                else:
                    # 如果没有缓存帧，直接显示原始帧
                    processed_frame = cv2.resize(frame, (640, 480))
                    cv2.putText(processed_frame, "Processing...", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 在画面上显示FPS
            cv2.putText(processed_frame, f"FPS: {current_fps}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 显示当前识别状态
            status_text = f"Status: {self.last_result['name']}"
            cv2.putText(processed_frame, status_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 显示帧处理间隔信息
            interval_text = f"Process every {self.process_interval} frames"
            cv2.putText(processed_frame, interval_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # 显示画面
            display_frame = cv2.resize(processed_frame, (640, 480))
            cv2.imshow("Face Recognition", display_frame)
            
            # 按 'q' 退出，按 '+' 增加处理频率，按 '-' 减少处理频率
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("用户退出")
                break
            elif key == ord('+') or key == ord('='):
                self.process_interval = max(1, self.process_interval - 1)
                rospy.loginfo(f"处理间隔: 每 {self.process_interval} 帧")
            elif key == ord('-'):
                self.process_interval = min(10, self.process_interval + 1)
                rospy.loginfo(f"处理间隔: 每 {self.process_interval} 帧")

        self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("👋 节点已停止")

if __name__ == "__main__":
    try:
        node = FaceRecognizerNode()
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        import traceback
        traceback.print_exc()