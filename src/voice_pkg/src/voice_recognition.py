#!/usr/bin/env python3
# voice_recognition.py
# 语音识别节点，将麦克风输入转为文本并发布

import rospy
from std_msgs.msg import String
import speech_recognition as sr
import threading

class VoiceRecognitionNode:
    def __init__(self):
        rospy.init_node('voice_recognition', anonymous=True)
        self.pub = rospy.Publisher('/voice_recognition', String, queue_size=10)
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.running = True
        
        # 调整环境噪音
        with self.microphone as source:
            rospy.loginfo("🎤 正在校准麦克风...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("✅ 麦克风已就绪")
        
        # 启动识别线程
        self.thread = threading.Thread(target=self.recognition_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def recognition_loop(self):
        """持续识别语音"""
        while not rospy.is_shutdown() and self.running:
            try:
                with self.microphone as source:
                    rospy.loginfo("🎤 正在监听...")
                    audio = self.recognizer.listen(source, timeout=10, phrase_time_limit=5)
                
                # 使用Google语音识别（需要联网）
                text = self.recognizer.recognize_google(audio, language='zh-CN')
                rospy.loginfo(f"🎤 识别结果: {text}")
                
                # 发布识别结果
                self.pub.publish(String(text))
                
            except sr.WaitTimeoutError:
                # 超时继续监听
                continue
            except sr.UnknownValueError:
                # 无法识别
                continue
            except sr.RequestError as e:
                rospy.logwarn(f"⚠️ 语音识别服务错误: {e}")
                rospy.sleep(1)
            except Exception as e:
                rospy.logwarn(f"⚠️ 识别异常: {e}")
                rospy.sleep(0.5)
    
    def shutdown(self):
        """关闭节点"""
        self.running = False
        rospy.loginfo("🛑 语音识别节点已关闭")

if __name__ == '__main__':
    try:
        node = VoiceRecognitionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()