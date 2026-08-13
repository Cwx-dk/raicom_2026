#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
import speech_recognition as sr
import threading


class VoiceRecognitionNode:

    def __init__(self):

        rospy.init_node(
            'voice_recognition',
            anonymous=False
        )

        # =====================================================
        # ROS Publisher
        # =====================================================

        self.pub = rospy.Publisher(
            '/voice_recognition',
            String,
            queue_size=10
        )

        # =====================================================
        # 基本运行状态
        # =====================================================

        self.running = True

        # task_scheduler 控制是否允许监听
        self.listen_enabled = threading.Event()

        # 启动时默认关闭
        self.listen_enabled.clear()

        # =====================================================
        # SpeechRecognition
        # =====================================================

        self.recognizer = sr.Recognizer()

        # 不在启动阶段反复校准麦克风，减少 ALSA / PortAudio 问题
        self.recognizer.energy_threshold = float(
            rospy.get_param(
                '~energy_threshold',
                50.0
            )
        )

        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.dynamic_energy_adjustment_damping = 0.15
        self.recognizer.dynamic_energy_ratio = 1.5

        self.recognizer.pause_threshold = 0.6
        self.recognizer.phrase_threshold = 0.2
        self.recognizer.non_speaking_duration = 0.3

        rospy.loginfo(
            "🎚️ 初始 energy_threshold = %.2f",
            self.recognizer.energy_threshold
        )

        # =====================================================
        # 麦克风选择
        #
        # -1：
        #     使用当前电脑的系统默认输入设备
        #
        # >=0：
        #     使用指定 PyAudio device_index
        #
        # 非常重要：
        # 不能把 -1 直接传给 sr.Microphone(device_index=-1)
        # =====================================================

        self.device_index = rospy.get_param(
            '~device_index',
            -1
        )

        try:
            self.device_index = int(
                self.device_index
            )
        except Exception:
            self.device_index = -1

        if self.device_index < 0:

            rospy.loginfo(
                "🎤 麦克风模式：使用系统默认输入设备"
            )

            # 不传 device_index
            # PyAudio 会使用当前电脑真正的默认输入设备
            self.microphone = sr.Microphone()

        else:

            rospy.loginfo(
                "🎤 麦克风模式：使用指定 device_index = %d",
                self.device_index
            )

            self.microphone = sr.Microphone(
                device_index=self.device_index
            )

        rospy.loginfo(
            "🎤 麦克风实际采样率 = %s Hz",
            str(self.microphone.SAMPLE_RATE)
        )

        # =====================================================
        # 启动识别线程
        # =====================================================

        self.thread = threading.Thread(
            target=self.recognition_loop,
            daemon=True
        )

        self.thread.start()

        # =====================================================
        # 最后订阅控制 Topic
        #
        # scheduler 使用 latched publisher，
        # 后启动也可以收到最新状态。
        # =====================================================

        self.listen_control_sub = rospy.Subscriber(
            '/voice_listen_enable',
            Bool,
            self.listen_control_callback,
            queue_size=1
        )

        rospy.on_shutdown(
            self.shutdown
        )

        rospy.loginfo(
            "===================================="
        )

        rospy.loginfo(
            "✅ voice_recognition 节点初始化完成"
        )

        rospy.loginfo(
            "🔇 当前语音监听关闭，等待任务调度器控制"
        )

        rospy.loginfo(
            "===================================="
        )


    # =========================================================
    # task_scheduler 控制语音阶段
    # =========================================================

    def listen_control_callback(self, msg):

        if msg.data:

            if not self.listen_enabled.is_set():

                rospy.loginfo(
                    "===================================="
                )

                rospy.loginfo(
                    "🎧 收到控制命令：开启语音监听"
                )

                rospy.loginfo(
                    "===================================="
                )

            self.listen_enabled.set()

        else:

            if self.listen_enabled.is_set():

                rospy.loginfo(
                    "🔇 收到控制命令：关闭语音监听"
                )

            self.listen_enabled.clear()


    # =========================================================
    # Google 中文语音识别
    #
    # 网络失败时：
    # 对同一段录音最多重试3次。
    # =========================================================

    def recognize_google_with_retry(
        self,
        audio,
        max_attempts=3
    ):

        last_error = None

        for attempt in range(
            1,
            max_attempts + 1
        ):

            try:

                rospy.loginfo(
                    "🌐 Google中文语音识别，第 %d/%d 次尝试...",
                    attempt,
                    max_attempts
                )

                text = (
                    self.recognizer
                    .recognize_google(
                        audio,
                        language='zh-CN'
                    )
                    .strip()
                )

                return text

            except sr.UnknownValueError:

                # 网络正常，但是 Google 没听懂
                raise

            except sr.RequestError as e:

                last_error = e

                rospy.logwarn(
                    "⚠️ 第 %d/%d 次网络识别失败: %s",
                    attempt,
                    max_attempts,
                    str(e)
                )

                if attempt < max_attempts:

                    rospy.loginfo(
                        "🔄 0.5秒后使用同一段录音重新识别..."
                    )

                    rospy.sleep(
                        0.5
                    )

        if last_error is not None:
            raise last_error

        return None


    # =========================================================
    # 主识别循环
    # =========================================================

    def recognition_loop(self):

        while (
            not rospy.is_shutdown()
            and self.running
        ):

            # voice OFF 时完全不碰麦克风
            if not self.listen_enabled.wait(
                timeout=0.1
            ):
                continue

            pending_text = None

            try:

                # =================================================
                # 一整个语音阶段只打开一次麦克风
                # =================================================

                with self.microphone as source:

                    rospy.loginfo(
                        "🎙️ 麦克风已打开，本轮持续监听"
                    )

                    while (
                        self.listen_enabled.is_set()
                        and
                        not rospy.is_shutdown()
                        and
                        self.running
                    ):

                        try:

                            rospy.loginfo(
                                "🎧 正在等待用户讲话..."
                            )

                            audio = self.recognizer.listen(
                                source,
                                timeout=3,
                                phrase_time_limit=5
                            )

                        except sr.WaitTimeoutError:

                            rospy.loginfo_throttle(
                                10.0,
                                "⏳ 暂未检测到用户讲话，继续等待语音..."
                            )

                            continue

                        # scheduler 已经关闭监听
                        if not self.listen_enabled.is_set():

                            rospy.loginfo(
                                "🔇 当前语音阶段已关闭，停止处理录音"
                            )

                            break

                        # =================================================
                        # 计算捕获到的语音长度
                        # =================================================

                        try:

                            duration = (
                                len(audio.frame_data)
                                /
                                float(
                                    audio.sample_rate
                                    *
                                    audio.sample_width
                                )
                            )

                        except Exception:
                            duration = 0.0

                        rospy.loginfo(
                            "✅ 已捕获到用户语音，长度约 %.2f 秒",
                            duration
                        )

                        # =================================================
                        # 保存调试 WAV
                        # =================================================

                        try:

                            with open(
                                "/tmp/voice_debug.wav",
                                "wb"
                            ) as f:

                                f.write(
                                    audio.get_wav_data()
                                )

                            rospy.loginfo(
                                "💾 本次录音已保存到: "
                                "/tmp/voice_debug.wav"
                            )

                        except Exception as e:

                            rospy.logwarn(
                                "⚠️ 保存调试录音失败: %s",
                                str(e)
                            )

                        # =================================================
                        # Google识别
                        # =================================================

                        try:

                            text = (
                                self.recognize_google_with_retry(
                                    audio,
                                    max_attempts=3
                                )
                            )

                        except sr.UnknownValueError:

                            rospy.logwarn(
                                "⚠️ 已录到声音，但Google无法识别文字"
                            )

                            rospy.loginfo(
                                "🎧 麦克风保持打开，继续等待用户重新讲话..."
                            )

                            continue

                        except sr.RequestError as e:

                            rospy.logerr(
                                "🌐 Google语音识别连续失败: %s",
                                str(e)
                            )

                            rospy.logwarn(
                                "⏳ 麦克风保持打开，继续等待新的用户语音"
                            )

                            continue

                        if not text:
                            continue

                        # =================================================
                        # 成功得到文字
                        # =================================================

                        rospy.loginfo(
                            "===================================="
                        )

                        rospy.loginfo(
                            "📝 用户说的是: [%s]",
                            text
                        )

                        rospy.loginfo(
                            "===================================="
                        )

                        # 先记录文字，再退出 with 释放麦克风
                        pending_text = text

                        self.listen_enabled.clear()

                        break

                # =================================================
                # 离开 with 后，麦克风已经释放
                # =================================================

                if pending_text is not None:

                    rospy.loginfo(
                        "🔇 麦克风已释放"
                    )

                    msg = String()
                    msg.data = pending_text

                    self.pub.publish(
                        msg
                    )

                    rospy.loginfo(
                        "📤 已发布到 /voice_recognition: [%s]",
                        pending_text
                    )

                    rospy.loginfo(
                        "🔇 本轮语音文字已提交，等待调度器决定下一步"
                    )

            except Exception as e:

                rospy.logerr(
                    "❌ 语音阶段发生异常: %s",
                    str(e)
                )

                rospy.logwarn(
                    "🔄 0.5秒后重新尝试当前语音阶段"
                )

                rospy.sleep(
                    0.5
                )


    # =========================================================
    # 关闭节点
    # =========================================================

    def shutdown(self):

        self.running = False
        self.listen_enabled.clear()

        rospy.loginfo(
            "🛑 voice_recognition 节点关闭"
        )


if __name__ == '__main__':

    node = None

    try:

        node = VoiceRecognitionNode()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:

        if node is not None:
            node.shutdown()
