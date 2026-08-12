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

        # task_scheduler 控制当前是否允许监听
        self.listen_enabled = threading.Event()

        # 启动时默认关闭
        self.listen_enabled.clear()

        # =====================================================
        # SpeechRecognition
        # =====================================================

        self.recognizer = sr.Recognizer()

        # =====================================================
        # 不再执行 adjust_for_ambient_noise()
        #
        # 之前多次正常校准值大约为 45~50。
        # 因此直接使用 50 作为初始值。
        #
        # 这样可以避免启动阶段频繁打开 Pulse，
        # 降低 PortAudio / ALSA 原生层崩溃概率。
        # =====================================================

        self.recognizer.energy_threshold = float(
            rospy.get_param(
                '~energy_threshold',
                50.0
            )
        )

        # 保留动态阈值调整
        # 在不同比赛现场环境下有一定适应能力
        self.recognizer.dynamic_energy_threshold = True

        self.recognizer.dynamic_energy_adjustment_damping = 0.15
        self.recognizer.dynamic_energy_ratio = 1.5

        # 用户停止讲话约0.6秒后认为一句话结束
        self.recognizer.pause_threshold = 0.6

        # 不把极短的瞬时噪声轻易认为是一句话
        # 但不使用硬性的“0.75秒丢弃”，
        # 防止比赛现场只说“参观”时被误删。
        self.recognizer.phrase_threshold = 0.2

        self.recognizer.non_speaking_duration = 0.3

        # =====================================================
        # 麦克风
        # =====================================================

        self.device_index = rospy.get_param(
            '~device_index',
            6
        )

        try:
            self.device_index = int(
                self.device_index
            )

        except Exception:
            self.device_index = 6

        rospy.loginfo(
            "🎤 使用 Pulse 麦克风 device_index = %d",
            self.device_index
        )

        rospy.loginfo(
            "🎚️ 初始 energy_threshold = %.2f",
            self.recognizer.energy_threshold
        )

        # =====================================================
        # 这里只创建对象。
        #
        # 此处不会真正打开音频流。
        # 真正打开 Pulse 发生在收到 voice ON 以后。
        # =====================================================

        self.microphone = sr.Microphone(
            device_index=self.device_index
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
        # 最后再订阅控制 Topic
        #
        # task_scheduler 使用 latched publisher，
        # 所以即使控制信号之前已经发布，
        # 现在订阅以后仍然能够收到最新状态。
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
    # Google语音识别
    #
    # 网络错误时使用同一段 audio 最多重试3次。
    #
    # 注意：
    # UnknownValueError 表示 Google 已收到语音，
    # 只是没听懂。
    # 这种情况继续听用户重新说，不重复提交同一段。
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

                # Google已经成功处理音频，
                # 只是没有听懂。
                # 直接交回外层继续听新的用户语音。
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

            # =================================================
            # voice OFF：
            #
            # 不碰麦克风，只等待 task_scheduler。
            # =================================================

            if not self.listen_enabled.wait(
                timeout=0.1
            ):

                continue

            # 当前这一轮成功得到的文字
            pending_text = None

            try:

                # =================================================
                # 关键修改：
                #
                # 一个完整“等待用户语音”阶段，
                # Pulse只打开一次。
                #
                # Google听不懂 / 没人讲话 / 网络失败，
                # 都留在这个 with 内继续监听。
                #
                # 只有：
                #
                # 1. 成功识别出一句文字
                # 2. scheduler关闭监听
                #
                # 才离开 with 并释放 Pulse。
                # =================================================

                with self.microphone as source:

                    rospy.loginfo(
                        "🎙️ Pulse麦克风已打开，本轮持续监听"
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

                            # =====================================
                            # timeout只表示：
                            #
                            # 3秒没有开始讲话就返回，
                            # 然后继续下一轮listen。
                            #
                            # 不会退出语音阶段。
                            # =====================================

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


                        # =========================================
                        # 如果录音过程中 scheduler 已关闭监听
                        # =========================================

                        if not self.listen_enabled.is_set():

                            rospy.loginfo(
                                "🔇 当前语音阶段已关闭，停止处理录音"
                            )

                            break


                        # =========================================
                        # 计算音频长度，仅用于调试
                        # =========================================

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


                        # =========================================
                        # 保存最后一次有效捕获的原始音频
                        # =========================================

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


                        # =========================================
                        # Google识别
                        # =========================================

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
                                "🎧 Pulse保持打开，继续等待用户重新讲话..."
                            )

                            continue

                        except sr.RequestError as e:

                            rospy.logerr(
                                "🌐 Google语音识别连续失败: %s",
                                str(e)
                            )

                            rospy.logwarn(
                                "⏳ Pulse保持打开，继续等待新的用户语音"
                            )

                            continue


                        if not text:

                            continue


                        # =========================================
                        # 成功得到一句文字
                        # =========================================

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


                        # =========================================
                        # 非常关键：
                        #
                        # 先保存文字，然后退出麦克风 with。
                        #
                        # 这样在 task_scheduler 播放：
                        #
                        # “好的，请跟我来”
                        #
                        # 或重新播放：
                        #
                        # “你好，需要帮助吗？”
                        #
                        # 之前，Pulse输入设备已经释放。
                        # =========================================

                        pending_text = text

                        self.listen_enabled.clear()

                        break


                # =================================================
                # 执行到这里时已经离开 with，
                # Pulse输入流已经关闭。
                # =================================================

                if pending_text is not None:

                    rospy.loginfo(
                        "🔇 Pulse麦克风已释放"
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

                # Python层面的音频异常可以在这里恢复。
                #
                # 如果底层PortAudio直接SIGSEGV，
                # Python无法捕获，
                # test.launch的respawn仍作为最后一道保护。

                rospy.logerr(
                    "❌ Pulse语音阶段发生异常: %s",
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