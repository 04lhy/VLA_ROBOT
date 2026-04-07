#!/usr/bin/env python3
import queue
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('topic_name', '/robot/tts_text')
        self.declare_parameter('voice_rate', 180)
        self.declare_parameter('voice_volume', 1.0)

        self.topic_name = self.get_parameter('topic_name').value
        self.voice_rate = int(self.get_parameter('voice_rate').value)
        self.voice_volume = float(self.get_parameter('voice_volume').value)

        self._tts_queue = queue.Queue()
        self._running = True
        self._engine = None
        self._espeak_bin = shutil.which('espeak-ng') or shutil.which('espeak')

        self._init_engine()

        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.text_callback,
            10,
        )

        self._worker = threading.Thread(target=self._tts_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(f'TTS node started, listening on {self.topic_name}')

    def _init_engine(self):
        try:
            import pyttsx3
            self._engine = pyttsx3.init()
            self._engine.setProperty('rate', self.voice_rate)
            self._engine.setProperty('volume', self.voice_volume)
        except Exception as exc:
            if self._espeak_bin:
                self.get_logger().warn(
                    f'pyttsx3 unavailable ({exc}), fallback to {self._espeak_bin}.'
                )
            else:
                self.get_logger().error(
                    f'Failed to initialize pyttsx3: {exc}. Install pyttsx3 or espeak-ng.'
                )

    def text_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().warn('Received empty TTS text, skipped.')
            return

        self._tts_queue.put(text)
        self.get_logger().info(f'Queued TTS text: {text}')

    def _tts_worker(self):
        while self._running:
            try:
                text = self._tts_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if self._engine is None:
                if not self._espeak_bin:
                    self.get_logger().error('No TTS engine available, drop message.')
                    continue

                try:
                    subprocess.run([self._espeak_bin, text], check=False)
                except Exception as exc:
                    self.get_logger().error(f'espeak playback failed: {exc}')
                continue

            try:
                self._engine.say(text)
                self._engine.runAndWait()
            except Exception as exc:
                self.get_logger().error(f'TTS playback failed: {exc}')

    def stop(self):
        self._running = False
        self._worker.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
