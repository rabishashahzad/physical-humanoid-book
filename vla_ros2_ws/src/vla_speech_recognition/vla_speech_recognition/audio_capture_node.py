import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
import struct


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')

        # Audio configuration
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1

        # Create publisher for audio data
        self.audio_publisher = self.create_publisher(
            AudioData,
            'audio_input',
            10
        )

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Timer to periodically capture audio
        self.timer = self.create_timer(0.1, self.capture_audio)

        self.get_logger().info('Audio capture node initialized')

    def capture_audio(self):
        """Capture audio from microphone and publish as AudioData message"""
        try:
            # Read audio data from stream
            data = self.stream.read(self.chunk, exception_on_overflow=False)

            # Create AudioData message
            audio_msg = AudioData()
            audio_msg.data = data

            # Publish audio data
            self.audio_publisher.publish(audio_msg)

        except Exception as e:
            self.get_logger().error(f'Error capturing audio: {str(e)}')

    def destroy_node(self):
        """Clean up audio resources"""
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    audio_node = AudioCaptureNode()

    try:
        rclpy.spin(audio_node)
    except KeyboardInterrupt:
        pass
    finally:
        audio_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()