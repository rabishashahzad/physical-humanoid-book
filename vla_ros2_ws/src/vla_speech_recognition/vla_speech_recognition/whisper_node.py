import rclpy
from rclpy.node import Node
import whisper
import torch
import numpy as np
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
import io
import wave
from builtin_interfaces.msg import Time


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model (using small model for faster processing)
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("small")
        self.get_logger().info('Whisper model loaded successfully!')

        # Create subscription to audio data
        self.subscription = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Create publisher for recognized text
        self.text_publisher = self.create_publisher(
            String,
            'recognized_text',
            10
        )

        self.get_logger().info('Whisper node initialized')

    def audio_callback(self, msg):
        """Process incoming audio data and perform speech recognition"""
        try:
            # Convert audio data to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16)

            # Normalize audio data
            audio_array = audio_data.astype(np.float32) / 32768.0

            # Convert to tensor and run through Whisper
            audio_tensor = torch.from_numpy(audio_array)

            # Perform transcription
            result = self.model.transcribe(audio_tensor.numpy())
            recognized_text = result['text'].strip()

            if recognized_text:  # Only publish if there's actual text
                # Create and publish the recognized text
                text_msg = String()
                text_msg.data = recognized_text
                self.text_publisher.publish(text_msg)

                self.get_logger().info(f'Recognized: "{recognized_text}"')
            else:
                self.get_logger().debug('No speech recognized')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    whisper_node = WhisperNode()

    try:
        rclpy.spin(whisper_node)
    except KeyboardInterrupt:
        pass
    finally:
        whisper_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()