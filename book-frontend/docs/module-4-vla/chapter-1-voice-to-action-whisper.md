---
title: "Chapter 1 - Voice-to-Action (Whisper Integration)"
sidebar_position: 2
---

# Chapter 1: Voice-to-Action (Whisper Integration)

In this chapter, we'll explore how to integrate OpenAI Whisper for speech recognition with ROS 2 systems. You'll learn to build a voice command processing pipeline that can understand natural language and convert it to actionable commands for your humanoid robot.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up and configure OpenAI Whisper for real-time speech recognition
- Integrate Whisper with ROS 2 nodes and message passing
- Process and validate voice commands in noisy environments
- Implement error handling and confidence scoring for recognition results

## Introduction to Whisper for Robotics

OpenAI Whisper is a state-of-the-art speech recognition model that can convert spoken language to text with remarkable accuracy. For robotics applications, Whisper provides several advantages:

- **Robustness**: Performs well in various acoustic environments
- **Multilingual Support**: Can recognize multiple languages
- **Real-time Processing**: Capable of processing streaming audio
- **Open Source**: Free to use and modify for research applications

## Setting Up Whisper with ROS 2

First, let's create a ROS 2 node that integrates Whisper for speech recognition:

```python
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
```

## Audio Capture and Preprocessing

For real-time applications, we need to capture audio from a microphone and preprocess it for Whisper:

```python
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
```

## Voice Command Processing

Once we have the recognized text, we need to process it into actionable commands:

```python
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        # Create subscription to raw text commands
        self.raw_command_subscription = self.create_subscription(
            String,
            'recognized_text',
            self.raw_command_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Create publisher for parsed commands
        self.parsed_command_publisher = self.create_publisher(
            String,
            'parsed_commands',
            10
        )

        # Define command vocabulary and patterns
        self.command_patterns = {
            'move': ['go to', 'move to', 'navigate to', 'walk to', 'travel to'],
            'pick': ['pick up', 'grab', 'take', 'collect', 'get'],
            'place': ['place', 'put', 'drop', 'set down'],
            'follow': ['follow', 'come with me', 'accompany'],
            'stop': ['stop', 'halt', 'pause', 'wait'],
            'greet': ['hello', 'hi', 'greet', 'say hello', 'wave']
        }

        self.get_logger().info('Command Parser node initialized')

    def raw_command_callback(self, msg):
        """Parse raw commands and convert to structured format"""
        try:
            raw_command = msg.data.lower().strip()
            if not raw_command:
                return

            # Parse the command
            parsed_command = self.parse_command(raw_command)

            if parsed_command:
                # Publish parsed command
                parsed_msg = String()
                parsed_msg.data = json.dumps(parsed_command)
                self.parsed_command_publisher.publish(parsed_msg)

                self.get_logger().info(f'Parsed command: {parsed_command}')
        except Exception as e:
            self.get_logger().error(f'Error parsing command: {str(e)}')

    def parse_command(self, command):
        """Parse a natural language command into structured format"""
        command_lower = command.lower()

        # Identify command type
        command_type = None
        for cmd_type, patterns in self.command_patterns.items():
            if any(pattern in command_lower for pattern in patterns):
                command_type = cmd_type
                break

        if not command_type:
            return None

        # Extract additional information
        result = {
            'command_type': command_type,
            'original_command': command,
            'confidence': 0.9  # High confidence for simple pattern matching
        }

        # Add specific parameters based on command type
        if command_type == 'move':
            # Extract destination if mentioned
            if 'kitchen' in command_lower:
                result['destination'] = 'kitchen'
                result['coordinates'] = {'x': 1.0, 'y': 2.0, 'z': 0.0}
            elif 'living room' in command_lower:
                result['destination'] = 'living_room'
                result['coordinates'] = {'x': 3.0, 'y': 1.0, 'z': 0.0}
            elif 'bedroom' in command_lower:
                result['destination'] = 'bedroom'
                result['coordinates'] = {'x': 0.0, 'y': -1.0, 'z': 0.0}
            else:
                result['destination'] = 'unknown'

        elif command_type in ['pick', 'place']:
            # Extract object if mentioned
            if 'cup' in command_lower:
                result['object'] = 'cup'
            elif 'book' in command_lower:
                result['object'] = 'book'
            elif 'ball' in command_lower:
                result['object'] = 'ball'
            else:
                result['object'] = 'unknown'

        return result
```

## Audio Quality Optimization

For robust performance in various environments, implement audio preprocessing:

```python
import numpy as np
from scipy import signal


class AudioPreprocessor:
    def __init__(self):
        # Define preprocessing parameters
        self.sample_rate = 16000
        self.noise_threshold = 0.01  # Minimum amplitude to consider as speech
        self.preemphasis_coeff = 0.97  # For pre-emphasis filtering

    def preprocess_audio(self, audio_data):
        """Apply preprocessing to improve speech recognition quality"""
        # Apply pre-emphasis filter to boost high frequencies
        preemphasized = self.preemphasis_filter(audio_data)

        # Normalize audio
        normalized = self.normalize_audio(preemphasized)

        # Apply noise reduction if needed
        denoised = self.reduce_noise(normalized)

        return denoised

    def preemphasis_filter(self, audio_data):
        """Apply pre-emphasis filter to boost high frequencies"""
        return np.append(audio_data[0], audio_data[1:] - self.preemphasis_coeff * audio_data[:-1])

    def normalize_audio(self, audio_data):
        """Normalize audio to a consistent level"""
        max_amplitude = np.max(np.abs(audio_data))
        if max_amplitude > 0:
            return audio_data / max_amplitude
        return audio_data

    def reduce_noise(self, audio_data):
        """Simple noise reduction by applying a threshold"""
        # Apply a simple noise gate
        mask = np.abs(audio_data) > self.noise_threshold
        return audio_data * mask.astype(float)
```

## Launch Configuration

Create a launch file to start all nodes together:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Whisper speech recognition node
        Node(
            package='vla_speech_recognition',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
            parameters=[]
        ),

        # Audio capture node (simulated)
        Node(
            package='vla_speech_recognition',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen',
            parameters=[]
        ),

        # Command parser node
        Node(
            package='vla_speech_recognition',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen',
            parameters=[]
        )
    ])
```

## Exercises

1. **Setup Exercise**: Install OpenAI Whisper and integrate it with your ROS 2 workspace
2. **Enhancement Exercise**: Add support for multiple languages in your speech recognition system
3. **Optimization Exercise**: Implement noise reduction algorithms to improve recognition in noisy environments
4. **Testing Exercise**: Test your system with various audio inputs and measure recognition accuracy

## Summary

In this chapter, you've learned to integrate OpenAI Whisper with ROS 2 systems for speech recognition. You've created nodes for audio capture, speech recognition, and command parsing. This forms the foundation for voice-controlled robotic systems.

In the next chapter, we'll explore how to use large language models for cognitive planning and task decomposition.