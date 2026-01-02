---
sidebar_position: 17
title: "Chapter 17: Voice to Text with Whisper"
---

# Chapter 17: Voice to Text with Whisper

## Introduction
OpenAI's Whisper model has revolutionized automatic speech recognition (ASR) by providing highly accurate, robust speech-to-text capabilities. This chapter explores the integration of Whisper with robotics systems for voice-controlled robot interaction.

## Learning Objectives
- Understand the Whisper model architecture and capabilities
- Learn about speech recognition in robotics applications
- Explore real-time speech processing for robotics
- Implement Whisper integration with robot control systems
- Recognize the benefits of voice interfaces for robotics

## Core Concepts
### Whisper Model Architecture
Advanced speech recognition model:
- **Transformer Architecture**: Attention-based neural network
- **Multilingual Support**: Trained on 98+ languages
- **Robustness**: Handles various accents, background noise, and speaking styles
- **Large-Scale Training**: Trained on 680,000 hours of multilingual data
- **Zero-Shot Capabilities**: Works without task-specific fine-tuning

### Speech Recognition in Robotics
Voice interfaces for robot control:
- **Natural Interaction**: Intuitive human-robot communication
- **Hands-Free Operation**: Useful when hands are occupied
- **Accessibility**: Enabling interaction for users with mobility limitations
- **Multilingual Support**: Serving diverse user populations
- **Context Awareness**: Understanding commands in context

### Real-Time Processing
Challenges in real-time speech recognition:
- **Latency Requirements**: Low-latency processing for natural interaction
- **Streaming Audio**: Processing continuous audio streams
- **VAD Integration**: Voice Activity Detection to identify speech segments
- **Buffer Management**: Efficient handling of audio data
- **Resource Constraints**: Running on embedded robotics hardware

### Robotics Integration
Connecting speech recognition with robot systems:
- **ROS Integration**: Publishing recognized text as ROS messages
- **Intent Recognition**: Converting text to robot commands
- **Error Handling**: Managing recognition errors and ambiguities
- **Feedback Systems**: Providing audio/visual feedback to users
- **Privacy Considerations**: Handling sensitive voice data

## Practical Examples
### Example 1: Basic Whisper Integration
```python
import whisper
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np

class WhisperSpeechToText(Node):
    def __init__(self):
        super().__init__('whisper_speech_to_text')

        # Load Whisper model
        self.model = whisper.load_model("base")

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000

        # Initialize audio
        self.audio = pyaudio.PyAudio()

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, 'recognized_text', 10)

        # Timer for continuous recognition
        self.timer = self.create_timer(5.0, self.recognize_audio)

    def recognize_audio(self):
        # Record audio
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        frames = []
        for i in range(0, int(self.rate / self.chunk * 5)):  # 5 seconds
            data = stream.read(self.chunk)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Convert to numpy array and process
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_float = audio_data.astype(np.float32) / 32768.0

        # Transcribe with Whisper
        result = self.model.transcribe(audio_float)
        text = result["text"]

        # Publish recognized text
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

        self.get_logger().info(f'Recognized: {text}')

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperSpeechToText()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()
```

### Example 2: Streaming Speech Recognition
```python
import whisper
import torch
import numpy as np
from queue import Queue
import threading

class StreamingWhisper:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = Queue()
        self.result_queue = Queue()
        self.is_running = False

    def start_streaming(self):
        self.is_running = True
        recognition_thread = threading.Thread(target=self._process_audio)
        recognition_thread.start()

    def _process_audio(self):
        while self.is_running:
            if not self.audio_queue.empty():
                audio_chunk = self.audio_queue.get()

                # Transcribe the audio chunk
                result = self.model.transcribe(audio_chunk)

                # Put result in queue for processing
                self.result_queue.put(result)

    def add_audio_chunk(self, audio_data):
        self.audio_queue.put(audio_data)

    def get_transcription(self):
        if not self.result_queue.empty():
            return self.result_queue.get()
        return None

# Usage in robot system
streaming_whisper = StreamingWhisper()
streaming_whisper.start_streaming()

# Continuously add audio chunks and get results
while robot_running:
    audio_chunk = get_microphone_audio()
    streaming_whisper.add_audio_chunk(audio_chunk)

    result = streaming_whisper.get_transcription()
    if result:
        process_command(result["text"])
```

## Diagram Placeholders
*Diagram showing the Whisper model architecture*

*Pipeline for voice-to-text processing in robotics*

## Summary
Whisper provides state-of-the-art speech recognition capabilities that can be integrated into robotics systems for natural voice interaction. Understanding its architecture and integration patterns is crucial for developing voice-controlled robots.

## Exercises
1. Install and test Whisper on your robotics platform.
2. Implement a voice command recognition system for robot navigation.
3. Compare Whisper with other speech recognition systems for robotics.
4. Research and describe techniques for improving real-time performance.