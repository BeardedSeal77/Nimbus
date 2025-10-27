"""
Audio Service
Handles microphone recording, STT, and intent extraction
"""

import threading
import logging
import numpy as np
import pyaudio
import sys
import os
from queue import Queue
import time

logger = logging.getLogger(__name__)

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'classes'))
from stt_node import stt_node
from intent_object_node import intent_object_node

class AudioService:
    def __init__(self):
        self.app = None
        self.recording = False
        self.audio_data = []
        self.thread = None

        self.sample_rate = 16000
        self.channels = 1
        self.chunk_size = 1024
        self.format = pyaudio.paInt16

        self.pyaudio_instance = None
        self.stream = None

    def initialize(self, app):
        """Initialize with app config"""
        self.app = app
        logger.info("Audio service initialized")

    def start_recording(self):
        """Start recording from microphone"""
        if self.recording:
            logger.warning("Already recording")
            return False

        self.recording = True
        self.audio_data = []

        self.thread = threading.Thread(target=self._record_audio, daemon=True)
        self.thread.start()

        logger.info("Started audio recording")
        return True

    def stop_recording(self):
        """Stop recording and process audio through pipeline"""
        if not self.recording:
            logger.warning("Not currently recording")
            return {"success": False, "error": "Not recording"}

        self.recording = False

        if self.thread:
            self.thread.join(timeout=2.0)

        logger.info(f"Stopped recording. Captured {len(self.audio_data)} chunks")

        if len(self.audio_data) == 0:
            logger.warning("No audio data captured")
            return {"success": False, "error": "No audio captured"}

        result = self._process_audio()
        return result

    def _record_audio(self):
        """Record audio in background thread"""
        try:
            self.pyaudio_instance = pyaudio.PyAudio()

            self.stream = self.pyaudio_instance.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            logger.info(f"Audio stream opened: {self.sample_rate}Hz, {self.channels} channel(s)")

            while self.recording:
                try:
                    data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                    self.audio_data.append(data)
                except Exception as e:
                    logger.error(f"Error reading audio chunk: {e}")
                    break

        except Exception as e:
            logger.error(f"Failed to initialize audio recording: {e}")
            self.recording = False

        finally:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()
            logger.info("Audio stream closed")

    def _process_audio(self):
        """Process recorded audio through STT and intent extraction"""
        try:
            audio_bytes = b''.join(self.audio_data)
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            audio_float = audio_array.astype(np.float32) / 32768.0

            logger.info(f"Processing audio: {len(audio_float)} samples, {len(audio_float)/self.sample_rate:.2f} seconds")

            transcript = stt_node(audio_float, self.sample_rate)
            logger.info(f"Transcript: '{transcript}'")

            if not transcript or transcript.strip() == "":
                logger.warning("Empty transcript from STT")
                return {"success": False, "error": "No speech detected"}

            result = intent_object_node(transcript)
            logger.info(f"Intent: '{result['intent']}', Object: '{result['object']}'")

            if self.app:
                with self.app.config['STATE_LOCK']:
                    # if result['intent']:
                    self.app.config['GLOBAL_INTENT'] = result['intent']
                    if hasattr(self.app, 'shared_state'):
                        self.app.shared_state['global_intent'] = result['intent']
                        # Trigger object position calculation if intent is 'go'
                        if result['intent'].lower() == 'go':
                            self.app.shared_state['calculate_position_trigger'] = True
                            logger.info("Position calculation trigger SET (intent='go')")

                    # if result['object']:
                    self.app.config['GLOBAL_OBJECT'] = result['object']
                    if hasattr(self.app, 'shared_state'):
                        self.app.shared_state['global_object'] = result['object']
                        # Also update target_object for UI display
                        self.app.shared_state['target_object'] = result['object']
                    
                    # if no object detected, set position to none
                    if not result['object']:
                        self.app.shared_state['object_absolute_position'] = None

                    # Sets hud message
                    if result['intent'].lower() == 'go' and (not result['object'] or result['object'].lower() == 'none'):
                        self.app.shared_state['hud_message'] = {
                            'text': "No such object detected, please repeat!",
                            'timestamp': time.time()
                        }
                    if not result['intent'] or result['intent'] == 'none':
                        self.app.shared_state['hud_message'] = {
                            'text': "No intent, please repeat!",
                            'timestamp': time.time()
                        }

                logger.info(f"Updated globals: intent='{result['intent']}', object='{result['object']}'")

            return {
                "success": True,
                "transcript": transcript,
                "intent": result['intent'],
                "object": result['object']
            }

        except Exception as e:
            logger.error(f"Audio processing failed: {e}", exc_info=True)
            return {"success": False, "error": str(e)}
