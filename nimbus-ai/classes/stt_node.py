from transformers.pipelines import pipeline
import tempfile
import soundfile as sf
import os
import numpy as np

#import librosa
#import librosa
#import noisereduce as nr
#from typing import Tuple


# Load model once
pipe = pipeline("automatic-speech-recognition", model="openai/whisper-base", device=-1)

"""
def preprocess_audio(audio_data: np.ndarray, original_sample_rate: int) -> Tuple[np.ndarray, int]:

    Enhances audio by normalizing volume, removing noise, trimming silence, and resampling to 16 kHz.

    Parameters:
        audio_data (np.ndarray): Raw audio waveform
        original_sample_rate (int): Original sample rate of audio

    Returns:
        Tuple[np.ndarray, int]: (Processed audio, sample rate 16000)

    # Step 1: Normalize volume to -1.0 to 1.0 range
    peak = np.max(np.abs(audio_data))
    if peak > 0:
        audio_data = audio_data / peak

    # Step 2: Noise reduction using the first 0.5 sec as noise profile
    noise_sample = audio_data[:int(0.5 * original_sample_rate)]  # first 0.5 sec
    audio_data = nr.reduce_noise(y=audio_data, sr=original_sample_rate, y_noise=noise_sample, verbose=False)

    # Step 3: Trim silence
    audio_data, _ = librosa.effects.trim(audio_data, top_db=30)

    # Step 4: Resample to 16 kHz
    audio_data_resampled = librosa.resample(audio_data.astype(np.float32), orig_sr=original_sample_rate, target_sr=16000)

    return audio_data_resampled, 16000
"""


def stt_node(audio_data: np.ndarray, sample_rate: int = 44100) -> str:
    try:

        # Normalize and resample
        # processed_audio, processed_sr = preprocess_audio(audio_data, sample_rate)

        # Write audio to temporary WAV file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            sf.write(tmp.name, audio_data, sample_rate)
            temp_path = tmp.name

        # Run transcription
        result = pipe(temp_path)["text"]

        # Clean up
        os.remove(temp_path)
        return result.strip()

    except Exception as e:
        raise RuntimeError(f"STT failed: {e}")
