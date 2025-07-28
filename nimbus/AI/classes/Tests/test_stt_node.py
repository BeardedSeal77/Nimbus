import soundfile as sf
import numpy as np
from nimbus.AI.classes.stt_node import stt_node

def load_audio(filepath: str) -> np.ndarray:
    """
    Loads a .wav file and returns audio data and sample rate.
    """
    data, sample_rate = sf.read(filepath)
    
    # If stereo, convert to mono
    if len(data.shape) > 1:
        data = np.mean(data, axis=1)
    
    return data, sample_rate

def test_stt_on_sample():
    audio_path = "nimbus/AI/classes/Tests/test_audio.wav"
    print(f"🔊 Loading audio file: {audio_path}")
    
    try:
        audio_data, sr = load_audio(audio_path)
        print(f"Audio loaded. Shape: {audio_data.shape}, Sample rate: {sr}")
        
        print("Running STT...")
        transcript = stt_node(audio_data, sample_rate=sr)
        
        print("📝 Transcript:")
        print(transcript)
    except Exception as e:
        print(f"❌ Test failed: {e}")

if __name__ == "__main__":
    test_stt_on_sample()
