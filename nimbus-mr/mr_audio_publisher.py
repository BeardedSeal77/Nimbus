# publish microphone audio to ROS via rosbridge websocket
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mr/audio"
import json
import time
import websocket
import pyaudio

# ROS 2 WebSocket bridge
ROS2_WS_URL = "ws://localhost:9090"
TOPIC = "/mr/audio"
MSG_TYPE = "std_msgs/UInt8MultiArray"   # ROS message for audio

# Audio settings
RATE = 16000         # Hz sample rate
CHUNK = 1024         # samples per frame
CHANNELS = 1
FORMAT = pyaudio.paInt16   # 16-bit PCM


def main():
    # Connect to rosbridge
    ws = websocket.WebSocket()
    ws.connect(ROS2_WS_URL)
    print(f"Connected to rosbridge at {ROS2_WS_URL}")

    # Advertise topic
    advertise_msg = {
        "op": "advertise",
        "topic": TOPIC,
        "type": MSG_TYPE
    }
    ws.send(json.dumps(advertise_msg))
    print(f"Advertised topic {TOPIC} as {MSG_TYPE}")

    # Init microphone
    pa = pyaudio.PyAudio()
    # List all audio input devices
    print("Available audio input devices:")
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:  # only input devices
            print(f"Index {i}: {info['name']} ({info['maxInputChannels']} channels)")

    # Example: pick device index manually
    device_index = int(input("Enter the device index to use: "))

    stream = pa.open(format=FORMAT,
                     channels=CHANNELS,
                     rate=RATE,
                     input=True,
                     input_device_index=device_index,
                     frames_per_buffer=CHUNK)
    print(f"Using device {device_index}: {pa.get_device_info_by_index(device_index)['name']}")

    try:
        while True:
            # Read microphone chunk
            data = stream.read(CHUNK, exception_on_overflow=False)

            # AudioData.msg expects uint8[] -> convert from bytes
            audio_msg = {
                "op": "publish",
                "topic": TOPIC,
                "msg": {
                    "data": list(data)   # convert bytes -> list of ints
                }
            }

            ws.send(json.dumps(audio_msg))
            print(f"Published {len(data)} bytes of audio")

            # Small delay keeps CPU sane
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping audio publisher...")
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()
        ws.close()
        print("WebSocket closed")


if __name__ == "__main__":
    main()
