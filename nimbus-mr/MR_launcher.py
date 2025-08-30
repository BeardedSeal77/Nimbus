import subprocess
import os

scripts = [
    "mr_video_subscriber.py",
    "mr_yaw_publisher.py",
    "TESTING_detection_publisher.py",
    "TESTING_messaging_publisher.py",
    "TESTING_pos_publisher.py"
]

path = os.path.dirname(os.path.abspath(__file__))

for script in scripts:
    script_path = os.path.join(path, script)
    subprocess.Popen(["start", "cmd", "/k", "python", script_path], shell=True)