import threading
import mr_drone_pose_subscriber
import mr_messaging_subscriber
import mr_object_telemetry_subscriber
import mr_video_subscriber
import mr_yaw_publisher
import time

def run_in_thread(target):
    t = threading.Thread(target=target)
    t.daemon = True
    t.start()
    return t

if __name__ == "__main__":
    threads = [
        run_in_thread(mr_drone_pose_subscriber.main),
        run_in_thread(mr_messaging_subscriber.main),
        run_in_thread(mr_object_telemetry_subscriber.main),
        run_in_thread(mr_video_subscriber.main),
        run_in_thread(mr_yaw_publisher.main)
    ]

    time.sleep(0.5)
    print("All subsystems started. Press Ctrl+C to stop.")
    try:
        while True:
            pass  # keep main alive
    except KeyboardInterrupt:
        print("\nShutting down all threads...") 