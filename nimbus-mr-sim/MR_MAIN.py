import threading
import mr_drone_pose_subscriber
import mr_messaging_subscriber
import mr_object_telemetry_subscriber
import mr_video_subscriber
import mr_yaw_publisher
import time
import multiprocessing

def run_in_thread(target):
    t = threading.Thread(target=target)
    t.daemon = True
    t.start()
    return t

def run_in_process(target):
    p = multiprocessing.Process(target=target)
    p.daemon = True
    p.start()
    return p

if __name__ == "__main__":
    # threads = [
    #     run_in_thread(mr_drone_pose_subscriber.main),
    #     run_in_thread(mr_messaging_subscriber.main),
    #     run_in_thread(mr_object_telemetry_subscriber.main),
    #     run_in_thread(mr_video_subscriber.main),
    #     # run_in_thread(mr_yaw_publisher.main)
    # ]

    processes = [
        run_in_process(mr_drone_pose_subscriber.main),
        run_in_process(mr_messaging_subscriber.main),
        run_in_process(mr_object_telemetry_subscriber.main),
        run_in_process(mr_video_subscriber.main)
    ]

    time.sleep(0.5)
    print("All subsystems started. Press Ctrl+C to stop.")
    try:
        for p in processes:
            p.join()
    except KeyboardInterrupt:
        print("\nShutting down all processes...")
        for p in processes:
            p.terminate()
    # try:
    #     while True:
    #         pass  # keep main alive
    # except KeyboardInterrupt:
    #     print("\nShutting down all threads...") 