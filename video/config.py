"""
Video Configuration
Contains camera connection settings and parameters
"""

# Camera connection settings
CAMERA_SETTINGS = {
    # Try these camera indices in order
    'usb_indices': [0, 1, 2, 3, 4, 5],
    
    # Camera backends to try (in order of preference)
    'backends': [
        'cv2.CAP_DSHOW',    # DirectShow (Windows)
        'cv2.CAP_MSMF',     # Microsoft Media Foundation
        'cv2.CAP_V4L2',     # Video4Linux (Linux)
        'cv2.CAP_GSTREAMER' # GStreamer
    ],
    
    # Resolution preferences (try in order)
    'resolutions': [
        (1920, 1080),  # 1080p
        (1280, 720),   # 720p  
        (640, 480),    # VGA
        (320, 240)     # QVGA
    ],
    
    # Frame rate preferences
    'fps_targets': [30, 25, 20, 15, 10],
    
    # Connection timeout
    'connection_timeout': 5.0,
    
    # Frame read timeout  
    'frame_timeout': 2.0
}

# Display settings
DISPLAY_SETTINGS = {
    'window_name': 'Camo Studio Test',
    'window_size': (1280, 720),
    'show_fps': True,
    'show_resolution': True,
    'show_timestamp': True,
    'fps_update_interval': 30  # frames
}

# Performance settings optimized for 14700KF (28 threads)
PERFORMANCE_SETTINGS = {
    'max_fps': 30,  # Match Camo Studio's 30fps limit
    'buffer_size': 1,
    'use_threading': True,
    'frame_skip_threshold': 2,  # Very aggressive frame skipping
    
    # Multi-threading settings
    'capture_threads': 2,      # Dedicated capture threads
    'encode_threads': 4,       # JPEG encoding threads 
    'publish_threads': 2,      # WebSocket publishing threads
    'use_thread_pool': True,   # Use ThreadPoolExecutor
    'thread_priority': 'high'  # High thread priority
}