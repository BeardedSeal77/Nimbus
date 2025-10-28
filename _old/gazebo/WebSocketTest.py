import websocket
import json
try:
    ws = websocket.create_connection('ws://192.168.8.102:9090')
    print('✅ WebSocket connected to rosbridge!')       

    # Test ROS2 topic list via rosbridge
    msg = {
        'op': 'call_service',
        'service': '/rosapi/topics',
        'args': {}
    }
    ws.send(json.dumps(msg))
    result = ws.recv()
    print('📋 Available topics:', result)

    ws.close()
except Exception as e:
    print(f'❌ WebSocket failed: {e}')