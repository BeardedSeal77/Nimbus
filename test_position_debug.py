import requests
import json

HUB_URL = "http://localhost:5000"

print("=" * 80)
print("OBJECT POSITION CALCULATION DEBUG")
print("=" * 80)

try:
    response = requests.get(f"{HUB_URL}/api/debug/position_calculation", timeout=2)

    if response.status_code == 200:
        data = response.json()

        print("\nTRIGGER STATUS:")
        print(f"  calculate_position_trigger: {data.get('calculate_position_trigger')}")
        print(f"  global_intent: '{data.get('global_intent')}'")
        print(f"  global_object: '{data.get('global_object')}'")
        print(f"  target_object: '{data.get('target_object')}'")

        print("\nCALCULATED POSITION:")
        if data.get('object_absolute_position'):
            pos = data['object_absolute_position']
            print(f"  x: {pos.get('x')} m")
            print(f"  y: {pos.get('y')} m")
            print(f"  z: {pos.get('z')} m")
        else:
            print("  NOT CALCULATED YET")

        print("\nCAMERA CONFIG:")
        if data.get('has_camera_config'):
            cfg = data.get('camera_config')
            if cfg:
                print(f"  FOV: {cfg.get('fov')} rad")
                print(f"  Width: {cfg.get('width')} px")
                print(f"  Height: {cfg.get('height')} px")
        else:
            print("  NOT RECEIVED YET")

        print("\nDRONE STATE:")
        if data.get('has_drone_telemetry'):
            print(f"  Position: {data.get('drone_position')}")
            print(f"  Yaw: {data.get('drone_yaw')} rad")
        else:
            print("  NO TELEMETRY")

        print("\n" + "=" * 80)
        print("RAW JSON:")
        print(json.dumps(data, indent=2))

    else:
        print(f"ERROR: HTTP {response.status_code}")
        print(response.text)

except requests.exceptions.ConnectionError:
    print("ERROR: Cannot connect to hub. Is it running?")
except Exception as e:
    print(f"ERROR: {e}")

print("\n" + "=" * 80)
