"""
Drone State Service
Subscribes to Nimbus Hub via SSE and updates web interface state
"""

import threading
import time
import logging
import json
import requests

logger = logging.getLogger(__name__)

class DroneStateService:
    def __init__(self):
        self.app = None
        self.thread = None
        self.running = False

        # Hub connection
        self.hub_url = 'http://localhost:8000'
        self.connected = False

        # Latest drone state
        self.drone_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular_velocity': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'facing_vector': {'x': 1.0, 'y': 0.0, 'z': 0.0},
            'mode': 'UNKNOWN',
            'target_altitude': 0.0,
            'distance_to_home': 0.0,
            'connected': False
        }

    def start_service(self, app):
        """Start the drone state service"""
        self.app = app
        self.running = True

        # Start Hub SSE subscription thread
        self.thread = threading.Thread(target=self._hub_sse_loop, daemon=True)
        self.thread.start()

        app.config['DRONE_STATE_SERVICE_RUNNING'] = True
        logger.info("Drone state service started - subscribing to Nimbus Hub")

    def stop_service(self):
        """Stop the drone state service"""
        self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.app:
            self.app.config['DRONE_STATE_SERVICE_RUNNING'] = False
        logger.info("Drone state service stopped")

    def _hub_sse_loop(self):
        """Main Hub SSE subscription loop"""
        retry_count = 0
        max_retries = 10

        while self.running and retry_count < max_retries:
            try:
                logger.info(f"Connecting to Nimbus Hub at {self.hub_url}")

                # Connect to Hub SSE stream
                response = requests.get(
                    f"{self.hub_url}/stream/drone/state",
                    stream=True,
                    timeout=5
                )

                if response.status_code == 200:
                    self.connected = True
                    self.drone_state['connected'] = True
                    logger.info("Connected to Nimbus Hub SSE stream")
                    retry_count = 0

                    # Read SSE stream
                    for line in response.iter_lines(decode_unicode=True):
                        if not self.running:
                            break

                        if line and line.startswith('data: '):
                            try:
                                # Parse JSON data from SSE
                                data_str = line[6:]  # Remove "data: " prefix
                                state_update = json.loads(data_str)

                                # Hub sends complete state in 'message' field
                                if 'message' in state_update:
                                    self._update_state(state_update['message'])
                                else:
                                    # Direct state update
                                    self._update_state(state_update)

                            except json.JSONDecodeError as e:
                                logger.debug(f"JSON decode error: {e}")
                            except Exception as e:
                                logger.error(f"Error processing SSE data: {e}")

                else:
                    logger.warning(f"Hub connection failed with status {response.status_code}")
                    self.connected = False
                    self.drone_state['connected'] = False
                    retry_count += 1
                    time.sleep(2)

            except requests.exceptions.ConnectionError:
                logger.warning("Cannot connect to Hub - is it running?")
                self.connected = False
                self.drone_state['connected'] = False
                retry_count += 1
                time.sleep(2)

            except Exception as e:
                logger.error(f"Hub SSE error: {e}")
                self.connected = False
                self.drone_state['connected'] = False
                retry_count += 1
                time.sleep(2)

        if retry_count >= max_retries:
            logger.error(f"Failed to connect to Hub after {max_retries} attempts")

    def _update_state(self, state_update):
        """Update drone state from Hub message"""
        try:
            # Hub sends complete state, just update everything
            if 'position' in state_update:
                self.drone_state['position'] = state_update['position']
            if 'orientation' in state_update:
                self.drone_state['orientation'] = state_update['orientation']
            if 'velocity' in state_update:
                self.drone_state['velocity'] = state_update['velocity']
            if 'acceleration' in state_update:
                self.drone_state['acceleration'] = state_update['acceleration']
            if 'angular_velocity' in state_update:
                self.drone_state['angular_velocity'] = state_update['angular_velocity']
            if 'facing_vector' in state_update:
                self.drone_state['facing_vector'] = state_update['facing_vector']
            if 'mode' in state_update:
                self.drone_state['mode'] = state_update['mode']
            if 'target_altitude' in state_update:
                self.drone_state['target_altitude'] = state_update['target_altitude']
            if 'distance_to_home' in state_update:
                self.drone_state['distance_to_home'] = state_update['distance_to_home']
            if 'connected' in state_update:
                self.drone_state['connected'] = state_update['connected']

            # Push to web interface
            self._push_state_to_web()

        except Exception as e:
            logger.error(f"Error updating state: {e}")

    def _push_state_to_web(self):
        """Update web interface with latest drone state"""
        try:
            from routes.web_routes import update_drone_state
            update_drone_state(self.drone_state)
        except Exception as e:
            logger.error(f"Error pushing state to web: {e}")

    def get_state(self):
        """Get current drone state"""
        return self.drone_state.copy()


# Global service instance
drone_state_service_instance = DroneStateService()

def start_service(app):
    """Start the drone state service"""
    drone_state_service_instance.start_service(app)

def stop_service():
    """Stop the drone state service"""
    drone_state_service_instance.stop_service()

def get_state():
    """Get current drone state"""
    return drone_state_service_instance.get_state()