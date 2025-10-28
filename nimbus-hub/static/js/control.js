// Server-Sent Events for live drone state
let droneStateSource = null;

function connectDroneState() {
    droneStateSource = new EventSource('/drone_state_stream');

    droneStateSource.onmessage = function(event) {
        const state = JSON.parse(event.data);
        updateDroneDisplay(state);
    };

    droneStateSource.onerror = function(error) {
        console.error('Drone state SSE error:', error);
        droneStateSource.close();
        // Retry connection after 5 seconds
        setTimeout(connectDroneState, 5000);
    };
}

function updateDroneDisplay(state) {
    // Position
    document.getElementById('pos-x').textContent = state.position.x.toFixed(2);
    document.getElementById('pos-y').textContent = state.position.y.toFixed(2);
    document.getElementById('pos-z').textContent = state.position.z.toFixed(2);

    // Orientation
    document.getElementById('ori-roll').textContent = state.orientation.roll.toFixed(3);
    document.getElementById('ori-pitch').textContent = state.orientation.pitch.toFixed(3);
    document.getElementById('ori-yaw').textContent = state.orientation.yaw.toFixed(3);

    // Velocity
    document.getElementById('vel-x').textContent = state.velocity.x.toFixed(2);
    document.getElementById('vel-y').textContent = state.velocity.y.toFixed(2);
    document.getElementById('vel-z').textContent = state.velocity.z.toFixed(2);

    // Acceleration
    document.getElementById('accel-x').textContent = state.acceleration.x.toFixed(2);
    document.getElementById('accel-y').textContent = state.acceleration.y.toFixed(2);
    document.getElementById('accel-z').textContent = state.acceleration.z.toFixed(2);

    // Angular velocity
    document.getElementById('ang-roll').textContent = state.angular_velocity.roll.toFixed(3);
    document.getElementById('ang-pitch').textContent = state.angular_velocity.pitch.toFixed(3);
    document.getElementById('ang-yaw').textContent = state.angular_velocity.yaw.toFixed(3);

    // Facing vector
    document.getElementById('facing-x').textContent = state.facing_vector.x.toFixed(2);
    document.getElementById('facing-y').textContent = state.facing_vector.y.toFixed(2);
    document.getElementById('facing-z').textContent = state.facing_vector.z.toFixed(2);

    // Flight info
    document.getElementById('flight-mode').textContent = state.mode;
    document.getElementById('target-alt').textContent = state.target_altitude.toFixed(2);
    document.getElementById('home-dist').textContent = state.distance_to_home.toFixed(2);

    // Connection status
    const connStatus = document.getElementById('connection-status');
    if (state.connected) {
        connStatus.style.color = '#4CAF50';
        connStatus.textContent = 'Connected';
    } else {
        connStatus.style.color = 'red';
        connStatus.textContent = 'Disconnected';
    }
}

// Auto-refresh status every 2 seconds
setInterval(refreshStatus, 2000);

// Load status on page load and connect to drone state stream
window.onload = function() {
    refreshStatus();
    connectDroneState();
};

async function makeRequest(url, method = 'GET', data = null) {
    try {
        const options = {
            method: method,
            headers: {
                'Content-Type': 'application/json',
            }
        };

        if (data) {
            options.body = JSON.stringify(data);
        }

        const response = await fetch(url, options);
        const result = await response.json();

        if (!response.ok) {
            throw new Error(result.message || 'Request failed');
        }

        return result;
    } catch (error) {
        showMessage(`Error: ${error.message}`, 'error');
        throw error;
    }
}

async function triggerDepthDetection() {
    try {
        const result = await makeRequest('/api/trigger_depth', 'POST');
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error triggering depth detection:', error);
    }
}

async function stopDepthDetection() {
    try {
        const result = await makeRequest('/api/stop_depth', 'POST');
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error stopping depth detection:', error);
    }
}

async function setDepthMethod() {
    try {
        const method = document.getElementById('depth-method').value;
        const result = await makeRequest('/api/set_depth_method', 'POST', {
            depth_method: method
        });
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error setting depth method:', error);
        showMessage('Failed to set depth method', 'error');
    }
}

async function setTargetObject() {
    try {
        const targetObject = document.getElementById('target-object').value;
        if (!targetObject) {
            showMessage('Please enter an object name', 'error');
            return;
        }
        const result = await makeRequest('/api/set_target_object', 'POST', {
            object: targetObject
        });
        showMessage(result.message, 'success');
        document.getElementById('target-object').value = '';
        refreshStatus();
    } catch (error) {
        console.error('Error setting target object:', error);
    }
}

async function resetSystem() {
    try {
        const result = await makeRequest('/api/reset_system', 'POST');
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error resetting system:', error);
    }
}

async function setAIParameters() {
    try {
        const frameIntervalMs = parseInt(document.getElementById('frame-interval-ms').value);

        const result = await makeRequest('/api/set_ai_params', 'POST', {
            frame_processing_interval_ms: frameIntervalMs
        });
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error setting AI parameters:', error);
    }
}

async function setDepthParameters() {
    try {
        const frameCount = parseInt(document.getElementById('frame-count').value);
        const referencePosition = parseInt(document.getElementById('reference-position').value);

        const result = await makeRequest('/api/set_depth_params', 'POST', {
            frame_count: frameCount,
            reference_frame_position: referencePosition
        });
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error setting depth parameters:', error);
    }
}

async function setROS2Parameters() {
    try {
        const ros2Host = document.getElementById('ros2-host').value;
        const ros2Port = parseInt(document.getElementById('ros2-port').value);

        const result = await makeRequest('/api/set_ros2_params', 'POST', {
            ros2_host: ros2Host,
            ros2_port: ros2Port
        });
        showMessage(result.message + ' (Restart required for ROS2 changes)', 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error setting ROS2 parameters:', error);
    }
}

async function setGlobalState() {
    try {
        const globalIntent = document.getElementById('global-intent').value;
        const globalStateActive = document.getElementById('global-state-active').value === 'true';
        const globalCommandStatus = document.getElementById('global-command-status').value;

        const result = await makeRequest('/api/set_global_state', 'POST', {
            global_intent: globalIntent,
            global_state_active: globalStateActive,
            global_command_status: globalCommandStatus
        });
        showMessage(result.message, 'success');
        refreshStatus();
    } catch (error) {
        console.error('Error setting global state:', error);
    }
}

async function refreshStatus() {
    try {
        const status = await makeRequest('/api/status');
        updateStatusDisplay(status);
    } catch (error) {
        console.error('Error refreshing status:', error);
    }
}

function updateStatusDisplay(status) {
    // Update depth status
    const depthStatusElement = document.getElementById('depth-status');
    const depthActive = status.global_get_dist === 1;
    const depthBusy = status.depth_processing_busy;

    let depthStatusClass = depthActive ? 'status-active' : 'status-inactive';
    if (depthBusy) depthStatusClass = 'status-processing';

    depthStatusElement.innerHTML = `
<span class="status-indicator ${depthStatusClass}"></span>Depth Detection: ${depthActive ? 'ENABLED' : 'DISABLED'}
Target Distance: ${status.global_target_distance > 0 ? status.global_target_distance.toFixed(2) + 'm' : 'Not calculated'}
Depth Processing: ${depthBusy ? 'BUSY' : 'Ready'}
Movement Frames: ${status.depth_frame_count} frames (detections 1-${status.depth_frame_count})
Reference Frame: Detection #${status.reference_frame_position || 10}`;

    // Update current target object display (not the input field)
    const currentTargetElement = document.getElementById('current-target-object');
    if (currentTargetElement) {
        currentTargetElement.textContent = status.global_object || 'car';
    }

    // Update form values (but not target-object input - user might be typing)
    document.getElementById('depth-method').value = status.depth_method || 'TRIG';
    document.getElementById('frame-count').value = status.depth_frame_count;
    document.getElementById('reference-position').value = status.reference_frame_position || 10;
    document.getElementById('frame-interval-ms').value = status.frame_processing_interval_ms || 100;
    document.getElementById('ros2-host').value = status.ros2_host || 'localhost';
    document.getElementById('ros2-port').value = status.ros2_port || 9090;
    document.getElementById('global-intent').value = status.global_intent || '';
    document.getElementById('global-state-active').value = status.global_state_active ? 'true' : 'false';
    document.getElementById('global-command-status').value = status.global_command_status || 'none';
}

function showMessage(message, type) {
    const messageArea = document.getElementById('message-area');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${type}`;
    messageDiv.textContent = message;

    messageArea.appendChild(messageDiv);

    // Remove message after 5 seconds
    setTimeout(() => {
        messageDiv.remove();
    }, 5000);
}

// Audio Recording Functions
async function startAudioRecording() {
    const recordBtn = document.getElementById('audio-record-btn');
    const stopBtn = document.getElementById('audio-stop-btn');
    const statusDiv = document.getElementById('audio-status');

    try {
        const response = await fetch('/api/audio/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });

        const data = await response.json();

        if (data.status === 'ok') {
            recordBtn.disabled = true;
            stopBtn.disabled = false;
            statusDiv.innerHTML = '<span style="color: red;">🔴 Recording in progress...</span>';
            statusDiv.style.background = '#fff3cd';
        } else {
            statusDiv.innerHTML = `<span style="color: red;">Error: ${data.message}</span>`;
        }
    } catch (error) {
        statusDiv.innerHTML = `<span style="color: red;">Error: ${error.message}</span>`;
    }
}

async function stopAudioRecording() {
    const recordBtn = document.getElementById('audio-record-btn');
    const stopBtn = document.getElementById('audio-stop-btn');
    const statusDiv = document.getElementById('audio-status');

    stopBtn.disabled = true;
    statusDiv.innerHTML = '<span style="color: orange;">⏳ Processing audio...</span>';

    try {
        const response = await fetch('/api/audio/stop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });

        const data = await response.json();

        if (data.status === 'ok') {
            const result = data.result;

            if (result.success) {
                statusDiv.innerHTML = `
                    <div style="color: green; font-weight: bold;">✅ Audio Processed Successfully</div>
                    <div style="margin-top: 5px; color: black;">
                        <strong>Transcript:</strong> "${result.transcript}"<br>
                        <strong>Intent:</strong> ${result.intent || 'none'}<br>
                        <strong>Object:</strong> ${result.object || 'none'}
                    </div>
                `;
                statusDiv.style.background = '#d4edda';
                showMessage(`Voice command processed: ${result.intent} ${result.object}`, 'success');

                // Refresh status to show updated globals
                setTimeout(refreshStatus, 500);
            } else {
                statusDiv.innerHTML = `<span style="color: red;">❌ Error: ${result.error}</span>`;
                statusDiv.style.background = '#f8d7da';
            }
        } else {
            statusDiv.innerHTML = `<span style="color: red;">Error: ${data.message}</span>`;
            statusDiv.style.background = '#f8d7da';
        }
    } catch (error) {
        statusDiv.innerHTML = `<span style="color: red;">Error: ${error.message}</span>`;
        statusDiv.style.background = '#f8d7da';
    } finally {
        recordBtn.disabled = false;
        stopBtn.disabled = true;
    }
}

async function processManualTranscript() {
    const transcriptInput = document.getElementById('manual-transcript');
    const statusDiv = document.getElementById('manual-status');
    const transcript = transcriptInput.value.trim();

    if (!transcript) {
        showMessage('Please enter a command', 'error');
        return;
    }

    statusDiv.innerHTML = '<span style="color: orange;">⏳ Processing command...</span>';

    try {
        const response = await fetch('/api/audio/process_text', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ transcript: transcript })
        });

        const data = await response.json();

        if (data.status === 'ok' && data.result.success) {
            const result = data.result;
            statusDiv.innerHTML = `
                <div style="color: green; font-weight: bold;">✅ Command Processed</div>
                <div style="margin-top: 5px; color: black;">
                    <strong>Transcript:</strong> "${result.transcript}"<br>
                    <strong>Intent:</strong> ${result.intent || 'none'}<br>
                    <strong>Object:</strong> ${result.object || 'none'}
                </div>
            `;
            statusDiv.style.background = '#d4edda';
            showMessage(`Command processed: ${result.intent} ${result.object}`, 'success');

            transcriptInput.value = '';
            setTimeout(refreshStatus, 500);
        } else {
            statusDiv.innerHTML = `<span style="color: red;">Error: ${data.result?.error || 'Processing failed'}</span>`;
            statusDiv.style.background = '#f8d7da';
        }
    } catch (error) {
        statusDiv.innerHTML = `<span style="color: red;">Error: ${error.message}</span>`;
        statusDiv.style.background = '#f8d7da';
    }
}
