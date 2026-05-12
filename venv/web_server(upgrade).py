from flask import Flask, render_template, jsonify, request, send_from_directory
import json
import os
import time
import threading

app = Flask(__name__, static_folder='static', template_folder='templates')

CONFIG_FILE = 'config.json'

# --- Locks ---
# Protects all multi-field reads/writes on drone_state
_state_lock = threading.Lock()
# Protects all config.json reads/writes
_config_lock = threading.Lock()

drone_state = {
    'emergency_stop':    False,
    'launch_authorized': False,
    'destination':       '',
    'mode':              'idle',   # idle | preflight | airborne | test | dry_run
    'last_heartbeat':    None,
}


# --- Thread-safe state helpers ---

def get_state(*keys):
    """Read one or more state fields atomically."""
    with _state_lock:
        if len(keys) == 1:
            return drone_state.get(keys[0])
        return {k: drone_state.get(k) for k in keys}


def set_state(**kwargs):
    """Write one or more state fields atomically."""
    with _state_lock:
        drone_state.update(kwargs)


# --- Config I/O (file-locked + atomic writes) ---

def load_config():
    with _config_lock:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        return {}


def save_config(config):
    """Write atomically: write to .tmp then rename so a crash never corrupts the file."""
    with _config_lock:
        tmp = CONFIG_FILE + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(config, f, indent=2)
        os.replace(tmp, CONFIG_FILE)  # atomic on POSIX / Linux


# --- Pages ---

@app.route('/')
def index():
    return render_template('index.html')


# --- Destinations API ---

@app.route('/api/destinations', methods=['GET'])
def get_destinations():
    return jsonify(load_config().get('destinations', {}))


@app.route('/api/destinations', methods=['POST'])
def create_destination():
    data = request.json
    dest_id = data.get('id')
    if not dest_id:
        return jsonify({'error': 'Destination ID required'}), 400
    config = load_config()
    config.setdefault('destinations', {})[dest_id] = {
        'name':        data.get('name', dest_id),
        'latitude':    float(data.get('latitude', 0)),
        'longitude':   float(data.get('longitude', 0)),
        'description': data.get('description', ''),
    }
    save_config(config)
    return jsonify({'success': True, 'message': f'Destination {dest_id} saved'}), 201


@app.route('/api/destinations/<dest_id>', methods=['DELETE'])
def delete_destination(dest_id):
    if dest_id == 'home':
        return jsonify({'error': 'Cannot delete home location'}), 403
    config = load_config()
    if dest_id in config.get('destinations', {}):
        del config['destinations'][dest_id]
        save_config(config)
        return jsonify({'success': True})
    return jsonify({'error': 'Not found'}), 404


@app.route('/api/home', methods=['GET'])
def get_home():
    return jsonify(load_config().get('destinations', {}).get('home', {}))


@app.route('/api/home', methods=['POST'])
def update_home():
    data = request.json
    config = load_config()
    config.setdefault('destinations', {})['home'] = {
        'name':        'Home',
        'latitude':    float(data.get('latitude', 0)),
        'longitude':   float(data.get('longitude', 0)),
        'description': 'Home Location',
    }
    save_config(config)
    return jsonify({'success': True}), 201


# --- Flight Parameters API ---

@app.route('/api/flight-parameters', methods=['GET'])
def get_flight_parameters():
    return jsonify(load_config().get('flight_parameters', {}))


@app.route('/api/flight-parameters', methods=['POST'])
def update_flight_parameters():
    data   = request.json
    config = load_config()
    params = config.setdefault('flight_parameters', {})

    limits = {
        'takeoff_altitude':    (1.0,  50.0),
        'flight_speed':        (0.1,  10.0),
        'yaw_sensitivity':     (0.001, 0.1),
        'obstacle_turn_rate':  (0.1,   2.0),
        'confidence_threshold':(0.1,   1.0),
        'low_battery_voltage': (18.0, 26.0),
    }
    errors = []
    for key, (lo, hi) in limits.items():
        if key in data:
            val = float(data[key])
            if not (lo <= val <= hi):
                errors.append(f'{key} must be between {lo} and {hi}')
            else:
                params[key] = val

    if errors:
        return jsonify({'error': '; '.join(errors)}), 400

    save_config(config)
    return jsonify({'success': True}), 201


# --- Control API ---

@app.route('/api/launch', methods=['POST'])
def launch():
    """Authorize the drone to arm and take off. Called from the web UI launch button."""
    data    = request.json or {}
    dest_id = data.get('destination', '').upper()

    config = load_config()
    if dest_id not in config.get('destinations', {}):
        return jsonify({'error': f'Unknown destination: {dest_id}'}), 400

    if get_state('emergency_stop'):
        return jsonify({'error': 'Emergency stop is active — clear it before launching'}), 403

    # Atomic: all three fields written together
    set_state(
        destination=dest_id,
        launch_authorized=True,
        last_heartbeat=time.time(),
    )
    return jsonify({'success': True, 'message': f'Launch authorized for destination {dest_id}'})


@app.route('/api/set-destination', methods=['POST'])
def set_destination():
    """Change destination mid-flight (future use)."""
    data    = request.json or {}
    dest_id = data.get('destination', '').upper()
    config  = load_config()
    if dest_id not in config.get('destinations', {}):
        return jsonify({'error': 'Unknown destination'}), 400
    set_state(destination=dest_id)
    return jsonify({'success': True})


# --- Safety API ---

@app.route('/api/emergency-stop', methods=['POST'])
def api_emergency_stop():
    """
    Trigger emergency stop. Requires confirmation token in body:
    { "confirm": "EMERGENCY_STOP_CONFIRMED" }
    This prevents accidental calls.
    """
    data = request.json or {}
    if data.get('confirm') != 'EMERGENCY_STOP_CONFIRMED':
        return jsonify({'error': 'Confirmation token required'}), 400

    # Atomic: clear launch auth at the same time
    set_state(emergency_stop=True, launch_authorized=False)
    return jsonify({'success': True, 'message': 'Emergency stop activated — drone will land immediately'})


@app.route('/api/emergency-stop/clear', methods=['POST'])
def clear_emergency_stop():
    """Clear the emergency stop flag so a new flight can be launched."""
    set_state(emergency_stop=False)
    return jsonify({'success': True, 'message': 'Emergency stop cleared'})


@app.route('/api/heartbeat', methods=['POST'])
def heartbeat():
    """
    Web controller heartbeat — POST this every 5 seconds from the UI.
    If the drone is airborne and heartbeats stop for 15 s, it auto-lands.
    """
    set_state(last_heartbeat=time.time())
    ts = get_state('last_heartbeat')
    return jsonify({'success': True, 'timestamp': ts})


# --- Status API ---

@app.route('/api/status', methods=['GET'])
def get_status():
    config = load_config()
    state  = get_state('mode', 'emergency_stop', 'launch_authorized',
                        'destination', 'last_heartbeat')
    return jsonify({
        'drone_mode':            state['mode'],
        'emergency_stop':        state['emergency_stop'],
        'launch_authorized':     state['launch_authorized'],
        'current_destination':   state['destination'],
        'destinations_count':    len(config.get('destinations', {})),
        'flight_parameters_set': len(config.get('flight_parameters', {})) > 0,
        'last_heartbeat':        state['last_heartbeat'],
        'config_loaded':         True,
    })


@app.route('/api/autostart', methods=['POST'])
def set_autostart():
    """Enable/disable auto-start on boot."""
    data    = request.json or {}
    enabled = bool(data.get('enabled', False))
    config  = load_config()
    config['autostart_enabled'] = enabled
    save_config(config)
    return jsonify({'success': True, 'autostart_enabled': enabled})


# --- Server Runner ---

def run_server(host='0.0.0.0', port=5000, debug=False):
    app.run(host=host, port=port, debug=debug, use_reloader=False)


def start_server_background(host='0.0.0.0', port=5000):
    t = threading.Thread(target=run_server, args=(host, port, False), daemon=True)
    t.start()
    return t


if __name__ == '__main__':
    run_server(debug=True)
