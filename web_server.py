"""
Web Server for Drone Configuration
Provides REST API and web interface for updating drone destinations and flight parameters
"""

from flask import Flask, render_template, jsonify, request, send_from_directory
import json
import os
from threading import Thread

app = Flask(__name__, static_folder='static', template_folder='templates')

CONFIG_FILE = 'config.json'

def load_config():
    """Load configuration from JSON file"""
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_config(config):
    """Save configuration to JSON file"""
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)

@app.route('/')
def index():
    """Serve the main web interface"""
    return render_template('index.html')

@app.route('/api/destinations', methods=['GET'])
def get_destinations():
    """Get all destinations"""
    config = load_config()
    destinations = config.get('destinations', {})
    return jsonify(destinations)

@app.route('/api/destinations/<dest_id>', methods=['GET'])
def get_destination(dest_id):
    """Get a specific destination"""
    config = load_config()
    destination = config.get('destinations', {}).get(dest_id)
    if destination:
        return jsonify(destination)
    return jsonify({'error': 'Destination not found'}), 404

@app.route('/api/destinations', methods=['POST'])
def create_destination():
    """Create or update a destination"""
    data = request.json
    dest_id = data.get('id')
    
    if not dest_id:
        return jsonify({'error': 'Destination ID required'}), 400
    
    config = load_config()
    if 'destinations' not in config:
        config['destinations'] = {}
    
    config['destinations'][dest_id] = {
        'name': data.get('name', dest_id),
        'latitude': float(data.get('latitude', 0)),
        'longitude': float(data.get('longitude', 0)),
        'description': data.get('description', '')
    }
    
    save_config(config)
    return jsonify({'success': True, 'message': f'Destination {dest_id} saved'}), 201

@app.route('/api/destinations/<dest_id>', methods=['DELETE'])
def delete_destination(dest_id):
    """Delete a destination"""
    if dest_id == 'home':
        return jsonify({'error': 'Cannot delete home location'}), 403
    
    config = load_config()
    if 'destinations' in config and dest_id in config['destinations']:
        del config['destinations'][dest_id]
        save_config(config)
        return jsonify({'success': True, 'message': f'Destination {dest_id} deleted'})
    
    return jsonify({'error': 'Destination not found'}), 404

@app.route('/api/home', methods=['GET'])
def get_home():
    """Get home location"""
    config = load_config()
    home = config.get('destinations', {}).get('home', {})
    return jsonify(home)

@app.route('/api/home', methods=['POST'])
def update_home():
    """Update home location"""
    data = request.json
    config = load_config()
    
    if 'destinations' not in config:
        config['destinations'] = {}
    
    config['destinations']['home'] = {
        'name': 'Home',
        'latitude': float(data.get('latitude', 0)),
        'longitude': float(data.get('longitude', 0)),
        'description': 'Home Location'
    }
    
    save_config(config)
    return jsonify({'success': True, 'message': 'Home location updated'}), 201

@app.route('/api/flight-parameters', methods=['GET'])
def get_flight_parameters():
    """Get flight parameters"""
    config = load_config()
    params = config.get('flight_parameters', {})
    return jsonify(params)

@app.route('/api/flight-parameters', methods=['POST'])
def update_flight_parameters():
    """Update flight parameters"""
    data = request.json
    config = load_config()
    
    if 'flight_parameters' not in config:
        config['flight_parameters'] = {}
    
    for key in ['takeoff_altitude', 'flight_speed', 'yaw_sensitivity', 'obstacle_turn_rate', 'confidence_threshold']:
        if key in data:
            config['flight_parameters'][key] = float(data[key])
    
    save_config(config)
    return jsonify({'success': True, 'message': 'Flight parameters updated'}), 201

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get system status"""
    config = load_config()
    return jsonify({
        'status': 'ready',
        'config_loaded': True,
        'destinations_count': len(config.get('destinations', {})),
        'flight_parameters_set': len(config.get('flight_parameters', {})) > 0
    })

def run_server(host='0.0.0.0', port=5000, debug=False):
    """Run the Flask server"""
    app.run(host=host, port=port, debug=debug, use_reloader=False)

def start_server_background(host='0.0.0.0', port=5000):
    """Start the server in a background thread"""
    server_thread = Thread(target=run_server, args=(host, port, False), daemon=True)
    server_thread.start()
    return server_thread

if __name__ == '__main__':
    run_server(debug=True)