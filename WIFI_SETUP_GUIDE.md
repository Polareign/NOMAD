### Files Created:
1. **config.json** - Stores all destinations and flight parameters (dynamically updated via web)
2. **web_server.py** - Flask server with REST API endpoints for managing drone config
3. **templates/index.html** - Beautiful web interface accessible from iPhone Safari

### Files Modified:
1. **requirements.txt** - Added Flask dependency
2. **drone_control.py** - Integrated web server and config file loading

## What You Need to Do

### Step 1: Install Dependencies
```bash
source venv/bin/activate
pip install -r requirements.txt
```

### Step 2: Run the Drone
```bash
python drone_control.py
```

This will:
- Start the Flask web server on port 5000
- Load destinations and parameters from config.json
- Run the normal drone control system
- Print: "Access the web interface at: http://<your-raspberry-pi-ip>:5000"

### Step 3: Access from iPhone

#### On the Same Network:
1. Connect your Raspberry Pi and iPhone to the same WiFi network
2. Find your Pi's IP address:
   ```bash
   hostname -I
   ```
   (It will show something like `192.168.1.100`)

3. On iPhone Safari, go to:
   ```
   http://192.168.1.100:5000
   ```
   (Replace the IP with your actual Pi IP)

4. You'll see a beautiful interface with three tabs:
   - **Destinations** - Add/edit/delete flight destinations
   - **Home** - Set home location (can use iPhone's current GPS!)
   - **Parameters** - Adjust flight parameters (altitude, speed, etc.)

### Step 4: Use the Interface

#### Destinations Tab:
- View all current destinations
- Add new destinations by entering ID, name, description, latitude, longitude
- Delete destinations (except home)
- Each change is saved immediately to config.json

#### Home Tab:
- View current home location
- Enter new coordinates OR use "Use Current Location" button to set home to iPhone's GPS
- Very useful when you want to change home location in the field!

#### Parameters Tab:
- Adjust flight parameters in real-time:
  - Takeoff altitude
  - Flight speed
  - Yaw sensitivity
  - Obstacle turn rate
  - Confidence threshold
  - NMS threshold
- All changes are saved to config.json

### Step 5: How It Works

```
iPhone (Safari) ──WiFi──> Raspberry Pi (Flask Server)
                             │
                             └──> Reads/Updates config.json
                             └──> drone_control.py reloads config
```

When you update settings on the iPhone:
1. Data is sent via HTTP to the Flask server
2. Settings are saved to config.json
3. drone_control.py automatically reloads the config
4. Drone uses new settings on next flight

## Hotspot / Captive Portal Setup
If you want the Raspberry Pi to act as its own Wi-Fi hotspot, use the setup script with the --hotspot flag.

### Setup
1. Run the setup script with hotspot mode:
   ```bash
   sudo bash setup.sh --hotspot
   ```
2. Start the drone web server:
   ```bash
   source venv/bin/activate
   python drone_control.py
   ```
3. Connect your device to the SSID `NOMAD-Drone` with password `dronecontrol`.
4. Open the browser to:
   ```
   http://192.168.4.1:5000/
   ```

### Test captive portal routing
Use the test script with hotspot mode to verify the Flask routes for captive portals:
```bash
python test_setup.py --hotspot
```

If the automatic redirect does not work on a device, manually navigate to the Pi IP above.