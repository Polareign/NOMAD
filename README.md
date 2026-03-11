# Custom Drone Autonomous Flight Code

This project implements obstacle avoidance, image recognition, and autonomous GPS flight for a custom drone using Raspberry Pi 4, SpeedyBee F405 V4 flight controller with iNAV, GPS module, HMC5883L compass, and Raspberry Pi Camera Module 3.

## Hardware
- Raspberry Pi 4 Model B 4GB
- SpeedyBee F405 V4 stack with BLS 55A 4in1 ESC
- GPS receiver module with active antenna
- HMC5883L compass module
- Raspberry Pi Camera Module 3 (12MP autofocus)

## Features
- Obstacle avoidance using YOLO object detection
- Autonomous navigation to predefined destinations
- Real-time GPS and compass data
- Image capture and processing with Raspberry Pi camera

## Requirements
- Python libraries: opencv-python, numpy, picamera2, pymavlink, smbus (for I2C compass)
- YOLOv3 files: obstacles.names, yolov3.cfg, yolov3.weights
- iNAV firmware on F405 flight controller

## Setup

### Quick Start (Recommended with Virtual Environment)
```bash
bash setup.sh
source venv/bin/activate
pip install -r requirements.txt
python test_setup.py
```

### Manual Setup
1. Install system dependencies:
   ```
   sudo apt-get update && sudo apt-get install -y python3-pip python3-venv build-essential
   ```

2. Create and activate virtual environment:
   ```
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install Python dependencies:
   ```
   pip install --upgrade pip setuptools wheel
   pip install -r requirements.txt
   ```

4. Test the setup:
   ```
   python test_setup.py
   ```

5. Download YOLO Files:
   - yolov3.cfg: https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg
   - yolov3.weights: https://pjreddie.com/media/files/yolov3.weights

6. Connect hardware:
   - GPS to flight controller UART
   - HMC5883L to Raspberry Pi I2C
   - Configure iNAV for MAVLink serial passthrough

## Running the Code
```
python drone_control.py
```

**WARNING**: Only run with props off first! Ensure all systems are working before attaching propellers.

## Easy Configuration

### Changing Destinations
Edit `destinations.py` to add or modify flight destinations:

```python
DESTINATIONS = {
    'A': (37.7749, -122.4194),      # San Francisco
    'B': (34.0522, -118.2437),      # Los Angeles
    'home': (40.7128, -74.0060),    # Your location
}
```

### Flight Parameters
Modify these settings in `destinations.py`:
- `TAKEOFF_ALTITUDE` = flight height in meters
- `FLIGHT_SPEED` = forward velocity
- `OBSTACLE_TURN_RATE` = rotation speed
- `OBSTACLE_OBJECTS` = objects to avoid (e.g., 'PERSON', 'CAR')

### Virtual Environment FAQ
- **Q: Do I need venv?** 
  - **A**: Recommended! It isolates dependencies and prevents conflicts with system Python.
- **Q: What's the difference between venv and venv-1?**
  - **A**: venv-1 doesn't exist by default. Use just `venv` as shown in setup.sh
- **Q: How to activate/deactivate?**
  - **A**: `source venv/bin/activate` to activate, `deactivate` to exit

## Pre-Flight Checklist

### Hardware Connections
- [ ] GPS module connected to F405 UART (configure in iNAV)
- [ ] HMC5883L compass connected to Raspberry Pi I2C pins (SDA: GPIO 2, SCL: GPIO 3)
- [ ] Raspberry Pi Camera Module 3 properly seated
- [ ] Flight controller powered and iNAV firmware flashed

### iNAV Configuration
- [ ] Enable MAVLink serial passthrough
- [ ] Configure GPS settings
- [ ] Calibrate compass and accelerometer
- [ ] Set up failsafes

### Safety Checks
- [ ] Verify propeller directions
- [ ] Test motor spin without props
- [ ] Confirm GPS lock
- [ ] Test compass calibration
- [ ] Ensure clear flight area

### Testing
- [ ] Run `python test_setup.py` to verify software
- [ ] Test camera: `python -c "from picamera2 import Picamera2; p=Picamera2(); p.start(); print('Camera OK'); p.stop()"`
- [ ] Test MAVLink: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200); m.wait_heartbeat(); print('MAVLink OK')"`
- [ ] Test compass: `python -c "import smbus; bus=smbus.SMBus(1); print('I2C OK')"`

## Running the Code
```
python drone_control.py
```

**WARNING**: Only run with props off first! Ensure all systems are working before attaching propellers.

## Usage
- Power on the drone
- Run the script
- Enter destination when prompted
- Drone will arm, takeoff, avoid obstacles, and navigate to destination