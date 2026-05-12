# Custom Drone Autonomous Flight Setup

Autonomous drone control for a drone. This project implements obstacle avoidance, image recognition, and autonomous GPS flight for a custom drone using Raspberry Pi 4B, SpeedyBee F405 V5 flight controller with ArduPilot, GPS module, DY-880 GPS with HMC5883L compass, and Raspberry Pi Camera Module 3.

## Overview
This project runs a web enabled companion computer controller that launches and monitors an ArduPilot-powered drone. The Pi hosts a web UI, camera/YOLO obstacle detection, compass/GPS checks, and a launch flow that waits for web authorization before arming and taking off.

## Hardware
- Raspberry Pi 4 Model B
- DY-880 GPS module with HMC5883L compass
- Raspberry Pi Camera Module 3
- 6S LiPo battery
- SpeedyBee F405 V5 stack with BLS 55A 4in1 ESC

## Firmware
- ArduPilot Copter stable firmware for SpeedyBee F405 V5
- Companion computer mode via SERIAL2 on UART2

## Connections
- GPS/Compass SDA, SCL, VCC, GND, RTX & TXD Into Flight Controller G, T4, R4, CL, DA, 4V5
-Flightcontroller R2, T2 & G into Pi Pin's 14,15 & GND

## Requirements
- Python libraries: `opencv-python`, `numpy`, `picamera2`, `pymavlink`, `Flask`, `ultralytics`, `torch`, `smbus2`
- `picamera2` compatible camera support
- `pymavlink` for MAVLink communication
- `smbus2` for I2C compass reads

## Setup

### Quick Start
```bash
bash setup.sh
source venv/bin/activate
python test_setup.py
python drone_control.py --dry-run
```

### Manual Setup
```bash
sudo apt-get update && sudo apt-get install -y python3-pip python3-venv python3-dev build-essential i2c-tools libopenjp2-7 libopenjp2-7-dev libssl-dev
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install --no-cache-dir -r requirements.txt
```

## Systemd Auto-Start Service
The `nomad.service` file is designed to run `drone_control.py` from `/home/pi/nomad` using the virtual environment at `/home/pi/nomad/venv`.

### Install service
```bash
sudo cp nomad.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now nomad
sudo systemctl status nomad
```

### Service behavior
- Waits for `network-online.target` before starting
- Sleeps 5 seconds before running the script to allow hardware to settle
- Starts the web server and companion controller automatically
- Does not automatically launch the drone; it waits for the web UI `Launch` authorization
- Logs are available via `journalctl -u nomad -f`

> If your installation path is not `/home/pi/nomad`, adjust `WorkingDirectory` and `ExecStart` in `nomad.service` accordingly.

## Running the Code
### Dry run
```bash
python drone_control.py --dry-run
```

### Test mode
```bash
python drone_control.py --test --preview
```

### Comprehensive test
```bash
python drone_control.py --comprehensive-test --preview
```

Runs all available tests including system resources, network, USB devices, GPIO status, extended flight controller diagnostics (IMU, barometer, vibration, radio), web server status, and log file checks.

### Motor test
```bash
python drone_control.py --motor-test
```

The script connects to the flight controller and spins all motors at 40% power for 2 seconds. **Remove props first!** It will attempt to auto-arm the drone if possible.

### Normal run
```bash
python drone_control.py
```

The script now starts the Flask web UI and waits for a controller to press `Launch` before arming.

## Command Line Options
- `--dry-run`: No hardware interaction, web server + config only
- `--test`: Hardware enabled but no arm/takeoff; tests camera, compass, MAVLink
- `--comprehensive-test`: Run all available hardware and system tests (extended diagnostics)
- `--motor-test`: Spin all motors for 2 seconds at 40% power with auto-arm if available
- `--no-server`: Skip Flask web server
- `--preview`: Show camera window (requires DISPLAY)
- `--port <number>`: Set web server port (default: 5000)
- `--config <file>`: Specify config file (default: config.json)

## Hardware Tests
- Camera: `python -c "from picamera2 import Picamera2; p=Picamera2(); p.start(); print('Camera OK'); p.stop()"`
- MAVLink: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); m.wait_heartbeat(); print('MAVLink OK')"`
- Compass/I2C: `python -c "import smbus2 as smbus; bus=smbus.SMBus(1); print('I2C OK')"`
- Serial connection: `ls -l /dev/serial*` (should show serial0)
- I2C devices: `i2cdetect -y 1` (should show 1e for HMC5883L)
- GPS: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); msg=m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5); print('GPS sats:', msg.satellites_visible if msg else 'No GPS')"`
- Motor test (props removed): `python drone_control.py --motor-test` (requires armed drone)
- System resources: `python -c "import psutil; print(f'CPU: {psutil.cpu_percent()}%, RAM: {psutil.virtual_memory().percent}%')"`
- CPU temperature: `vcgencmd measure_temp` or `cat /sys/class/thermal/thermal_zone0/temp`
- Network: `ip route` (should show default route)
- USB devices: `lsusb` (should show connected devices)
- GPIO pins: `gpio readall` (shows pin states)
- RTC: `hwclock -r` (shows real-time clock)
- IMU data: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); msg=m.recv_match(type='RAW_IMU', blocking=True, timeout=5); print('IMU OK' if msg else 'No IMU')"`
- Barometer: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); msg=m.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=5); print(f'Pressure: {msg.press_abs} hPa' if msg else 'No baro')"`
- Vibration: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); msg=m.recv_match(type='VIBRATION', blocking=True, timeout=5); print('Vibration OK' if msg else 'No vibration data')"`
- Radio link: `python -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('/dev/serial0', baud=57600); msg=m.recv_match(type='RADIO_STATUS', blocking=True, timeout=5); print(f'RSSI: {msg.rssi}' if msg else 'No radio')"`
- Web server: `curl http://localhost:5000/status` (should return JSON status)
- Log files: `ls -lh nomad_flight.log` (check log file exists and size)
- Disk space: `df -h /` (check available disk space)
- Time sync: `timedatectl status` (check NTP synchronization)

## Notes
- The current software assumes HMC5883L on address `0x1E`.
- The web UI is required to authorize launch and provide destination selection.
- Props must be removed until the first full hardware test is complete.
