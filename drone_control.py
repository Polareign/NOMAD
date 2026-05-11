import argparse
import json
import logging
import math
import os
import sys
import time
import threading
 
import cv2
import numpy as np
import SkeletonYolo as sy
from web_server import start_server_background, drone_state
 
try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None
 
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
 
try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        smbus = None
 
# Constants
DEFAULT_CONFIG_FILE   = 'config.json'
DEFAULT_SERVER_PORT   = 5000
HEARTBEAT_TIMEOUT     = 15   # Number of seconds if web client silent this long, auto-land
HMC5883L_ADDR         = 0x1E # HMC5883L I2C address (NOT 0x0D which was QMC5883L)
COMPASS_POLL_RATE     = 0.05 # seconds between compass reads
MODE_STABILIZE = 0
MODE_GUIDED    = 4
MODE_LAND      = 9
 
# Logger
logger = logging.getLogger('nomad')
logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
logger.addHandler(_sh)
_fh = logging.FileHandler('nomad_flight.log')
_fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
logger.addHandler(_fh)
 
 
# Args
def parse_args():
    p = argparse.ArgumentParser(description='NOMAD drone control — ArduPilot')
    p.add_argument('--dry-run',   action='store_true', help='No hardware, web server + config only')
    p.add_argument('--test',      action='store_true', help='Hardware on but no arm/takeoff; tests camera, compass, MAVLink')
    p.add_argument('--no-server', action='store_true', help='Skip Flask web server')
    p.add_argument('--preview',   action='store_true', help='Show camera window (requires DISPLAY)')
    p.add_argument('--port',      type=int, default=DEFAULT_SERVER_PORT)
    p.add_argument('--config',    default=DEFAULT_CONFIG_FILE)
    return p.parse_args()
 
 
# Config
def load_config(config_file=DEFAULT_CONFIG_FILE):
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError as exc:
            logger.warning('Bad config JSON: %s', exc)
 
    from destinations import (DESTINATIONS, TAKEOFF_ALTITUDE, FLIGHT_SPEED,
                               YAW_SENSITIVITY, OBSTACLE_TURN_RATE, CONFIDENCE_THRESHOLD)
    return {
        'destinations': {k: {'latitude': v[0], 'longitude': v[1]} for k, v in DESTINATIONS.items()},
        'flight_parameters': {
            'takeoff_altitude':  TAKEOFF_ALTITUDE,
            'flight_speed':      FLIGHT_SPEED,
            'yaw_sensitivity':   YAW_SENSITIVITY,
            'obstacle_turn_rate':OBSTACLE_TURN_RATE,
            'confidence_threshold': CONFIDENCE_THRESHOLD,
        },
        'obstacle_objects': [],
        'autostart_enabled': False,
    }
 
 
# Hardware Initialization
def init_camera(dry_run=False):
    if dry_run or Picamera2 is None:
        logger.info('Camera: skipped (%s)', 'dry-run' if dry_run else 'not installed')
        return None
    try:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}))
        cam.start()
        logger.info('Camera: OK')
        return cam
    except Exception as exc:
        logger.warning('Camera init failed: %s', exc)
        return None
 
 
def init_mavlink(dry_run=False):
    """
    Connect to ArduPilot on the SpeedyBee F405 V5 via UART6 → Pi GPIO UART.
    Wire: FC TX6 → Pi GPIO 15 (RXD), FC RX6 → Pi GPIO 14 (TXD), FC GND → Pi GND.
    In Mission Planner set SERIAL6_PROTOCOL=2 (MAVLink2), SERIAL6_BAUD=57.
    """
    if dry_run or mavutil is None:
        logger.info('MAVLink: skipped')
        return None
    try:
        # /dev/ttyAMA0 = Pi hardware UART (GPIO 14/15); baud matches SERIAL6_BAUD=57 → 57600
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        master.wait_heartbeat(timeout=10)
        logger.info('MAVLink: connected (sys=%d comp=%d)', master.target_system, master.target_component)
        return master
    except Exception as exc:
        logger.warning('MAVLink init failed: %s', exc)
        return None
 
 
def init_compass(dry_run=False):
    """
    HMC5883L on the DY-880 uses I2C address 0x1E (NOT 0x0D which is QMC5883L).
    Wire: GPS SDA → Pi GPIO 2 (SDA1), GPS SCL → Pi GPIO 3 (SCL1).
    The compass is read by ArduPilot directly via I2C; the Pi reads it independently
    for bearing calculations.
    """
    if dry_run or smbus is None:
        logger.info('Compass: skipped')
        return None
    try:
        bus = smbus.SMBus(1)
        bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)
        bus.write_byte_data(HMC5883L_ADDR, 0x01, 0x20)
        bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)
        time.sleep(0.1)
        logger.info('Compass HMC5883L: OK (addr=0x1E)')
        return bus
    except Exception as exc:
        logger.warning('Compass init failed: %s', exc)
        return None
 
 
def read_compass(bus):
    """
    HMC5883L data register order: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
    Note: register order is X, Z, Y — not X, Y, Z. This is per the datasheet.
    """
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    x = (data[0] << 8) | data[1]
    z = (data[2] << 8) | data[3]
    y = (data[4] << 8) | data[5]
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536
    heading = math.atan2(y, x)
    if heading < 0:
        heading += 2 * math.pi
    return math.degrees(heading)
 
 
# ArduPilot Flight Commands
def set_mode(master, mode_id):
    if master is None:
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    logger.info('Flight mode set to %d', mode_id)
 
 
def arm_drone(master, force=False):
    if master is None:
        return False
    force_param = 21196 if force else 0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, force_param, 0, 0, 0, 0, 0)
    for _ in range(10):
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            logger.info('Drone ARMED')
            return True
        time.sleep(0.5)
    logger.warning('Arm confirmation not received')
    return False
 
 
def disarm_drone(master):
    if master is None:
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    logger.info('Drone DISARMED')
 
 
def takeoff(master, altitude):
    if master is None:
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    logger.info('Takeoff to %.1f m', altitude)
 
 
def land(master):
    if master is None:
        return
    set_mode(master, MODE_LAND)
    logger.info('LAND mode activated')
 
 
def emergency_stop(master):
    """
    Immediate safe landing sequence:
    1. Zero all velocity
    2. Switch to LAND mode
    3. Disarm after landing (handled by ArduPilot auto-disarm)
    """
    if master is None:
        logger.error('EMERGENCY STOP — no MAVLink connection!')
        return
    logger.critical('EMERGENCY STOP TRIGGERED')
    # Zero velocity immediately
    set_velocity(master, 0, 0, 0, 0)
    time.sleep(0.1)
    set_velocity(master, 0, 0, 0, 0)
    set_mode(master, MODE_LAND)
 
 
def set_velocity(master, vx, vy, vz, yaw_rate):
    """Send velocity setpoint in body frame (NED). vz positive = down."""
    if master is None:
        return
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,   # ignore pos & accel, use velocity & yaw rate
        0, 0, 0,              # position
        vx, vy, vz,           # velocity m/s
        0, 0, 0,              # acceleration
        yaw_rate, 0)
 
 
def get_gps(master):
    """Returns (lat, lon) or None if unavailable."""
    if master is None:
        return None
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if not msg:
        return None
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    if lat == 0.0 and lon == 0.0:
        return None
    return (lat, lon)
 
 
def get_battery_voltage(master):
    """Returns battery voltage in volts, or None."""
    if master is None:
        return None
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
    if not msg:
        return None
    return msg.voltage_battery / 1000.0
 
 
# Safety Checks
def is_obstacle_detected(detected_list, obstacle_objects):
    dl = [o.lower() for o in detected_list]
    ol = [o.lower() for o in obstacle_objects]
    return any(o in ol for o in dl)
 
 
def check_geofence(current_pos, home_pos, max_radius_m=500):
    """Returns True if drone is outside the geofence radius."""
    if current_pos is None or home_pos is None:
        return False
    dlat = current_pos[0] - home_pos[0]
    dlon = current_pos[1] - home_pos[1]
    dist = math.sqrt((dlat * 111320)**2 + (dlon * 111320 * math.cos(math.radians(home_pos[0])))**2)
    if dist > max_radius_m:
        logger.warning('GEOFENCE BREACH: %.0f m from home (limit %d m)', dist, max_radius_m)
        return True
    return False
 
 
def check_emergency_flag():
    """Check the shared state set by the web server emergency-stop endpoint."""
    return drone_state.get('emergency_stop', False)
 
 
def check_heartbeat_timeout():
    """Returns True if web client hasn't pinged in HEARTBEAT_TIMEOUT seconds."""
    last = drone_state.get('last_heartbeat', None)
    if last is None:
        return False
    return (time.time() - last) > HEARTBEAT_TIMEOUT
 
 
# Navigation
def navigate_to_destination(master, bus, picam2, yolo, obstacle_objects,
                             flight_speed, yaw_sensitivity, obstacle_turn_rate,
                             target_lat, target_lon,
                             home_pos=None, show_preview=False):
    if master is None or picam2 is None:
        logger.warning('Cannot navigate: missing MAVLink or camera')
        return 'abort'
 
    logger.info('Navigating to (%.6f, %.6f)', target_lat, target_lon)
 
    while True:
        # Safety Checks
        if check_emergency_flag():
            logger.critical('Emergency stop during navigation')
            return 'emergency'
 
        if check_heartbeat_timeout():
            logger.warning('Controller heartbeat lost — returning home')
            return 'lost_link'
 
        pos = get_gps(master)
        if pos is None:
            logger.warning('GPS unavailable, hovering...')
            set_velocity(master, 0, 0, 0, 0)
            time.sleep(1)
            continue
 
        if check_geofence(pos, home_pos):
            logger.warning('Geofence breach — landing')
            return 'geofence'
 
        # Arrival Check
        dlat = target_lat - pos[0]
        dlon = target_lon - pos[1]
        dist_m = math.sqrt((dlat * 111320)**2 +
                           (dlon * 111320 * math.cos(math.radians(pos[0])))**2)
        if dist_m < 2.0:
            logger.info('Destination reached')
            set_velocity(master, 0, 0, 0, 0)
            return 'arrived'
 
        # Bearing & Heading
        bearing = math.degrees(math.atan2(dlon, dlat))
        if bus:
            try:
                current_heading = read_compass(bus)
            except Exception:
                current_heading = bearing
        else:
            current_heading = bearing
 
        yaw_diff = bearing - current_heading
        if yaw_diff >  180: yaw_diff -= 360
        if yaw_diff < -180: yaw_diff += 360
 
        # Obstacle Detection
        img = picam2.capture_array()
        detected = yolo.findObjects(img)
        obstacle = is_obstacle_detected(detected, obstacle_objects)
 
        if obstacle:
            set_velocity(master, 0, 0, 0, obstacle_turn_rate)
        else:
            set_velocity(master, flight_speed, 0, 0, yaw_diff * yaw_sensitivity)
 
        if show_preview and os.environ.get('DISPLAY'):
            cv2.imshow('NOMAD', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
 
        time.sleep(0.1)
 
    return 'interrupted'
 
 
# Test Mode
def run_test_mode(master, bus, picam2, yolo, obstacle_objects, args):
    """
    Hardware-on test mode — no arming or takeoff.
    Validates camera, compass, MAVLink comms, YOLO detection, and web server.
    Press Ctrl+C to exit.
    """
    logger.info('=' * 50)
    logger.info('TEST MODE — hardware active, motors will NOT spin')
    logger.info('=' * 50)
 
    drone_state['mode'] = 'test'
 
    # MAVLink
    if master:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            logger.info('[TEST] MAVLink heartbeat OK — firmware type %d', msg.autopilot)
        else:
            logger.warning('[TEST] MAVLink heartbeat timeout')
 
        # Battery
        v = get_battery_voltage(master)
        if v:
            logger.info('[TEST] Battery: %.2f V', v)
 
        # GPS
        pos = get_gps(master)
        if pos:
            logger.info('[TEST] GPS fix: %.6f, %.6f', pos[0], pos[1])
        else:
            logger.warning('[TEST] No GPS fix yet')
    else:
        logger.warning('[TEST] MAVLink not available')
 
    # Compass
    if bus:
        try:
            hdg = read_compass(bus)
            logger.info('[TEST] Compass heading: %.1f°', hdg)
        except Exception as exc:
            logger.warning('[TEST] Compass read failed: %s', exc)
    else:
        logger.warning('[TEST] Compass (smbus) not available')
 
    # Camera & YOLO Loop
    logger.info('[TEST] Starting camera + YOLO loop (Ctrl+C to stop)...')
    frame_count = 0
    try:
        while True:
            if check_emergency_flag():
                logger.info('[TEST] Emergency stop received via web UI')
                drone_state['emergency_stop'] = False
                break
 
            if picam2:
                img = picam2.capture_array()
                detected = yolo.findObjects(img)
                if detected:
                    logger.info('[TEST] Frame %d — detected: %s', frame_count, detected)
                if args.preview and os.environ.get('DISPLAY'):
                    cv2.imshow('NOMAD TEST', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
 
            if bus:
                try:
                    hdg = read_compass(bus)
                    if frame_count % 20 == 0:
                        logger.info('[TEST] Compass: %.1f°', hdg)
                except Exception:
                    pass
 
            frame_count += 1
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info('[TEST] Stopped by user')
    finally:
        drone_state['mode'] = 'idle'
        if args.preview and os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()
 
 
# Main
def run_main():
    args = parse_args()
    config_data = load_config(args.config)
 
    destinations = {
        k: (v['latitude'], v['longitude'])
        for k, v in config_data.get('destinations', {}).items()
    }
    params          = config_data.get('flight_parameters', {})
    obstacle_objects = config_data.get('obstacle_objects', [])
 
    takeoff_altitude  = float(params.get('takeoff_altitude',   5.0))
    flight_speed      = float(params.get('flight_speed',       1.0))
    yaw_sensitivity   = float(params.get('yaw_sensitivity',    0.01))
    obstacle_turn_rate= float(params.get('obstacle_turn_rate', 0.5))
    confidence_threshold = float(params.get('confidence_threshold', 0.5))
 
    low_battery_v     = float(params.get('low_battery_voltage', 21.0))
 
    home_dest = destinations.get('home')
    home_pos  = (home_dest[0], home_dest[1]) if home_dest else None
 
    # Start Web Server
    if not args.no_server:
        logger.info('Starting web server on port %d', args.port)
        start_server_background('0.0.0.0', args.port)
 
    # Initilize YOLO
    yolo = sy.SkeletonYolo(
        confThreshold=confidence_threshold,
        classes=obstacle_objects if obstacle_objects else None,
    )
 
    # Dry Run Mode
    if args.dry_run:
        logger.info('DRY RUN — no hardware. Web server available at http://<pi-ip>:%d', args.port)
        logger.info('Destinations loaded: %s', list(destinations.keys()))
        drone_state['mode'] = 'dry_run'
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('Dry run ended')
        return
 
    # Hardware Inilization
    picam2 = init_camera()
    master = init_mavlink()
    bus    = init_compass()
 
    # Test Mode
    if args.test:
        run_test_mode(master, bus, picam2, yolo, obstacle_objects, args)
        if picam2: picam2.stop()
        return
 
    # Full Flight Mode
    if picam2 is None or master is None:
        logger.error('Hardware init failed. Use --dry-run or --test to proceed without all hardware.')
        return
 
    drone_state['mode'] = 'preflight'
 
    # Wait For Web Controller To connect
    logger.info('Waiting for web controller connection...')
    logger.info('Open http://<pi-ip>:%d and press LAUNCH when ready', args.port)
    while not drone_state.get('launch_authorized', False):
        if check_emergency_flag():
            logger.info('Emergency stop set before launch — aborting')
            return
        time.sleep(0.5)
 
    logger.info('Launch authorized by web controller')
 
    set_mode(master, MODE_GUIDED)
    time.sleep(0.5)
 
    armed = arm_drone(master, force=False)
    if not armed:
        logger.error('Arming failed — aborting flight')
        drone_state['mode'] = 'idle'
        return
 
    drone_state['mode'] = 'airborne'
    drone_state['last_heartbeat'] = time.time()
 
    takeoff(master, takeoff_altitude)
    time.sleep(6)
 
    # Destination From Web UI
    dest_key = drone_state.get('destination', '').upper()
    if dest_key not in destinations:
        logger.warning('No valid destination set — landing')
        land(master)
        time.sleep(6)
        disarm_drone(master)
        drone_state['mode'] = 'idle'
        return
 
    target_lat, target_lon = destinations[dest_key]
 
    try:
        # Flight loop
        result = navigate_to_destination(
            master, bus, picam2, yolo, obstacle_objects,
            flight_speed, yaw_sensitivity, obstacle_turn_rate,
            target_lat, target_lon,
            home_pos=home_pos,
            show_preview=args.preview,
        )
 
        # Handle Navigation Result
        if result == 'emergency':
            emergency_stop(master)
        elif result in ('lost_link', 'geofence'):
            logger.warning('Safety landing triggered by: %s', result)
            land(master)
        elif result == 'arrived':
            logger.info('Mission complete — landing')
            land(master)
        else:
            land(master)
 
        # Battery Monitor After Landing
        v = get_battery_voltage(master)
        if v:
            logger.info('Post-flight battery: %.2f V', v)
            if v < low_battery_v:
                logger.warning('LOW BATTERY: %.2f V — charge before next flight', v)
 
    except KeyboardInterrupt:
        logger.info('Ctrl+C received — initiating landing')
        land(master)
 
    finally:
        time.sleep(6)
        disarm_drone(master)
        drone_state['mode'] = 'idle'
        drone_state['emergency_stop'] = False
        drone_state['launch_authorized'] = False
        if picam2:
            picam2.stop()
        if args.preview and os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()
        logger.info('Flight session ended')
 
 
if __name__ == '__main__':
    run_main()