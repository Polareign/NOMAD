#!/usr/bin/env python3
"""Main entrypoint for NOMAD drone control."""

import argparse
import json
import logging
import math
import os
import sys
import time

import cv2
import numpy as np
import SkeletonYolo as sy
from web_server import start_server_background

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    import smbus
except ImportError:
    smbus = None

DEFAULT_CONFIG_FILE = 'config.json'
DEFAULT_CLASSES_FILE = 'obstacles.names'
DEFAULT_MODEL_CFG = 'yolov3.cfg'
DEFAULT_MODEL_WEIGHTS = 'yolov3.weights'
DEFAULT_SERVER_PORT = 5000

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
logger.addHandler(handler)


def parse_args():
    parser = argparse.ArgumentParser(description='NOMAD drone control runner')
    parser.add_argument('--dry-run', action='store_true', help='Run without camera, MAVLink, or compass hardware')
    parser.add_argument('--no-server', action='store_true', help='Do not start the Flask web server')
    parser.add_argument('--preview', action='store_true', help='Show camera preview window if DISPLAY is available')
    parser.add_argument('--port', type=int, default=DEFAULT_SERVER_PORT, help='Flask server port')
    return parser.parse_args()


def load_config(config_file=DEFAULT_CONFIG_FILE):
    """Load configuration from JSON file with fallback to destinations.py."""
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError as exc:
            logger.warning('Could not parse %s: %s', config_file, exc)

    from destinations import DESTINATIONS, TAKEOFF_ALTITUDE, FLIGHT_SPEED, YAW_SENSITIVITY, OBSTACLE_TURN_RATE

    obstacle_objects = []
    if os.path.exists(DEFAULT_CLASSES_FILE):
        with open(DEFAULT_CLASSES_FILE, 'r') as f:
            obstacle_objects = [line.strip() for line in f if line.strip()]

    return {
        'destinations': {k: {'latitude': v[0], 'longitude': v[1]} for k, v in DESTINATIONS.items()},
        'flight_parameters': {
            'takeoff_altitude': TAKEOFF_ALTITUDE,
            'flight_speed': FLIGHT_SPEED,
            'yaw_sensitivity': YAW_SENSITIVITY,
            'obstacle_turn_rate': OBSTACLE_TURN_RATE,
            'confidence_threshold': 0.5,
            'nms_threshold': 0.3,
        },
        'obstacle_objects': obstacle_objects,
    }


def is_display_available():
    return bool(os.environ.get('DISPLAY') or sys.platform.startswith('win'))


def init_camera(dry_run=False):
    if dry_run:
        logger.info('Dry run: skipping camera initialization')
        return None
    if Picamera2 is None:
        logger.warning('Picamera2 is not installed. Camera unavailable.')
        return None

    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration()
        picam2.configure(config)
        picam2.start()
        return picam2
    except Exception as exc:
        logger.warning('Failed to initialize Picamera2: %s', exc)
        return None


def init_mavlink(dry_run=False):
    if dry_run:
        logger.info('Dry run: skipping MAVLink initialization')
        return None
    if mavutil is None:
        logger.warning('pymavlink is not installed. MAVLink unavailable.')
        return None

    try:
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        return master
    except Exception as exc:
        logger.warning('Failed to connect to MAVLink: %s', exc)
        return None


def init_compass(dry_run=False):
    if dry_run:
        logger.info('Dry run: skipping compass initialization')
        return False
    if smbus is None:
        logger.warning('smbus is not installed. Compass unavailable.')
        return False

    try:
        bus = smbus.SMBus(1)
        address = 0x1E
        bus.write_byte_data(address, 0x00, 0x70)
        bus.write_byte_data(address, 0x01, 0xA0)
        bus.write_byte_data(address, 0x02, 0x00)
        return True
    except Exception as exc:
        logger.warning('Failed to initialize HMC5883L compass: %s', exc)
        return False


def read_compass(bus):
    address = 0x1E
    data = bus.read_i2c_block_data(address, 0x03, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]
    if x > 32767:
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767:
        z -= 65536
    heading = math.atan2(y, x)
    if heading < 0:
        heading += 2 * math.pi
    return math.degrees(heading)


def arm_drone(master):
    if master is None:
        logger.warning('Cannot arm: MAVLink master is not available.')
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    logger.info('Arming drone...')


def disarm_drone(master):
    if master is None:
        logger.warning('Cannot disarm: MAVLink master is not available.')
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    logger.info('Disarming drone...')


def takeoff(master, altitude):
    if master is None:
        logger.warning('Cannot takeoff: MAVLink master is not available.')
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    logger.info('Taking off to %.1f m...', altitude)


def land(master):
    if master is None:
        logger.warning('Cannot land: MAVLink master is not available.')
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    logger.info('Landing...')


def set_velocity(master, vx, vy, vz, yaw_rate):
    if master is None:
        logger.warning('Cannot set velocity: MAVLink master is not available.')
        return
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        yaw_rate, 0)


def get_gps(master):
    if master is None:
        return (0.0, 0.0)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    return (msg.lat / 1e7, msg.lon / 1e7) if msg else (0.0, 0.0)


def is_obstacle_detected(detected_list, obstacle_objects):
    detected_lower = [obj.lower() for obj in detected_list]
    obstacles_lower = [obj.lower() for obj in obstacle_objects]
    return any(obj in obstacles_lower for obj in detected_lower)


def navigate_to_destination(master, bus, picam2, yolo, obstacle_objects, takeoff_altitude, flight_speed, yaw_sensitivity, obstacle_turn_rate,
                             target_lat, target_lon, show_preview=False):
    if master is None or picam2 is None or bus is None:
        logger.warning('Cannot navigate: missing hardware path (MAVLink, camera, or compass).')
        return

    current_lat, current_lon = get_gps(master)
    while abs(current_lat - target_lat) >= 0.0001 or abs(current_lon - target_lon) >= 0.0001:
        current_lat, current_lon = get_gps(master)
        if current_lat == 0.0 and current_lon == 0.0:
            logger.warning('GPS data unavailable. Waiting for valid fix...')
            time.sleep(1)
            continue

        dlat = target_lat - current_lat
        dlon = target_lon - current_lon
        bearing = math.degrees(math.atan2(dlon, dlat))
        current_heading = read_compass(bus)
        yaw_diff = bearing - current_heading
        if yaw_diff > 180:
            yaw_diff -= 360
        elif yaw_diff < -180:
            yaw_diff += 360

        img = picam2.capture_array()
        detected = yolo.findObjects(img)
        obstacle_detected = is_obstacle_detected(detected, obstacle_objects)

        if obstacle_detected:
            set_velocity(master, 0, 0, 0, obstacle_turn_rate)
        else:
            set_velocity(master, flight_speed, 0, 0, yaw_diff * yaw_sensitivity)

        if show_preview and is_display_available():
            cv2.imshow('NOMAD Preview', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info('Preview closed by user.')
                break

        time.sleep(0.1)


def run_main():
    args = parse_args()
    config_data = load_config()

    destinations = {
        key: (value['latitude'], value['longitude'])
        for key, value in config_data.get('destinations', {}).items()
    }
    parameters = config_data.get('flight_parameters', {})
    obstacle_objects = config_data.get('obstacle_objects', [])

    takeoff_altitude = float(parameters.get('takeoff_altitude', 5))
    flight_speed = float(parameters.get('flight_speed', 1.0))
    yaw_sensitivity = float(parameters.get('yaw_sensitivity', 0.01))
    obstacle_turn_rate = float(parameters.get('obstacle_turn_rate', 0.5))
    confidence_threshold = float(parameters.get('confidence_threshold', 0.5))
    nms_threshold = float(parameters.get('nms_threshold', 0.3))

    if not args.no_server:
        logger.info('Starting web server on port %s', args.port)
        start_server_background('0.0.0.0', args.port)

    yolo = sy.SkeletonYolo(
        classesFile=DEFAULT_CLASSES_FILE,
        modelConfiguration=DEFAULT_MODEL_CFG,
        modelWeights=DEFAULT_MODEL_WEIGHTS,
        confThreshold=confidence_threshold,
        nmsThreshold=nms_threshold,
        classes=obstacle_objects if obstacle_objects else None,
    )

    if args.dry_run:
        logger.info('Dry run mode enabled. Runtime will not initialize drone hardware.')
        logger.info('Destinations: %s', list(destinations.keys()))
        if not destinations:
            logger.warning('No destinations configured. Add entries to config.json or destinations.py.')
        return

    picam2 = init_camera(dry_run=False)
    master = init_mavlink(dry_run=False)
    compass_ready = init_compass(dry_run=False)
    bus = smbus.SMBus(1) if smbus is not None else None

    if picam2 is None or master is None or not compass_ready or bus is None:
        logger.error('Hardware initialization failed. Use --dry-run to test software without hardware.')
        return

    try:
        master.wait_heartbeat()
        logger.info('Connected to flight controller')
    except Exception as exc:
        logger.warning('Could not get MAVLink heartbeat: %s', exc)

    arm_drone(master)
    time.sleep(2)
    takeoff(master, takeoff_altitude)
    time.sleep(5)

    try:
        while True:
            img = picam2.capture_array()
            detected_objects = yolo.findObjects(img)
            obstacle_detected = is_obstacle_detected(detected_objects, obstacle_objects)
            if obstacle_detected:
                set_velocity(master, 0, 0, 0, obstacle_turn_rate)
            else:
                set_velocity(master, 0, 0, 0, 0)

            if args.preview and is_display_available():
                cv2.imshow('NOMAD Preview', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logger.info('Preview closed by user.')
                    break

            time.sleep(0.1)

        logger.info('Available destinations: %s', list(destinations.keys()))
        destination = input('Enter destination: ').strip().upper()
        if destination in destinations:
            target_lat, target_lon = destinations[destination]
            logger.info('Navigating to %s (%s, %s)', destination, target_lat, target_lon)
            navigate_to_destination(
                master, bus, picam2, yolo, obstacle_objects,
                takeoff_altitude, flight_speed, yaw_sensitivity, obstacle_turn_rate,
                target_lat, target_lon, show_preview=args.preview,
            )
        else:
            logger.warning('Invalid destination. Landing now...')
    except KeyboardInterrupt:
        logger.info('Keyboard interrupt received. Landing...')
    finally:
        land(master)
        time.sleep(5)
        disarm_drone(master)
        if picam2 is not None:
            picam2.stop()
        if args.preview and is_display_available():
            cv2.destroyAllWindows()


if __name__ == '__main__':
    run_main()