import argparse
import json
import logging
import math
import os
import sys
import time
import threading
from dataclasses import dataclass

import cv2
import numpy as np
import SkeletonYolo as sy
from web_server import start_server_background, get_state, set_state

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

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DEFAULT_CONFIG_FILE  = 'config.json'
DEFAULT_SERVER_PORT  = 5000
HEARTBEAT_TIMEOUT    = 15    # seconds of web-client silence before auto-land
HMC5883L_ADDR        = 0x1E  # HMC5883L I2C address (NOT 0x0D = QMC5883L)
COMPASS_POLL_RATE    = 0.05  # seconds between compass reads
MODE_STABILIZE = 0
MODE_GUIDED    = 4
MODE_LAND      = 9

# Magnetic declination for Rolling Meadows, IL (degrees, positive = east)
# Update this if your launch site changes significantly.
# Look up your value at: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
MAGNETIC_DECLINATION = -2.5  # degrees west → negative

# Obstacle avoidance tuning
OBS_YAW_TIMEOUT    = 5.0   # seconds to yaw before trying to advance anyway
OBS_SIDESTEP_TIME  = 2.0   # seconds of sideways strafe to clear an obstacle
OBS_MIN_CLEAR_SECS = 1.0   # consecutive clear seconds required before resuming forward


# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------
logger = logging.getLogger('nomad')
logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
logger.addHandler(_sh)
_fh = logging.FileHandler('nomad_flight.log')
_fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
logger.addHandler(_fh)


# ---------------------------------------------------------------------------
# Flight config dataclass — replaces the 11-argument function signature
# ---------------------------------------------------------------------------
@dataclass
class FlightConfig:
    takeoff_altitude:   float = 5.0
    flight_speed:       float = 1.0
    yaw_sensitivity:    float = 0.01
    obstacle_turn_rate: float = 0.5
    confidence_threshold: float = 0.5
    low_battery_voltage: float = 21.0
    obstacle_objects:   list  = None
    home_pos:           tuple = None  # (lat, lon)

    def __post_init__(self):
        if self.obstacle_objects is None:
            self.obstacle_objects = []


# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description='NOMAD drone control — ArduPilot')
    p.add_argument('--dry-run',   action='store_true', help='No hardware, web server + config only')
    p.add_argument('--test',      action='store_true', help='Hardware on but no arm/takeoff')
    p.add_argument('--no-server', action='store_true', help='Skip Flask web server')
    p.add_argument('--preview',   action='store_true', help='Show camera window (requires DISPLAY)')
    p.add_argument('--port',      type=int, default=DEFAULT_SERVER_PORT)
    p.add_argument('--config',    default=DEFAULT_CONFIG_FILE)
    return p.parse_args()


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
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
            'takeoff_altitude':   TAKEOFF_ALTITUDE,
            'flight_speed':       FLIGHT_SPEED,
            'yaw_sensitivity':    YAW_SENSITIVITY,
            'obstacle_turn_rate': OBSTACLE_TURN_RATE,
            'confidence_threshold': CONFIDENCE_THRESHOLD,
        },
        'obstacle_objects': [],
        'autostart_enabled': False,
    }


def build_flight_config(config_data, destinations):
    params = config_data.get('flight_parameters', {})
    home_dest = destinations.get('home')
    return FlightConfig(
        takeoff_altitude   = float(params.get('takeoff_altitude',    5.0)),
        flight_speed       = float(params.get('flight_speed',        1.0)),
        yaw_sensitivity    = float(params.get('yaw_sensitivity',     0.01)),
        obstacle_turn_rate = float(params.get('obstacle_turn_rate',  0.5)),
        confidence_threshold = float(params.get('confidence_threshold', 0.5)),
        low_battery_voltage = float(params.get('low_battery_voltage', 21.0)),
        obstacle_objects   = config_data.get('obstacle_objects', []),
        home_pos           = (home_dest[0], home_dest[1]) if home_dest else None,
    )


# ---------------------------------------------------------------------------
# Hardware initialization
# ---------------------------------------------------------------------------
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
    In Mission Planner: SERIAL6_PROTOCOL=2 (MAVLink2), SERIAL6_BAUD=57.
    """
    if dry_run or mavutil is None:
        logger.info('MAVLink: skipped')
        return None
    try:
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        master.wait_heartbeat(timeout=10)
        logger.info('MAVLink: connected (sys=%d comp=%d)',
                    master.target_system, master.target_component)
        return master
    except Exception as exc:
        logger.warning('MAVLink init failed: %s', exc)
        return None


def init_compass(dry_run=False):
    """
    HMC5883L on the DY-880 uses I2C address 0x1E (NOT 0x0D which is QMC5883L).
    Wire: GPS SDA → Pi GPIO 2 (SDA1), GPS SCL → Pi GPIO 3 (SCL1).
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


# ---------------------------------------------------------------------------
# Compass
# ---------------------------------------------------------------------------
def read_compass(bus):
    """
    HMC5883L register order: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
    Note: register order is X, Z, Y — not X, Y, Z. This is per the datasheet.

    Returns magnetic heading corrected for local declination, in degrees [0, 360).
    """
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    x = (data[0] << 8) | data[1]
    z = (data[2] << 8) | data[3]
    y = (data[4] << 8) | data[5]
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    heading = math.degrees(math.atan2(y, x))
    # Apply magnetic declination to convert magnetic north → true north
    heading += MAGNETIC_DECLINATION
    heading %= 360.0
    return heading


# ---------------------------------------------------------------------------
# ArduPilot flight commands
# ---------------------------------------------------------------------------
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
    Immediate safe landing:
    1. Zero all velocity
    2. Switch to LAND mode
    ArduPilot handles auto-disarm after touchdown.
    """
    if master is None:
        logger.error('EMERGENCY STOP — no MAVLink connection!')
        return
    logger.critical('EMERGENCY STOP TRIGGERED')
    set_velocity_body(master, 0, 0, 0, 0)
    time.sleep(0.1)
    set_velocity_body(master, 0, 0, 0, 0)
    set_mode(master, MODE_LAND)


def set_velocity_body(master, vx, vy, vz, yaw_rate):
    """
    Send velocity setpoint in BODY frame (forward/right/down).
    vx = forward (m/s), vy = right (m/s), vz = down (m/s, positive = descend),
    yaw_rate = rotation rate (rad/s, positive = clockwise from above).

    Uses MAV_FRAME_BODY_NED so the command is always relative to where the
    drone is pointing — no heading math needed before sending.
    """
    if master is None:
        return
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,   # ignore pos & accel, use velocity + yaw rate
        0, 0, 0,              # position (ignored)
        vx, vy, vz,           # velocity m/s — body frame
        0, 0, 0,              # acceleration (ignored)
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


# ---------------------------------------------------------------------------
# Safety checks
# ---------------------------------------------------------------------------
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
    dist = math.sqrt((dlat * 111320)**2 +
                     (dlon * 111320 * math.cos(math.radians(home_pos[0])))**2)
    if dist > max_radius_m:
        logger.warning('GEOFENCE BREACH: %.0f m from home (limit %d m)', dist, max_radius_m)
        return True
    return False


def check_emergency_flag():
    return get_state('emergency_stop')


def check_heartbeat_timeout():
    last = get_state('last_heartbeat')
    if last is None:
        return False
    return (time.time() - last) > HEARTBEAT_TIMEOUT


# ---------------------------------------------------------------------------
# Obstacle avoidance state machine
#
# States: FORWARD → YAWING → SIDESTEPPING → FORWARD
#
# RAM cost: 4 floats + 1 string = negligible.
# No frame buffering, no history — purely reactive with timeouts so the
# drone doesn't spin forever when a moving obstacle tracks alongside it.
# ---------------------------------------------------------------------------
class ObstacleAvoider:
    FORWARD     = 'forward'
    YAWING      = 'yawing'
    SIDESTEPPING= 'sidestepping'

    def __init__(self, turn_rate, flight_speed, sidestep_speed=0.3):
        self.turn_rate      = turn_rate       # rad/s yaw
        self.flight_speed   = flight_speed    # m/s forward
        self.sidestep_speed = sidestep_speed  # m/s lateral strafe
        self.state          = self.FORWARD
        self._state_start   = time.time()
        self._clear_since   = None

    def _elapsed(self):
        return time.time() - self._state_start

    def _transition(self, new_state):
        logger.info('ObstacleAvoider: %s → %s', self.state, new_state)
        self.state       = new_state
        self._state_start = time.time()

    def update(self, master, obstacle_detected, yaw_err_rad):
        """
        Call once per loop iteration. Sends the appropriate velocity command.

        obstacle_detected: bool
        yaw_err_rad: signed heading error in radians (target bearing − current heading)
        """
        if self.state == self.FORWARD:
            if obstacle_detected:
                self._transition(self.YAWING)
            else:
                # Normal forward flight — yaw correction proportional to heading error
                yaw_cmd = max(-self.turn_rate, min(self.turn_rate, yaw_err_rad * 0.5))
                set_velocity_body(master, self.flight_speed, 0, 0, yaw_cmd)

        elif self.state == self.YAWING:
            if not obstacle_detected:
                # Obstacle cleared while yawing — wait OBS_MIN_CLEAR_SECS before advancing
                if self._clear_since is None:
                    self._clear_since = time.time()
                elif time.time() - self._clear_since >= OBS_MIN_CLEAR_SECS:
                    self._clear_since = None
                    self._transition(self.FORWARD)
                    return
            else:
                self._clear_since = None

            if self._elapsed() > OBS_YAW_TIMEOUT:
                # Yawed long enough without clearing — try strafing right
                logger.warning('ObstacleAvoider: yaw timeout, trying sidestep')
                self._transition(self.SIDESTEPPING)
            else:
                set_velocity_body(master, 0, 0, 0, self.turn_rate)

        elif self.state == self.SIDESTEPPING:
            if self._elapsed() > OBS_SIDESTEP_TIME:
                # Whether or not the obstacle is gone, return to yawing to reassess
                self._transition(self.YAWING)
            elif obstacle_detected:
                set_velocity_body(master, 0, self.sidestep_speed, 0, 0)
            else:
                # Path cleared during sidestep — go straight back to forward
                self._transition(self.FORWARD)


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------
def navigate_to_destination(master, bus, picam2, yolo, cfg: FlightConfig,
                             target_lat, target_lon, show_preview=False):
    if master is None or picam2 is None:
        logger.warning('Cannot navigate: missing MAVLink or camera')
        return 'abort'

    logger.info('Navigating to (%.6f, %.6f)', target_lat, target_lon)

    avoider = ObstacleAvoider(
        turn_rate     = cfg.obstacle_turn_rate,
        flight_speed  = cfg.flight_speed,
    )

    while True:
        # --- Safety checks ---
        if check_emergency_flag():
            logger.critical('Emergency stop during navigation')
            return 'emergency'

        if check_heartbeat_timeout():
            logger.warning('Controller heartbeat lost — returning home')
            return 'lost_link'

        pos = get_gps(master)
        if pos is None:
            logger.warning('GPS unavailable, hovering...')
            set_velocity_body(master, 0, 0, 0, 0)
            time.sleep(1)
            continue

        if check_geofence(pos, cfg.home_pos):
            logger.warning('Geofence breach — landing')
            return 'geofence'

        # --- Arrival check ---
        dlat = target_lat - pos[0]
        dlon = target_lon - pos[1]
        dist_m = math.sqrt((dlat * 111320)**2 +
                           (dlon * 111320 * math.cos(math.radians(pos[0])))**2)
        if dist_m < 2.0:
            logger.info('Destination reached')
            set_velocity_body(master, 0, 0, 0, 0)
            return 'arrived'

        # --- Bearing (true north) ---
        bearing_true = math.degrees(math.atan2(dlon, dlat)) % 360.0

        # --- Current heading (already declination-corrected in read_compass) ---
        if bus:
            try:
                current_heading = read_compass(bus)
            except Exception:
                current_heading = bearing_true  # fall back gracefully
        else:
            current_heading = bearing_true

        # --- Heading error in radians for the avoider ---
        yaw_diff_deg = bearing_true - current_heading
        if yaw_diff_deg >  180: yaw_diff_deg -= 360
        if yaw_diff_deg < -180: yaw_diff_deg += 360
        yaw_err_rad = math.radians(yaw_diff_deg)

        # --- Obstacle detection ---
        img = picam2.capture_array()
        detected  = yolo.findObjects(img)
        obstacle  = is_obstacle_detected(detected, cfg.obstacle_objects)

        # --- Avoider drives all velocity commands ---
        avoider.update(master, obstacle, yaw_err_rad)

        if show_preview and os.environ.get('DISPLAY'):
            cv2.imshow('NOMAD', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        time.sleep(0.1)

    return 'interrupted'


# ---------------------------------------------------------------------------
# Test mode
# ---------------------------------------------------------------------------
def run_test_mode(master, bus, picam2, yolo, cfg: FlightConfig, args):
    """
    Hardware-on test mode — no arming or takeoff.
    Validates camera, compass, MAVLink comms, YOLO detection, and web server.
    """
    logger.info('=' * 50)
    logger.info('TEST MODE — hardware active, motors will NOT spin')
    logger.info('=' * 50)

    set_state(mode='test')

    if master:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            logger.info('[TEST] MAVLink heartbeat OK — firmware type %d', msg.autopilot)
        else:
            logger.warning('[TEST] MAVLink heartbeat timeout')

        v = get_battery_voltage(master)
        if v:
            logger.info('[TEST] Battery: %.2f V', v)

        pos = get_gps(master)
        if pos:
            logger.info('[TEST] GPS fix: %.6f, %.6f', pos[0], pos[1])
        else:
            logger.warning('[TEST] No GPS fix yet')
    else:
        logger.warning('[TEST] MAVLink not available')

    if bus:
        try:
            hdg = read_compass(bus)
            logger.info('[TEST] Compass heading (true): %.1f°', hdg)
        except Exception as exc:
            logger.warning('[TEST] Compass read failed: %s', exc)
    else:
        logger.warning('[TEST] Compass (smbus) not available')

    logger.info('[TEST] Starting camera + YOLO loop (Ctrl+C to stop)...')
    frame_count = 0
    try:
        while True:
            if check_emergency_flag():
                logger.info('[TEST] Emergency stop received via web UI')
                set_state(emergency_stop=False)
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
        set_state(mode='idle')
        if args.preview and os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run_main():
    args        = parse_args()
    config_data = load_config(args.config)

    destinations = {
        k: (v['latitude'], v['longitude'])
        for k, v in config_data.get('destinations', {}).items()
    }

    cfg = build_flight_config(config_data, destinations)

    # Start web server
    if not args.no_server:
        logger.info('Starting web server on port %d', args.port)
        start_server_background('0.0.0.0', args.port)

    # Initialize YOLO
    yolo = sy.SkeletonYolo(
        confThreshold=cfg.confidence_threshold,
        classes=cfg.obstacle_objects if cfg.obstacle_objects else None,
    )

    # Dry-run mode
    if args.dry_run:
        logger.info('DRY RUN — no hardware. Web server at http://<pi-ip>:%d', args.port)
        logger.info('Destinations loaded: %s', list(destinations.keys()))
        set_state(mode='dry_run')
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('Dry run ended')
        return

    # Hardware initialization
    picam2 = init_camera()
    master = init_mavlink()
    bus    = init_compass()

    # Test mode
    if args.test:
        run_test_mode(master, bus, picam2, yolo, cfg, args)
        if picam2:
            picam2.stop()
        return

    # Full flight mode
    if picam2 is None or master is None:
        logger.error('Hardware init failed. Use --dry-run or --test to proceed without hardware.')
        return

    set_state(mode='preflight')

    # Wait for web controller to authorize launch
    logger.info('Waiting for web controller connection...')
    logger.info('Open http://<pi-ip>:%d and press LAUNCH when ready', args.port)
    while not get_state('launch_authorized'):
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
        set_state(mode='idle')
        return

    set_state(mode='airborne', last_heartbeat=time.time())

    takeoff(master, cfg.takeoff_altitude)
    time.sleep(6)

    # Get destination from web UI
    dest_key = get_state('destination').upper()
    if dest_key not in destinations:
        logger.warning('No valid destination set — landing')
        land(master)
        time.sleep(6)
        disarm_drone(master)
        set_state(mode='idle')
        return

    target_lat, target_lon = destinations[dest_key]

    try:
        result = navigate_to_destination(
            master, bus, picam2, yolo, cfg,
            target_lat, target_lon,
            show_preview=args.preview,
        )

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

        v = get_battery_voltage(master)
        if v:
            logger.info('Post-flight battery: %.2f V', v)
            if v < cfg.low_battery_voltage:
                logger.warning('LOW BATTERY: %.2f V — charge before next flight', v)

    except KeyboardInterrupt:
        logger.info('Ctrl+C received — initiating landing')
        land(master)

    finally:
        time.sleep(6)
        disarm_drone(master)
        set_state(
            mode='idle',
            emergency_stop=False,
            launch_authorized=False,
        )
        if picam2:
            picam2.stop()
        if args.preview and os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()
        logger.info('Flight session ended')


if __name__ == '__main__':
    run_main()
