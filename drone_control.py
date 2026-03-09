import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import time
import smbus
import math
import SkeletonYolo as sy
from destinations import DESTINATIONS, TAKEOFF_ALTITUDE, FLIGHT_SPEED, YAW_SENSITIVITY, OBSTACLE_TURN_RATE, OBSTACLE_OBJECTS

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

# MAVLink connection to flight controller
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)  # Adjust port and baud for iNAV

# I2C for HMC5883L compass
bus = smbus.SMBus(1)
HMC5883L_ADDRESS = 0x1E

def init_compass():
    bus.write_byte_data(HMC5883L_ADDRESS, 0x00, 0x70)  # 8 samples, 15Hz
    bus.write_byte_data(HMC5883L_ADDRESS, 0x01, 0xA0)  # Gain
    bus.write_byte_data(HMC5883L_ADDRESS, 0x02, 0x00)  # Continuous mode

def read_compass():
    data = bus.read_i2c_block_data(HMC5883L_ADDRESS, 0x03, 6)
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

init_compass()

# Initialize YOLO
yolo = sy.SkeletonYolo()

def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("Arming drone...")

def disarm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    print("Disarming drone...")

def takeoff(altitude=TAKEOFF_ALTITUDE):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    print(f"Taking off to {altitude}m...")

def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    print("Landing...")

def set_velocity(vx, vy, vz, yaw_rate):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity
        0, 0, 0,  # position
        vx, vy, vz,  # velocity
        0, 0, 0,  # acceleration
        yaw_rate, 0)  # yaw

def get_gps():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        return lat, lon
    return 0, 0

def navigate_to_destination(target_lat, target_lon):
    while True:
        current_lat, current_lon = get_gps()
        if abs(current_lat - target_lat) < 0.0001 and abs(current_lon - target_lon) < 0.0001:
            break

        # Simple navigation (calculate bearing)
        dlat = target_lat - current_lat
        dlon = target_lon - current_lon
        bearing = math.atan2(dlon, dlat) * 180 / math.pi
        current_heading = read_compass()
        yaw_diff = bearing - current_heading
        if yaw_diff > 180:
            yaw_diff -= 360
        elif yaw_diff < -180:
            yaw_diff += 360

        # Check for obstacles
        img = picam2.capture_array()
        detected = yolo.findObjects(img)
        
        obstacle_detected = any(obj in detected for obj in OBSTACLE_OBJECTS)
        
        if obstacle_detected:
            set_velocity(0, 0, 0, OBSTACLE_TURN_RATE)  # Turn
            time.sleep(1)
        else:
            set_velocity(FLIGHT_SPEED, 0, 0, yaw_diff * YAW_SENSITIVITY)  # Forward with yaw correction
        time.sleep(0.1)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected to flight controller")

# Arm and takeoff
arm_drone()
time.sleep(2)
takeoff()
time.sleep(5)

# Main obstacle avoidance loop
try:
    while True:
        img = picam2.capture_array()
        detected_objects = yolo.findObjects(img)
        
        # Check if any obstacle detected
        obstacle_detected = any(obj in detected_objects for obj in OBSTACLE_OBJECTS)
        
        if obstacle_detected:
            set_velocity(0, 0, 0, OBSTACLE_TURN_RATE)  # Turn right
            time.sleep(1)
        else:
            set_velocity(0, 0, 0, 0)

        cv2.imshow('Image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.1)

    # Autonomous flight
    print("\nAvailable destinations:", list(DESTINATIONS.keys()))
    destination = input("Enter destination: ").strip().upper()
    
    if destination in DESTINATIONS:
        target_lat, target_lon = DESTINATIONS[destination]
        print(f"Navigating to {destination} ({target_lat}, {target_lon})...")
        navigate_to_destination(target_lat, target_lon)
    else:
        print("Invalid destination. Landing now...")

except KeyboardInterrupt:
    pass

# Land and disarm
land()
time.sleep(5)
disarm_drone()
picam2.stop()
cv2.destroyAllWindows()