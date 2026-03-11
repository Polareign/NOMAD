#!/usr/bin/env python3
"""
Test script to verify drone control setup
"""
print("=== Drone Control Setup Test ===\n")

tests = []

# Test OpenCV
try:
    import cv2
    print("OpenCV OK")
    tests.append(True)
except ImportError:
    print("OpenCV not installed")
    tests.append(False)

# Test NumPy
try:
    import numpy as np
    print("NumPy OK")
    tests.append(True)
except ImportError:
    print("NumPy not installed")
    tests.append(False)

# Test PiCamera2
try:
    from picamera2 import Picamera2
    print("PiCamera2 OK")
    tests.append(True)
except ImportError:
    print("PiCamera2 not installed")
    tests.append(False)

# Test PyMAVLink
try:
    from pymavlink import mavutil
    print("PyMAVLink OK")
    tests.append(True)
except ImportError:
    print("PyMAVLink not installed")
    tests.append(False)

# Test SMBus
try:
    import smbus
    print("SMBus OK")
    tests.append(True)
except ImportError:
    print("SMBus not installed")
    tests.append(False)

# Test YOLO files
import os
yolo_files = ['obstacles.names', 'yolov3.cfg', 'yolov3.weights']
for file in yolo_files:
    if os.path.exists(file):
        print(f"{file} found")
        tests.append(True)
    else:
        print(f"{file} missing")
        tests.append(False)

print(f"\n=== Summary: {sum(tests)}/{len(tests)} tests passed ===")

if all(tests):
    print("All basic tests passed! Ready for hardware testing.")
else:
    print("Some components missing. Install with: pip install -r requirements.txt")
    print("Download missing YOLO files as per README.")