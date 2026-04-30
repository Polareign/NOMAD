#!/usr/bin/env python3
"""
Test script to verify drone control setup
"""
print("=== Drone Control Setup Test ===\n")

tests = []

# Test OpenCV
try:
    import cv2
    print("✓ OpenCV OK")
    tests.append(True)
except ImportError:
    print("✗ OpenCV not installed")
    tests.append(False)

# Test NumPy
try:
    import numpy as np
    print("✓ NumPy OK")
    tests.append(True)
except ImportError:
    print("✗ NumPy not installed")
    tests.append(False)

# Test Ultralytics (YOLO11n)
try:
    from ultralytics import YOLO
    print("✓ Ultralytics OK")
    tests.append(True)
except ImportError:
    print("✗ Ultralytics not installed")
    tests.append(False)

# Test PyTorch
try:
    import torch
    print("✓ PyTorch OK")
    tests.append(True)
except ImportError:
    print("✗ PyTorch not installed")
    tests.append(False)

# Test PiCamera2
try:
    from picamera2 import Picamera2
    print("✓ PiCamera2 OK")
    tests.append(True)
except ImportError:
    print("✗ PiCamera2 not installed (OK if not on Raspberry Pi)")
    tests.append(True)

# Test PyMAVLink
try:
    from pymavlink import mavutil
    print("✓ PyMAVLink OK")
    tests.append(True)
except ImportError:
    print("✗ PyMAVLink not installed")
    tests.append(False)

# Test SMBus
try:
    import smbus2
    print("✓ SMBus OK")
    tests.append(True)
except ImportError:
    print("✗ SMBus not installed (OK if not on Raspberry Pi)")
    tests.append(True)

# Test flask
try:
    import flask
    print("✓ Flask OK")
    tests.append(True)
except ImportError:
    print("✗ Flask not installed")
    tests.append(False)

# Test configuration
import os
import json

config_ok = False
if os.path.exists('config.json'):
    try:
        with open('config.json', 'r') as f:
            data = json.load(f)
            if data.get('flight_parameters') and data.get('destinations'):
                print('✓ config.json is properly configured')
                tests.append(True)
                config_ok = True
            else:
                print('✗ config.json is missing required fields')
                tests.append(False)
    except json.JSONDecodeError:
        print('✗ config.json exists but could not be parsed')
        tests.append(False)
else:
    print('✗ config.json not found')
    tests.append(False)

print(f"\n=== Summary: {sum(tests)}/{len(tests)} tests passed ===")

if all(tests):
    print("✓ All tests passed! Ready for drone control.")
else:
    print("Some components are missing. Install with: pip install -r requirements.txt")
