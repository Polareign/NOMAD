#!/usr/bin/env python3
"""
Test script to verify drone control setup
Usage: python test_setup.py [--hotspot]
"""
import argparse
import json
import os
import subprocess
import sys

def test_hotspot_redirects():
    """Test captive portal redirect routes"""
    print("\n=== Testing Hotspot Redirect Routes ===")

    base_url = "http://127.0.0.1:5000"
    paths = ["/generate_204", "/hotspot-detect.html", "/test-path"]

    try:
        import requests
    except ImportError:
        print("✗ requests library not available for redirect testing")
        return False

    success_count = 0
    for path in paths:
        try:
            response = requests.get(f"{base_url}{path}", allow_redirects=False, timeout=5)
            if response.status_code == 302 and 'location' in response.headers:
                location = response.headers['location']
                if location == '/' or location.startswith('/'):
                    print(f"✓ {path} -> {location}")
                    success_count += 1
                else:
                    print(f"✗ {path} -> {location} (unexpected redirect)")
            else:
                print(f"✗ {path} -> status {response.status_code}")
        except Exception as e:
            print(f"✗ {path} -> error: {e}")

    if success_count == len(paths):
        print("✓ All redirect routes working correctly")
        return True
    else:
        print(f"✗ {success_count}/{len(paths)} redirect routes working")
        return False

def main():
    parser = argparse.ArgumentParser(description='Test drone control setup')
    parser.add_argument('--hotspot', action='store_true', help='Also test hotspot redirect routes')
    args = parser.parse_args()

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

    if all(tests):
        print("✓ All tests passed! Ready for drone control.")
    else:
        print("Some components are missing. Install with: pip install -r requirements.txt")

    # Optional hotspot redirect testing
    if args.hotspot:
        hotspot_ok = test_hotspot_redirects()
        if hotspot_ok:
            print("\n✓ Hotspot redirect testing passed!")
        else:
            print("\n✗ Hotspot redirect testing failed!")
            sys.exit(1)

if __name__ == "__main__":
    main()
