#!/bin/bash
# Setup script for NOMAD drone control system
# Handles installation on Raspberry Pi 4 with proper dependency management

echo "===== NOMAD Drone Control Setup ====="
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /sys/firmware/devicetree/base/model 2>/dev/null; then
    echo "⚠ Warning: This may not be running on Raspberry Pi"
fi

# Step 1: Update system packages
echo "Step 1: Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Step 2: Install system dependencies
echo "Step 2: Installing system dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    libatlas-base-dev \
    libjasper-dev \
    libtiff5 \
    libjasper1 \
    libharfbuzz0b \
    libwebp6 \
    libtiff5 \
    libopenjp2-7 \
    libopenjp2-7-dev \
    i2c-tools \
    libkrb5-dev \
    libssl-dev

# Step 3: Create virtual environment (RECOMMENDED)
echo ""
echo "Step 3: Setting up Python virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "✓ Virtual environment created"
else
    echo "✓ Virtual environment already exists"
fi

# Activate virtual environment
source venv/bin/activate
echo "✓ Virtual environment activated"

# Step 4: Upgrade pip, setuptools, and wheel
echo ""
echo "Step 4: Upgrading pip and build tools..."
pip install --upgrade pip setuptools wheel

# Step 5: Install Python requirements
echo ""
echo "Step 5: Installing Python packages..."
pip install numpy==1.24.3
echo "✓ NumPy installed"

pip install opencv-python==4.8.0.74
echo "✓ OpenCV installed"

pip install pymavlink==2.4.41
echo "✓ PyMAVLink installed"

pip install picamera2
echo "✓ PiCamera2 installed"

pip install Flask==2.3.3
echo "✓ Flask installed"

pip install ultralytics==8.0.0
echo "✓ Ultralytics installed"

pip install torch==2.0.1
echo "✓ PyTorch installed"

echo ""
echo "===== Installation Complete ====="
echo ""
echo "To use the drone control system:"
echo "  1. Activate venv: source venv/bin/activate"
echo "  2. Configure hardware (GPS, compass, camera)"
echo "  3. Run: python drone_control.py"
echo ""
echo "The YOLO11n model will be automatically downloaded on first run."
echo "To deactivate venv: deactivate"
echo ""