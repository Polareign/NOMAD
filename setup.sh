#!/bin/bash
set -e
# Setup script for NOMAD drone control system

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
    libcap-dev \
    libopenjp2-7 \
    libopenjp2-7-dev \
    i2c-tools \
    libkrb5-dev \
    libssl-dev

# Step 2.5: Hotfix
echo "Removing Environment"
rm -rf venv
echo "Creating Environment"
python3 -m venv venv

# Step 3: Create virtual environment
echo ""
echo "Step 3: Setting up Python virtual environment..."
if [ ! -d "venv" ] || [ ! -f "venv/bin/pip" ]; then
    echo "Creating virtual environment..."
    rm -rf venv
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
python -m pip install --upgrade pip setuptools wheel

# Step 5: Install Python requirements
echo ""
echo "Step 5: Installing Python packages..."
echo "Step 5a: Installing CPU-only PyTorch and torchvision..."
python -m pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu
echo "✓ CPU-only PyTorch installed"
echo "Step 5b: Installing remaining Python requirements..."
python -m pip install --no-cache-dir -r requirements.txt

echo ""
echo "===== Installation Complete ====="
echo ""
echo "To use the drone control system:"
echo "  1. Activate venv: source venv/bin/activate"
echo "  2. Run: python drone_control.py"
echo ""
echo "The YOLO11n model will be automatically downloaded on first run."
echo "To deactivate venv: deactivate"
echo ""