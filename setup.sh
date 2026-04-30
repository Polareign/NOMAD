#!/bin/bash
set -e
# Setup script for NOMAD drone control system
# Usage: bash setup.sh [--hotspot] [--test]

HOTSPOT_MODE=false
TEST_MODE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --hotspot)
            HOTSPOT_MODE=true
            shift
            ;;
        --test)
            TEST_MODE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: bash setup.sh [--hotspot] [--test]"
            exit 1
            ;;
    esac
done

echo "===== NOMAD Drone Control Setup ====="
if [ "$HOTSPOT_MODE" = true ]; then
    echo "Hotspot mode enabled"
fi
if [ "$TEST_MODE" = true ]; then
    echo "Test mode enabled"
fi
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

# Optional hotspot setup
if [ "$HOTSPOT_MODE" = true ]; then
    echo ""
    echo "===== Setting up Wi-Fi Hotspot ====="
    echo ""

    HOTSPOT_SSID="NOMAD"
    HOTSPOT_PASSPHRASE="polareign"
    AP_IP="192.168.4.1"
    DHCP_RANGE_START="192.168.4.2"
    DHCP_RANGE_END="192.168.4.20"

    if [ "$(id -u)" -ne 0 ]; then
        echo "ERROR: Hotspot setup requires root privileges."
        echo "Run: sudo bash setup.sh --hotspot"
        exit 1
    fi

    echo "Installing hotspot packages..."
    apt-get update
    apt-get install -y hostapd dnsmasq

    systemctl stop hostapd
    systemctl stop dnsmasq

    backup_file() {
        if [ -f "$1" ]; then
            local stamp=$(date +%s)
            cp "$1" "$1.bak.$stamp"
            echo "Backed up $1 -> $1.bak.$stamp"
        fi
    }

    backup_file /etc/dhcpcd.conf
    backup_file /etc/dnsmasq.conf
    backup_file /etc/hostapd/hostapd.conf
    backup_file /etc/default/hostapd

    cat > /etc/dhcpcd.conf <<EOF
interface wlan0
    static ip_address=${AP_IP}/24
    nohook wpa_supplicant
EOF

    cat > /etc/dnsmasq.conf <<EOF
interface=wlan0
bind-interfaces
server=1.1.1.1
domain-needed
bogus-priv
cache-size=1000
dhcp-range=${DHCP_RANGE_START},${DHCP_RANGE_END},255.255.255.0,24h
address=/#/${AP_IP}
EOF

    cat > /etc/hostapd/hostapd.conf <<EOF
interface=wlan0
driver=nl80211
ssid=${HOTSPOT_SSID}
hw_mode=g
channel=7
ieee80211n=1
wmm_enabled=1
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${HOTSPOT_PASSPHRASE}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

    if grep -q '^#*DAEMON_CONF=' /etc/default/hostapd; then
        sed -i 's|^#*DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
    else
        echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' >> /etc/default/hostapd
    fi

    sysctl -w net.ipv4.ip_forward=1

    echo "Starting hotspot services..."
    systemctl unmask hostapd
    systemctl enable hostapd
    systemctl enable dnsmasq
    systemctl restart dhcpcd
    systemctl restart hostapd
    systemctl restart dnsmasq

    echo ""
    echo "Hotspot setup complete!"
    echo "SSID: ${HOTSPOT_SSID}"
    echo "Password: ${HOTSPOT_PASSPHRASE}"
    echo "Web interface: http://${AP_IP}:5000/"
    echo ""
fi

# Detect Pi's IP address for web interface
echo ""
echo "===== Network Information ====="
if [ "$HOTSPOT_MODE" = true ]; then
    PI_IP="${AP_IP}"
    INTERFACE="wlan0 (hotspot)"
else
    # Get the IP of the default route interface
    DEFAULT_IFACE=$(ip route show default | awk '{print $5}' | head -1)
    if [ -n "$DEFAULT_IFACE" ]; then
        PI_IP=$(ip addr show "$DEFAULT_IFACE" | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
        INTERFACE="$DEFAULT_IFACE"
    else
        PI_IP="unknown"
        INTERFACE="unknown"
    fi
fi

if [ "$PI_IP" != "unknown" ]; then
    echo "Pi IP Address: $PI_IP (interface: $INTERFACE)"
    echo "Web interface: http://${PI_IP}:5000/"
else
    echo "Could not determine Pi IP address"
    echo "Web interface: http://<pi-ip>:5000/"
fi
echo ""

# Run tests if requested
if [ "$TEST_MODE" = true ]; then
    echo "===== Running Setup Tests ====="
    echo ""
    
    # Activate venv for testing
    source venv/bin/activate
    
    # Run basic tests
    echo "Running basic component tests..."
    python test_setup.py
    
    # Run hotspot tests if hotspot mode was enabled
    if [ "$HOTSPOT_MODE" = true ]; then
        echo ""
        echo "Running hotspot redirect tests..."
        # Start web server in background for testing
        python web_server.py &
        SERVER_PID=$!
        sleep 2
        
        # Run hotspot tests
        python test_setup.py --hotspot
        
        # Stop web server
        kill $SERVER_PID 2>/dev/null || true
        wait $SERVER_PID 2>/dev/null || true
    fi
    
    echo ""
    echo "===== Test Results ====="
    echo "If all tests passed, your setup is ready!"
    echo "Run: source venv/bin/activate && python drone_control.py"
fi