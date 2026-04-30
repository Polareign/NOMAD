#!/bin/bash
set -e

# NOMAD Wi-Fi Hotspot + Captive Portal Setup
# Usage: sudo bash setup_hotspot.sh

HOTSPOT_SSID="NOMAD"
HOTSPOT_PASSPHRASE="polareign"
AP_IP="192.168.4.1"
DHCP_RANGE_START="192.168.4.2"
DHCP_RANGE_END="192.168.4.20"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root."
    echo "Usage: sudo bash setup_hotspot.sh"
    exit 1
fi

echo "=== NOMAD Wi-Fi Hotspot Setup ==="

echo "Installing required packages..."
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

echo "Starting services..."
systemctl unmask hostapd
systemctl enable hostapd
systemctl enable dnsmasq
systemctl restart dhcpcd
systemctl restart hostapd
systemctl restart dnsmasq

echo ""
echo "Hotspot setup complete."
echo "SSID: ${HOTSPOT_SSID}"
echo "Password: ${HOTSPOT_PASSPHRASE}"
echo "Point your browser at: http://${AP_IP}:5000/"
echo "If automatic captive portal detection does not work, manually visit the URL above."
