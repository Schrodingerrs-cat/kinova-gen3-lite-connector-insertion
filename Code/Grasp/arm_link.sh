#!/usr/bin/env bash
# Bring up the USB link to the Gen3 Lite (RNDIS) and check the arm answers. Needs sudo for ip. Safe to re-run.
ROBOT_IP=192.168.1.10
HOST_IP=192.168.1.11
# already reachable over a USB (RNDIS) interface: nothing to do, no sudo
if ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1 && ip route get "$ROBOT_IP" | grep -q "dev en"; then
  echo ">> Arm reachable at $ROBOT_IP via $(ip route get "$ROBOT_IP" | sed -n 's/.*dev \([^ ]*\).*/\1/p')"; exit 0
fi
IFACE=""
for d in /sys/class/net/*; do
  n=$(basename "$d")
  if [ -L "$d/device/driver" ] && [ "$(basename "$(readlink -f "$d/device/driver")")" = "rndis_host" ]; then IFACE=$n; fi
done
[ -n "$IFACE" ] || { echo "!! Arm USB interface not found: micro-B cable, arm powered?"; exit 1; }
ip -4 addr show dev "$IFACE" | grep -q "$HOST_IP" || sudo ip addr add "$HOST_IP/24" dev "$IFACE"
sudo ip link set "$IFACE" up
# wifi may sit on 192.168.1.0/24 too: pin the arm to the USB interface
sudo ip route replace "$ROBOT_IP/32" dev "$IFACE"
ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1 || { echo "!! Cannot ping $ROBOT_IP on $IFACE"; exit 1; }
echo ">> Arm reachable at $ROBOT_IP via $IFACE"
