#!/bin/sh

# Configure laptop as a router for the KR260 network
OUT_IFACE=wlp4s0
IN_IFACE=enp3s0

sudo echo 1 > /proc/sys/net/ipv4/ip_forward
sudo iptables -t nat -A POSTROUTING -o $OUT_IFACE -j MASQUERADE
sudo iptables -A FORWARD -i $IN_IFACE -o $OUT_IFACE -j ACCEPT
sudo iptables -A FORWARD -i $OUT_IFACE -o $IN_IFACE -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -j DROP
