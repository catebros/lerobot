#!/bin/bash
# Quick camera detection test for W250 setup

echo "========================================="
echo "RealSense Camera Detection Test"
echo "========================================="
echo ""

echo "1. Checking USB Bus Assignments:"
echo "---------------------------------"
lsusb | grep "Intel.*RealSense"
echo ""

echo "2. Enumerating RealSense Devices:"
echo "---------------------------------"
rs-enumerate-devices 2>&1 | grep -E "Serial Number|Physical Port|Usb Type" | head -12
echo ""

echo "3. Expected Configuration:"
echo "---------------------------------"
echo "✓ Camera 1: Serial 220322061194"
echo "✓ Camera 2: Serial 220222065523"
echo ""

echo "4. USB Bus Status:"
echo "---------------------------------"
camera_count=$(lsusb | grep -c "Intel.*RealSense")
echo "Cameras detected via lsusb: $camera_count"

if [ $camera_count -eq 2 ]; then
    echo "✅ SUCCESS: Both cameras detected!"
    echo ""
    echo "Checking bus distribution:"
    lsusb | grep "Intel.*RealSense" | while read line; do
        bus=$(echo $line | awk '{print $2}')
        echo "  - Camera on Bus $bus"
    done
elif [ $camera_count -eq 1 ]; then
    echo "⚠️  WARNING: Only 1 camera detected"
    echo ""
    echo "Troubleshooting steps:"
    echo "  1. Try plugging Camera 2 into different USB 3.0 port"
    echo "  2. Goal: Get cameras on different buses (Bus 002 and Bus 004)"
    echo "  3. Avoid using USB hub for RealSense cameras if possible"
else
    echo "❌ ERROR: No cameras detected"
    echo "  - Check camera connections"
    echo "  - Ensure cameras are plugged into USB 3.0 ports (blue/teal)"
fi
echo ""
echo "========================================="
