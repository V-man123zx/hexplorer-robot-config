#!/bin/bash
# Macarena Dance for the Hexplorer!
# Usage: bash ~/hexplorer/scripts/start_macarena.sh

echo "==================================="
echo "  HEEEY MACARENA!"
echo "  Hexplorer Dance Mode"
echo "==================================="
echo ""
echo "The robot will:"
echo "  1. Stand up"
echo "  2. Dance the Macarena (3 cycles)"
echo "  3. Sit down"
echo ""
echo "Press Ctrl+C to stop at any time."
echo ""

source /home/robot/robot_controller_release/ros2_packages/setup.bash
python3 /home/robot/hexplorer/navigation/macarena_dance.py
