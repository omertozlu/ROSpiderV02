#!/bin/bash

#This is a test script to verify if everything is working correctly. NOTE: we should explore the idea of automating initial testing.
# --- Colors for Output ---
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "============================================"
echo "🕷️  ROSpider Environment Verification Script"
echo "============================================"

# Ensure ROS is sourced
source /opt/ros/humble/setup.bash

# Function to print results
print_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}[PASS]${NC} $2"
    else
        echo -e "${RED}[FAIL]${NC} $2"
        echo "       Error details: $3"
        # Optional: Exit on critical failure
        # exit 1
    fi
}

# ---------------------------------------------------------
# Test 1: Environment Variables
# ---------------------------------------------------------
echo -n "Checking ROS 2 Environment... "
if [[ "$ROS_DISTRO" == "humble" ]]; then
    print_result 0 "ROS_DISTRO is set to Humble."
else
    print_result 1 "ROS_DISTRO not found."
fi

# ---------------------------------------------------------
# Test 2: ROS 2 Communication (Talker/Listener)
# ---------------------------------------------------------
echo -n "Checking ROS 2 Communication... "

# Start talker in background and send output to /dev/null to keep screen clean
ros2 run demo_nodes_cpp talker > /dev/null 2>&1 &
TALKER_PID=$!

# Give it a second to start
sleep 1

# Run listener for 3 seconds and capture output
LISTENER_OUTPUT=$(timeout 3s ros2 run demo_nodes_py listener 2>&1)

# Kill the talker
kill $TALKER_PID > /dev/null 2>&1
wait $TALKER_PID 2>/dev/null

# Check if listener heard anything
if echo "$LISTENER_OUTPUT" | grep -q "I heard"; then
    print_result 0 "Talker/Listener successful."
else
    print_result 1 "Nodes could not communicate." "$LISTENER_OUTPUT"
fi

# ---------------------------------------------------------
# Test 3: GUI & Display (Turtlesim)
# ---------------------------------------------------------
echo -n "Checking GUI (X11 Display)... "

# Try to run turtlesim for 2 seconds. 
# timeout returns 124 if it times out (which means it successfully ran for 2s!)
timeout 2s ros2 run turtlesim turtlesim_node > /tmp/turtle_log 2>&1
EXIT_CODE=$?

if [ $EXIT_CODE -eq 124 ]; then
    print_result 0 "Turtlesim launched successfully."
else
    # If it crashed immediately, exit code will be != 124
    print_result 1 "GUI failed to open." "$(cat /tmp/turtle_log)"
fi

# ---------------------------------------------------------
# Test 4: GPU Acceleration (glxgears)
# ---------------------------------------------------------
echo -n "Checking GPU Acceleration... "

# Check for mesa-utils first
if ! command -v glxinfo &> /dev/null; then
    echo -e "${RED}[WARN]${NC} glxinfo not found (mesa-utils missing). Skipping GPU test."
else
    # Run glxinfo and check for "direct rendering: Yes"
    if glxinfo | grep -q "direct rendering: Yes"; then
         print_result 0 "Direct Rendering is enabled."
    else
         print_result 1 "Software Rendering detected (Slow)."
    fi
fi

echo "============================================"
echo "✅ Verification Complete"