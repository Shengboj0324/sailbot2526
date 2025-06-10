#!/bin/bash
# Script to ensure GPS data is published to web server during autonomous operation
# This script monitors the GPS data and ensures it's being sent to the remote server

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}===== GPS Publishing Monitor =====${NC}"

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to workspace root
cd "$SCRIPT_DIR"

# Source ROS
echo -e "${YELLOW}Sourcing ROS...${NC}"
source /opt/ros/jazzy/setup.bash

# Source the workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}Workspace sourced${NC}"
else
    echo -e "${RED}Workspace not built! Run start_sailbot.sh first.${NC}"
    exit 1
fi

# Function to check if cellular_comm node is running
check_cellular_comm() {
    if ros2 node list | grep -q cellular_comm_node; then
        return 0
    else
        return 1
    fi
}

# Function to monitor GPS topics
monitor_gps() {
    echo -e "${YELLOW}Monitoring GPS data publishing...${NC}"
    echo -e "${GREEN}GPS topics being monitored:${NC}"
    echo "  - /gps/fix (latitude, longitude)"
    echo "  - /gps/speed"
    echo "  - /gps/fix_quality"
    echo ""
    
    # Monitor in background
    ros2 topic echo /gps/fix --once &
    local fix_pid=$!
    
    ros2 topic echo /gps/speed --once &
    local speed_pid=$!
    
    ros2 topic echo /gps/fix_quality --once &
    local quality_pid=$!
    
    # Wait for topics to display
    sleep 2
    
    # Clean up background processes
    kill $fix_pid $speed_pid $quality_pid 2>/dev/null
}

# Function to check boat status
check_boat_status() {
    echo -e "${YELLOW}Checking boat status...${NC}"
    ros2 topic echo /boat_status --once | grep -E "control_mode|event_type" || echo "Status not available"
}

# Main monitoring loop
echo -e "${YELLOW}Starting GPS publishing monitor...${NC}"

# Check if cellular_comm node is running
if check_cellular_comm; then
    echo -e "${GREEN}Cellular communication node is running${NC}"
    echo -e "${GREEN}GPS data is being published to: wss://sailbot-relay.onrender.com${NC}"
else
    echo -e "${RED}Cellular communication node is NOT running!${NC}"
    echo -e "${YELLOW}Starting cellular_comm node...${NC}"
    ros2 run sensors cellular_comm &
    CELL_PID=$!
    sleep 3
fi

# Set up trap to handle Ctrl+C
trap 'echo -e "\n${YELLOW}Stopping GPS monitor...${NC}"; [ ! -z "$CELL_PID" ] && kill $CELL_PID 2>/dev/null; exit 0' SIGINT

# Monitor GPS publishing
while true; do
    echo -e "\n${YELLOW}===== GPS Publishing Status =====${NC}"
    date
    
    # Check if nodes are still running
    if ! check_cellular_comm; then
        echo -e "${RED}WARNING: Cellular comm node stopped! Restarting...${NC}"
        ros2 run sensors cellular_comm &
        CELL_PID=$!
        sleep 3
    fi
    
    # Show current GPS data
    monitor_gps
    
    # Show boat status
    check_boat_status
    
    echo -e "\n${GREEN}GPS data is being published every 5 seconds to the web server${NC}"
    echo -e "${YELLOW}Press Ctrl+C to stop monitoring${NC}"
    
    # Wait before next check
    sleep 10
done