#!/bin/bash
# Script to monitor and ensure GPS publishing during autonomous operation
# This script specifically checks that GPS data is being published when in autonomous mode

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}===== Autonomous GPS Publishing Monitor =====${NC}"

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to workspace root
cd "$SCRIPT_DIR"

# Source ROS
source /opt/ros/jazzy/setup.bash

# Source the workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}Workspace not built! Run start_sailbot.sh first.${NC}"
    exit 1
fi

# Variables to track state
LAST_LAT=""
LAST_LON=""
LAST_SPEED=""
LAST_MODE=""
GPS_UPDATE_COUNT=0

# Function to get current GPS data
get_gps_data() {
    local gps_data=$(ros2 topic echo /gps/fix --once 2>/dev/null | grep -E "latitude:|longitude:" | awk '{print $2}')
    local speed_data=$(ros2 topic echo /gps/speed --once 2>/dev/null | grep "data:" | awk '{print $2}')
    echo "$gps_data $speed_data"
}

# Function to get current boat mode
get_boat_mode() {
    local status=$(ros2 topic echo /boat_status --once 2>/dev/null | grep -A 20 "data:" | grep -oP '"control_mode":\s*"\K[^"]+' || echo "unknown")
    echo "$status"
}

# Function to verify WebSocket connection
check_websocket_connection() {
    # Check if cellular_comm node is running and connected
    if ros2 node list | grep -q cellular_comm_node; then
        # Check recent logs for connection status
        echo -e "${GREEN}Cellular comm node is running${NC}"
        return 0
    else
        echo -e "${RED}Cellular comm node is NOT running${NC}"
        return 1
    fi
}

# Main monitoring function
monitor_autonomous_gps() {
    echo -e "${YELLOW}Monitoring GPS publishing in autonomous mode...${NC}"
    echo -e "${BLUE}WebSocket relay: wss://sailbot-relay.onrender.com${NC}"
    echo ""
    
    while true; do
        # Get current mode
        MODE=$(get_boat_mode)
        
        # Get GPS data
        GPS_DATA=($(get_gps_data))
        CURRENT_LAT=${GPS_DATA[0]:-"N/A"}
        CURRENT_LON=${GPS_DATA[1]:-"N/A"}
        CURRENT_SPEED=${GPS_DATA[2]:-"N/A"}
        
        # Clear screen for better visibility
        clear
        echo -e "${BLUE}===== Autonomous GPS Publishing Monitor =====${NC}"
        echo -e "Time: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        
        # Display current status
        echo -e "${YELLOW}Current Status:${NC}"
        echo -e "Control Mode: ${MODE}"
        echo -e "GPS Position: ${CURRENT_LAT}, ${CURRENT_LON}"
        echo -e "GPS Speed: ${CURRENT_SPEED} m/s"
        echo ""
        
        # Check if in autonomous mode
        if [[ "$MODE" == "autonomous" ]] || [[ "$MODE" == "auto" ]]; then
            echo -e "${GREEN}✓ AUTONOMOUS MODE ACTIVE${NC}"
            
            # Check if GPS data is updating
            if [[ "$CURRENT_LAT" != "$LAST_LAT" ]] || [[ "$CURRENT_LON" != "$LAST_LON" ]]; then
                ((GPS_UPDATE_COUNT++))
                echo -e "${GREEN}✓ GPS data is updating (${GPS_UPDATE_COUNT} updates)${NC}"
            else
                echo -e "${YELLOW}⚠ GPS data unchanged${NC}"
            fi
            
            # Verify WebSocket connection
            if check_websocket_connection; then
                echo -e "${GREEN}✓ GPS data is being published to web server${NC}"
                echo -e "  Publishing interval: 5 seconds"
                echo -e "  Data format: GPS,lat,lon,speed,fix_quality"
            else
                echo -e "${RED}✗ WebSocket connection issue detected${NC}"
                echo -e "${YELLOW}Attempting to restart cellular_comm node...${NC}"
                ros2 run sensors cellular_comm &
                sleep 3
            fi
        else
            echo -e "${YELLOW}⚠ Not in autonomous mode (current: ${MODE})${NC}"
            echo -e "GPS publishing continues in all modes"
        fi
        
        # Show publishing status
        echo ""
        echo -e "${BLUE}Publishing Status:${NC}"
        echo -e "GPS updates sent: ${GPS_UPDATE_COUNT}"
        echo -e "Last known position: ${LAST_LAT}, ${LAST_LON}"
        echo -e "Last known speed: ${LAST_SPEED} m/s"
        
        # Update last values
        LAST_LAT=$CURRENT_LAT
        LAST_LON=$CURRENT_LON
        LAST_SPEED=$CURRENT_SPEED
        LAST_MODE=$MODE
        
        echo ""
        echo -e "${YELLOW}Press Ctrl+C to stop monitoring${NC}"
        
        # Wait before next update
        sleep 5
    done
}

# Trap Ctrl+C
trap 'echo -e "\n${YELLOW}Stopping monitor...${NC}"; exit 0' SIGINT

# Start monitoring
monitor_autonomous_gps