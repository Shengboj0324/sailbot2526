#!/bin/bash
# Script to start the enhanced GPS publishing with better monitoring
# This ensures GPS data is properly published to the web server

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}===== Enhanced GPS Publishing System =====${NC}"

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

# Function to stop existing cellular_comm node
stop_existing_cellular() {
    echo -e "${YELLOW}Checking for existing cellular_comm node...${NC}"
    # Get PID of existing cellular_comm node
    local pid=$(pgrep -f "ros2 run sensors cellular_comm")
    if [ ! -z "$pid" ]; then
        echo -e "${YELLOW}Stopping existing cellular_comm node (PID: $pid)...${NC}"
        kill $pid 2>/dev/null
        sleep 2
    fi
}

# Function to start enhanced cellular comm
start_enhanced_cellular() {
    echo -e "${GREEN}Starting enhanced cellular communication node...${NC}"
    echo -e "${BLUE}Features:${NC}"
    echo "  - Improved GPS data validation"
    echo "  - Better connection monitoring"
    echo "  - Enhanced error handling"
    echo "  - Debug mode for troubleshooting"
    echo ""
    
    # Start with debug mode if requested
    if [[ "$1" == "--debug" ]]; then
        echo -e "${YELLOW}Starting in DEBUG mode${NC}"
        ros2 run sensors cellular_comm_enhanced --ros-args -p debug_mode:=true &
    else
        ros2 run sensors cellular_comm &
    fi
    CELL_PID=$!
    
    sleep 3
    
    # Verify it started
    if ps -p $CELL_PID > /dev/null; then
        echo -e "${GREEN}Enhanced cellular comm node started (PID: $CELL_PID)${NC}"
        return 0
    else
        echo -e "${RED}Failed to start enhanced cellular comm node${NC}"
        return 1
    fi
}

# Function to monitor GPS and publishing
monitor_gps_publishing() {
    echo -e "${YELLOW}Starting GPS publishing monitor...${NC}"
    
    # Start the debug monitoring script in parallel
    python3 "$SCRIPT_DIR/debug_gps_publishing.py" &
    DEBUG_PID=$!
    
    # Monitor system status
    while true; do
        clear
        echo -e "${BLUE}===== GPS Publishing Status =====${NC}"
        echo -e "Time: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        
        # Check if cellular comm is running
        if ! ps -p $CELL_PID > /dev/null 2>&1; then
            echo -e "${RED}WARNING: Cellular comm node stopped! Restarting...${NC}"
            start_enhanced_cellular
        else
            echo -e "${GREEN}✓ Cellular comm node running (PID: $CELL_PID)${NC}"
        fi
        
        # Show WebSocket status
        echo -e "\n${YELLOW}WebSocket Status:${NC}"
        echo -e "URL: wss://sailbot-relay.onrender.com"
        echo -e "Auth: antonius"
        echo -e "Type: boat"
        
        # Show current topics
        echo -e "\n${YELLOW}Active GPS Topics:${NC}"
        ros2 topic list | grep -E "gps|boat_status" | while read topic; do
            echo -e "  ${GREEN}✓${NC} $topic"
        done
        
        echo -e "\n${YELLOW}Press Ctrl+C to stop${NC}"
        sleep 5
    done
}

# Main execution
echo -e "${YELLOW}Enhanced GPS Publishing Starting...${NC}"

# Stop any existing cellular comm
stop_existing_cellular

# Start enhanced cellular comm
if [[ "$1" == "--debug" ]]; then
    start_enhanced_cellular --debug
else
    start_enhanced_cellular
fi

# Set up signal handler
trap 'echo -e "\n${YELLOW}Shutting down...${NC}"; kill $CELL_PID $DEBUG_PID 2>/dev/null; echo -e "${GREEN}Stopped${NC}"; exit 0' SIGINT

# Start monitoring
monitor_gps_publishing