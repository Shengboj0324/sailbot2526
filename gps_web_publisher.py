#!/usr/bin/env python3
"""
GPS Web Publisher - Reads GPS data from serial port and publishes to web server
"""

import serial
import time
import asyncio
import websockets
import json
import threading
import argparse
from datetime import datetime
import logging
import sys

# ANSI color codes for terminal output
COLORS = {
    'GREEN': '\033[92m',
    'YELLOW': '\033[93m',
    'RED': '\033[91m',
    'BLUE': '\033[94m',
    'CYAN': '\033[96m',
    'RESET': '\033[0m'
}

class GPSWebPublisher:
    def __init__(self, serial_port='/dev/ttyAMA0', baud_rate=38400, 
                 websocket_url='wss://sailbot-relay.onrender.com',
                 auth_token='antonius', debug=False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.websocket_url = websocket_url
        self.auth_token = auth_token
        self.debug = debug
        
        # GPS data storage
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'speed': 0.0,
            'fix_quality': 0,
            'timestamp': None,
            'valid': False
        }
        
        # Connection states
        self.serial_connected = False
        self.websocket_connected = False
        self.running = True
        
        # Setup logging
        logging.basicConfig(
            level=logging.DEBUG if debug else logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentences for GPS data"""
        try:
            # Remove any whitespace and check for valid sentence
            sentence = sentence.strip()
            if not sentence.startswith('$'):
                return
            
            # Split the sentence
            parts = sentence.split(',')
            
            # Parse GNGGA sentence (position and fix quality)
            if parts[0] in ['$GNGGA', '$GPGGA']:
                if len(parts) >= 10:
                    # Extract time
                    time_str = parts[1]
                    
                    # Extract latitude
                    lat = parts[2]
                    lat_dir = parts[3]
                    if lat and lat_dir:
                        lat_deg = float(lat[:2])
                        lat_min = float(lat[2:])
                        latitude = lat_deg + lat_min / 60
                        if lat_dir == 'S':
                            latitude = -latitude
                        self.gps_data['latitude'] = latitude
                    
                    # Extract longitude
                    lon = parts[4]
                    lon_dir = parts[5]
                    if lon and lon_dir:
                        lon_deg = float(lon[:3])
                        lon_min = float(lon[3:])
                        longitude = lon_deg + lon_min / 60
                        if lon_dir == 'W':
                            longitude = -longitude
                        self.gps_data['longitude'] = longitude
                    
                    # Extract fix quality
                    if parts[6]:
                        self.gps_data['fix_quality'] = int(parts[6])
                    
                    # Update validity
                    self.gps_data['valid'] = (
                        self.gps_data['latitude'] != 0.0 and 
                        self.gps_data['longitude'] != 0.0 and
                        self.gps_data['fix_quality'] > 0
                    )
                    
                    self.gps_data['timestamp'] = datetime.now().isoformat()
                    
            # Parse GNVTG sentence (speed)
            elif parts[0] in ['$GNVTG', '$GPVTG']:
                if len(parts) >= 8 and parts[7]:
                    # Speed in km/h, convert to m/s
                    speed_kmh = float(parts[7])
                    self.gps_data['speed'] = speed_kmh / 3.6
                    
        except Exception as e:
            if self.debug:
                self.logger.error(f"Error parsing NMEA: {e}")
    
    def read_gps_serial(self):
        """Read GPS data from serial port"""
        while self.running:
            try:
                # Connect to serial port
                self.logger.info(f"Connecting to GPS on {self.serial_port}...")
                ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                self.serial_connected = True
                self.logger.info(f"{COLORS['GREEN']}GPS serial connected{COLORS['RESET']}")
                
                # Read data
                while self.running and self.serial_connected:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore')
                        if line:
                            self.parse_nmea_sentence(line)
                            
                            # Print status based on fix quality
                            if self.gps_data['valid']:
                                fix_color = COLORS['GREEN'] if self.gps_data['fix_quality'] >= 4 else COLORS['YELLOW']
                                self.logger.info(
                                    f"{fix_color}GPS: {self.gps_data['latitude']:.6f}, "
                                    f"{self.gps_data['longitude']:.6f} | "
                                    f"Speed: {self.gps_data['speed']:.2f} m/s | "
                                    f"Fix: {self.gps_data['fix_quality']}{COLORS['RESET']}"
                                )
                    except Exception as e:
                        if self.debug:
                            self.logger.error(f"Serial read error: {e}")
                            
            except Exception as e:
                self.serial_connected = False
                self.logger.error(f"{COLORS['RED']}GPS serial error: {e}{COLORS['RESET']}")
                time.sleep(5)  # Wait before retry
    
    async def publish_to_websocket(self):
        """Publish GPS data to websocket server"""
        while self.running:
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    self.websocket_connected = True
                    self.logger.info(f"{COLORS['GREEN']}WebSocket connected to {self.websocket_url}{COLORS['RESET']}")
                    
                    # Send authentication
                    auth_msg = json.dumps({
                        "type": "auth",
                        "token": self.auth_token,
                        "connectionType": "boat"
                    })
                    await websocket.send(auth_msg)
                    
                    # Wait for auth response
                    response = await websocket.recv()
                    self.logger.info(f"Auth response: {response}")
                    
                    # Send GPS data periodically
                    while self.running and self.websocket_connected:
                        if self.gps_data['valid']:
                            # Format GPS message
                            gps_msg = f"GPS,{self.gps_data['latitude']},{self.gps_data['longitude']}," \
                                     f"{self.gps_data['speed']},{self.gps_data['fix_quality']}"
                            
                            await websocket.send(gps_msg)
                            self.logger.info(f"{COLORS['CYAN']}Sent: {gps_msg}{COLORS['RESET']}")
                        else:
                            self.logger.warning(f"{COLORS['YELLOW']}Waiting for valid GPS fix...{COLORS['RESET']}")
                        
                        await asyncio.sleep(5)  # Send updates every 5 seconds
                        
            except Exception as e:
                self.websocket_connected = False
                self.logger.error(f"{COLORS['RED']}WebSocket error: {e}{COLORS['RESET']}")
                await asyncio.sleep(5)  # Wait before reconnect
    
    def run(self):
        """Run the GPS web publisher"""
        self.logger.info(f"{COLORS['BLUE']}Starting GPS Web Publisher{COLORS['RESET']}")
        
        # Start GPS serial reading thread
        serial_thread = threading.Thread(target=self.read_gps_serial, daemon=True)
        serial_thread.start()
        
        # Run websocket publisher
        try:
            asyncio.run(self.publish_to_websocket())
        except KeyboardInterrupt:
            self.logger.info(f"{COLORS['YELLOW']}Shutting down...{COLORS['RESET']}")
            self.running = False

def main():
    parser = argparse.ArgumentParser(description='GPS Web Publisher')
    parser.add_argument('--serial-port', default='/dev/ttyAMA0',
                       help='GPS serial port (default: /dev/ttyAMA0)')
    parser.add_argument('--baud-rate', type=int, default=38400,
                       help='Serial baud rate (default: 38400)')
    parser.add_argument('--websocket-url', default='wss://sailbot-relay.onrender.com',
                       help='WebSocket server URL')
    parser.add_argument('--auth-token', default='antonius',
                       help='Authentication token')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    publisher = GPSWebPublisher(
        serial_port=args.serial_port,
        baud_rate=args.baud_rate,
        websocket_url=args.websocket_url,
        auth_token=args.auth_token,
        debug=args.debug
    )
    
    publisher.run()

if __name__ == '__main__':
    main()