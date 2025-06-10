#!/usr/bin/env python3
import serial
import websocket
import time
import sys
import argparse

class GPSWebPublisher:
    def __init__(self, serial_port='/dev/ttyAMA0', baud_rate=38400):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ws_url = 'wss://sailbot-relay.onrender.com?type=boat&auth=antonius'
        self.ws = None
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'speed': 0.0,
            'fix_quality': 0
        }
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            print(f"Opened {self.serial_port} @ {self.baud_rate}")
            return True
        except Exception as e:
            print(f"Error opening {self.serial_port}: {e}")
            return False
    
    def connect_websocket(self):
        try:
            self.ws = websocket.create_connection(self.ws_url)
            print("WebSocket connected")
            # Send initial message
            self.ws.send(bytes([0xB0, 0x55, 0xAA]), opcode=websocket.ABNF.OPCODE_BINARY)
            return True
        except Exception as e:
            print(f"WebSocket error: {e}")
            return False
    
    def parse_nmea(self, line):
        parts = line.split(',')
        
        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            try:
                # Fix quality
                if len(parts) > 6 and parts[6]:
                    self.gps_data['fix_quality'] = int(parts[6])
                
                # Latitude
                if len(parts) > 3 and parts[2] and parts[3]:
                    lat = float(parts[2][:2]) + float(parts[2][2:])/60
                    if parts[3] == 'S':
                        lat = -lat
                    self.gps_data['latitude'] = lat
                
                # Longitude
                if len(parts) > 5 and parts[4] and parts[5]:
                    lon = float(parts[4][:3]) + float(parts[4][3:])/60
                    if parts[5] == 'W':
                        lon = -lon
                    self.gps_data['longitude'] = lon
                    
            except:
                pass
                
        elif line.startswith('$GNVTG') or line.startswith('$GPVTG'):
            try:
                # Speed in km/h
                if len(parts) > 7 and parts[7]:
                    km_h = float(parts[7])
                    self.gps_data['speed'] = km_h / 3.6  # Convert to m/s
            except:
                pass
    
    def send_gps_data(self):
        if self.ws:
            try:
                # Format: GPS,lat,lon,speed,fix_quality
                msg = f"GPS,{self.gps_data['latitude']:.6f},{self.gps_data['longitude']:.6f},{self.gps_data['speed']:.2f},{self.gps_data['fix_quality']}\n"
                self.ws.send(msg.encode('utf-8'))
                print(f"Sent: {msg.strip()}")
            except Exception as e:
                print(f"Send error: {e}")
                self.ws = None
    
    def run(self):
        if not self.connect_serial():
            sys.exit(1)
        
        last_send = 0
        
        while True:
            try:
                # Read GPS data
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and line.startswith('$'):
                    self.parse_nmea(line)
                
                # Send every 5 seconds
                now = time.time()
                if now - last_send >= 5:
                    # Connect/reconnect websocket if needed
                    if not self.ws:
                        self.connect_websocket()
                    
                    # Send data if we have a fix
                    if self.gps_data['fix_quality'] > 0:
                        self.send_gps_data()
                    
                    last_send = now
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(1)
        
        if self.ws:
            self.ws.close()
        self.ser.close()
        print("Closed")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', default='/dev/ttyAMA0', help='GPS serial port')
    parser.add_argument('-b', '--baud', type=int, default=38400, help='Baud rate')
    args = parser.parse_args()
    
    publisher = GPSWebPublisher(args.port, args.baud)
    publisher.run()

if __name__ == '__main__':
    main()