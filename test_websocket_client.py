#!/usr/bin/env python3
"""
Simple WebSocket client to test real-time ground station updates
This will help debug connection issues and data flow
"""

import socketio
import time
import json

class GroundStationTestClient:
    def __init__(self, url='http://localhost:5001'):
        self.sio = socketio.Client()
        self.url = url
        self.data_count = 0
        
        # Set up event handlers
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('telemetry_update', self.on_telemetry_update)
        self.sio.on('ahrs_data', self.on_ahrs_data)
        self.sio.on('flight_data', self.on_flight_data)
        self.sio.on('drone_telemetry', self.on_drone_telemetry)
        
    def connect(self):
        try:
            print(f"🔗 Connecting to {self.url}...")
            self.sio.connect(self.url)
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def on_connect(self):
        print("✅ Connected to ground station WebSocket!")
        print("📊 Listening for real-time telemetry data...")
        print("-" * 50)
    
    def on_disconnect(self):
        print("❌ Disconnected from ground station")
    
    def on_telemetry_update(self, data):
        self.data_count += 1
        if self.data_count % 10 == 0:  # Print every 10th update
            try:
                flight_data = data.get('flight_data', {})
                ahrs = data.get('ahrs', {})
                
                print(f"📡 Update #{self.data_count}: "
                      f"Roll={ahrs.get('roll_angle', 0):.2f}°, "
                      f"Pitch={ahrs.get('pitch_angle', 0):.2f}°, "
                      f"Alt={ahrs.get('altitude', 0):.2f}m")
            except Exception as e:
                print(f"⚠️  Error processing telemetry data: {e}")
    
    def on_ahrs_data(self, data):
        try:
            print(f"🎯 AHRS: Roll={data.get('roll_angle', 0):.2f}°, "
                  f"Pitch={data.get('pitch_angle', 0):.2f}°, "
                  f"Yaw={data.get('yaw_angle', 0):.2f}°")
        except Exception as e:
            print(f"⚠️  Error processing AHRS: {e}")
    
    def on_flight_data(self, data):
        try:
            print(f"✈️  Flight: Alt={data.get('barometric_altitude', 0):.1f}m, "
                  f"Roll={data.get('roll_angle', 0):.2f}°, "
                  f"Pitch={data.get('pitch_angle', 0):.2f}°")
        except Exception as e:
            print(f"⚠️  Error processing flight data: {e}")
    
    def on_drone_telemetry(self, data):
        try:
            print(f"🚁 Drone: Mode={data.get('flight_mode', 'N/A')}, "
                  f"Armed={data.get('armed_status', 'N/A')}, "
                  f"Battery={data.get('voltage_per_cell', 0)*4:.1f}V")
        except Exception as e:
            print(f"⚠️  Error processing drone telemetry: {e}")
    
    def run(self):
        """Keep connection alive and monitor data"""
        if self.connect():
            try:
                print("🔄 WebSocket client running... Press Ctrl+C to stop")
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n🛑 Stopping WebSocket client...")
            finally:
                self.sio.disconnect()
        else:
            print("❌ Failed to connect to ground station")

if __name__ == '__main__':
    client = GroundStationTestClient()
    client.run()
