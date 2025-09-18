#!/usr/bin/env python3
"""
M-HIVE Ground Station - Custom Implementation
Based on FC ↔ GCS Data Protocol v0.9.1

Features:
- Serial communication with 3DR Telemetry
- Real-time data visualization
- PID gain control
- GPS mapping
- Web-based interface
"""

import serial
import struct
import time
import json
import threading
import logging
import math
from datetime import datetime
from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit
import serial.tools.list_ports

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class KMUGroundStation:
    def __init__(self):
        self.serial_port = None
        self.is_connected = False
        self.data_buffer = []
        self.latest_data = {
            'ahrs': {},
            'gps': {},
            'pid_gains': {},
            'connection_status': 'Disconnected',
            'battery_voltage': 0.0,
            'switches': {'swa': 0, 'swc': 0, 'failsafe': 0},
            'flight_mode': 'MANUAL',
            'mission_waypoints': [],
            # Enhanced telemetry data
            'flight_data': {
                'roll_angle': 0.0,
                'pitch_angle': 0.0,
                'yaw_angle': 0.0,
                'barometric_altitude': 0.0,
                'roll_setpoint': 0.0,
                'pitch_setpoint': 0.0,
                'yaw_setpoint': 0.0,
                'altitude_setpoint': 0.0
            },
            'navigation_data': {
                'gps_latitude': 0.0,
                'gps_longitude': 0.0,
                'gps_fix': False,
                'gps_satellites': 0
            },
            'power_system': {
                'battery_voltage': 0.0,
                'battery_percentage': 0.0,
                'low_battery_warning': False
            },
            'remote_control': {
                'ibus_swa': 0,  # 0=Up, 1=Down
                'ibus_swc': 0,  # 0=Up, 1=Mid, 2=Down
                'failsafe_status': 0,  # 0=Normal, 1=Triggered, 2=No iBus data
                'failsafe_triggered': False
            },
            'system_status': {
                'data_rate_ahrs': 0,  # Hz
                'data_rate_gps': 0,   # Hz
                'last_ahrs_update': None,
                'last_gps_update': None,
                'connection_quality': 0.0
            },
            # New telemetry data structures
            'raw_sensor_data': {
                'gyro_x_raw': 0,
                'gyro_y_raw': 0,
                'gyro_z_raw': 0,
                'pressure_raw': 0,
                'temperature_raw': 0.0,
                'mag_x_raw': 0,
                'mag_y_raw': 0,
                'mag_z_raw': 0
            },
            'processed_sensor_data': {
                'baro_alt_filtered': 0.0,
                'quaternion_q1': 0.0,
                'quaternion_q2': 0.0,
                'quaternion_q3': 0.0,
                'quaternion_q4': 0.0,
                'quat_radian_accuracy': 0.0,
                'magnetometer_accuracy': 0  # 0-3: Unreliable to High
            },
            'motor_commands': {
                'motor_1': 0,  # ccr1
                'motor_2': 0,  # ccr2
                'motor_3': 0,  # ccr3
                'motor_4': 0   # ccr4
            },
            'pid_debug': {
                'pitch_pid_result': 0.0,
                'roll_pid_result': 0.0,
                'yaw_pid_result': 0.0,
                'pitch_error': 0.0,
                'roll_error': 0.0,
                'yaw_error': 0.0
            },
            'system_flags': {
                'motor_arming_flag': False,
                'low_bat_flag': False,
                'failsafe_flag': 0  # 0=Normal, 1=Triggered, 2=No data
            },
            'rc_receiver_data': {
                'channel_rh': 0,  # Right Horizontal
                'channel_rv': 0,  # Right Vertical
                'channel_lh': 0,  # Left Horizontal
                'channel_lv': 0,  # Left Vertical
                'channel_swa': 0, # Switch A
                'channel_swc': 0  # Switch C
            }
        }
        
        # Initialize data logging
        self.data_logging_enabled = False
        self.log_file = None
        self.log_start_time = None
        
        # PID Gain storage
        self.pid_gains = {
            'roll_inner': {'p': 0.0, 'i': 0.0, 'd': 0.0},
            'roll_outer': {'p': 0.0, 'i': 0.0, 'd': 0.0},
            'pitch_inner': {'p': 0.0, 'i': 0.0, 'd': 0.0},
            'pitch_outer': {'p': 0.0, 'i': 0.0, 'd': 0.0},
            'yaw_angle': {'p': 0.0, 'i': 0.0, 'd': 0.0},
            'yaw_rate': {'p': 0.0, 'i': 0.0, 'd': 0.0}
        }
        
        # Initialize Flask app
        import os
        template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
        static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
        self.app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
        self.app.config['SECRET_KEY'] = 'kmu_ground_station'
        
        # Initialize SocketIO with simpler configuration
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Setup routes
        self.setup_routes()
        
        # No auto-test data generation - only real FC data
        self.test_mode = False
        
        # Settings storage (remember COM port)
        self.settings = {
            'com_port': 'COM14',  # Default port
            'baud_rate': 115200,
            'last_connected_port': None
        }
        
        # Load saved settings
        self.load_settings()
        
    def load_settings(self):
        """Load saved settings from file"""
        try:
            with open('settings.json', 'r') as f:
                saved_settings = json.load(f)
                self.settings.update(saved_settings)
                logger.info(f"Loaded settings: {self.settings}")
        except FileNotFoundError:
            logger.info("No saved settings found, using defaults")
        except Exception as e:
            logger.error(f"Error loading settings: {e}")
    
    def save_settings(self):
        """Save current settings to file"""
        try:
            with open('settings.json', 'w') as f:
                json.dump(self.settings, f, indent=2)
                logger.info(f"Saved settings: {self.settings}")
        except Exception as e:
            logger.error(f"Error saving settings: {e}")
        
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
            
        @self.app.route('/api/ports')
        def get_ports():
            ports = [port.device for port in serial.tools.list_ports.comports()]
            return jsonify({
                'ports': ports,
                'saved_port': self.settings.get('com_port', 'COM14'),
                'saved_baudrate': self.settings.get('baud_rate', 115200)
            })
            
        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            data = request.json
            port = data.get('port')
            baudrate = int(data.get('baudrate', 57600))
            
            # Save settings
            self.settings['com_port'] = port
            self.settings['baud_rate'] = baudrate
            self.settings['last_connected_port'] = port
            self.save_settings()
            
            try:
                self.serial_port = serial.Serial(port, baudrate, timeout=1)
                self.is_connected = True
                self.latest_data['connection_status'] = 'Connected'
                
                # Disable test mode when real FC connects
                self.test_mode = False
                logger.info(f"Real FC connected")
                
                # Start data reading thread
                self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
                
                # Automatically start AHRS data logging when FC connects
                if not self.data_logging_enabled:
                    logging_result = self.start_data_logging()
                    if logging_result['status'] == 'success':
                        logger.info("📊 Automatic AHRS data logging started")
                    else:
                        logger.warning(f"Failed to start automatic logging: {logging_result.get('message', 'Unknown error')}")
                
                logger.info(f"Connected to {port} at {baudrate} baud")
                return jsonify({'status': 'success', 'message': f'Connected to {port}'})
            except Exception as e:
                logger.error(f"Connection failed: {e}")
                return jsonify({'status': 'error', 'message': str(e)})
                
        @self.app.route('/api/disconnect')
        def disconnect():
            # Stop AHRS data logging if active
            if self.data_logging_enabled:
                logging_result = self.stop_data_logging()
                if logging_result['status'] == 'success':
                    logger.info("📊 AHRS data logging stopped due to disconnect")
                else:
                    logger.warning(f"Failed to stop logging: {logging_result.get('message', 'Unknown error')}")
            
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.latest_data['connection_status'] = 'Disconnected'
            
            # Re-enable test mode when FC disconnects
            self.test_mode = False
            logger.info("FC disconnected")
            
            logger.info("Disconnected from serial port")
            return jsonify({'status': 'success', 'message': 'Disconnected'})
            
        @self.app.route('/api/telemetry')
        def get_telemetry():
            """Get comprehensive telemetry data"""
            return jsonify({
                'flight_data': self.latest_data['flight_data'],
                'navigation_data': self.latest_data['navigation_data'],
                'power_system': self.latest_data['power_system'],
                'remote_control': self.latest_data['remote_control'],
                'system_status': self.latest_data['system_status'],
                'pid_gains': self.latest_data['pid_gains']
            })
            
        @self.app.route('/api/navigation')
        def get_navigation():
            """Get navigation data (10Hz)"""
            return jsonify(self.latest_data['navigation_data'])
            
        @self.app.route('/api/power')
        def get_power():
            """Get power system data"""
            return jsonify(self.latest_data['power_system'])
            
        @self.app.route('/api/remote_control')
        def get_remote_control():
            """Get remote control status"""
            return jsonify(self.latest_data['remote_control'])
            
        @self.app.route('/api/system')
        def get_system():
            """Get system status and data rates"""
            return jsonify(self.latest_data['system_status'])
        
        @self.app.route('/api/start_logging', methods=['POST'])
        def start_logging():
            """Start AHRS data logging"""
            result = self.start_data_logging()
            return jsonify(result)
        
        @self.app.route('/api/stop_logging', methods=['POST'])
        def stop_logging():
            """Stop AHRS data logging"""
            result = self.stop_data_logging()
            return jsonify(result)
        
        @self.app.route('/api/logging_status')
        def get_logging_status():
            """Get current logging status"""
            return jsonify({
                'enabled': self.data_logging_enabled,
                'filename': self.log_file.name if self.log_file else None,
                'start_time': self.log_start_time.isoformat() if self.log_start_time else None,
                'data_points_logged': getattr(self, 'data_points_logged', 0),
                'log_directory': 'Sensor Log'
            })
            
        @self.app.route('/api/status')
        def get_status():
            """Get current connection and data status"""
            return jsonify({
                'connection_status': self.latest_data['connection_status'],
                'is_connected': self.is_connected,
                'ahrs_data': self.latest_data['ahrs'],
                'gps_data': self.latest_data['gps'],
                'battery_voltage': self.latest_data['battery_voltage'],
                'switches': self.latest_data['switches']
            })
            
        @self.app.route('/api/all_data')
        def get_all_data():
            """Get all available data"""
            return jsonify(self.latest_data)
            
        @self.app.route('/api/send_pid', methods=['POST'])
        def send_pid():
            data = request.json
            gain_type = data.get('type')
            p = float(data.get('p', 0))
            i = float(data.get('i', 0))
            d = float(data.get('d', 0))
            
            logger.info(f"Sending PID gains: type={gain_type}, P={p}, I={i}, D={d}")
            
            if self.send_pid_gain(gain_type, p, i, d):
                logger.info(f"Successfully sent {gain_type} PID gains")
                return jsonify({'status': 'success', 'message': f'Sent {gain_type} PID gains'})
            else:
                logger.error(f"Failed to send {gain_type} PID gains")
                return jsonify({'status': 'error', 'message': 'Failed to send PID gains'})
        
        @self.app.route('/api/pid_gains')
        def get_pid_gains():
            """Get current PID gains from FC"""
            try:
                return jsonify({
                    'status': 'success',
                    'pid_gains': self.latest_data['pid_gains']
                })
            except Exception as e:
                logger.error(f"Error getting PID gains: {e}")
                return jsonify({'status': 'error', 'message': str(e)})

        @self.app.route('/api/set_flight_mode', methods=['POST'])
        def set_flight_mode():
            """Set flight mode"""
            try:
                data = request.get_json()
                mode = data.get('mode', 'MANUAL')
                
                # Update latest data
                self.latest_data['flight_mode'] = mode
                
                # Send flight mode command to FC
                if self.is_connected and self.serial_port:
                    # This would send the actual MAVLink command
                    # For now, just log the command
                    logger.info(f"Setting flight mode to: {mode}")
                    
                    return jsonify({
                        'status': 'success',
                        'message': f'Flight mode set to {mode}',
                        'mode': mode
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Not connected to flight controller'
                    })
            except Exception as e:
                logger.error(f"Set flight mode error: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500

        @self.app.route('/api/upload_mission', methods=['POST'])
        def upload_mission():
            """Upload mission waypoints to flight controller"""
            try:
                data = request.get_json()
                waypoints = data.get('waypoints', [])
                
                if not waypoints:
                    return jsonify({
                        'status': 'error',
                        'message': 'No waypoints provided'
                    })
                
                # Store waypoints
                self.latest_data['mission_waypoints'] = waypoints
                
                # Send mission to FC
                if self.is_connected and self.serial_port:
                    # This would send the actual MAVLink mission upload commands
                    logger.info(f"Uploading mission with {len(waypoints)} waypoints")
                    
                    return jsonify({
                        'status': 'success',
                        'message': f'Mission uploaded with {len(waypoints)} waypoints',
                        'waypoints': waypoints
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Not connected to flight controller'
                    })
            except Exception as e:
                logger.error(f"Upload mission error: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500

        @self.app.route('/api/download_mission', methods=['GET'])
        def download_mission():
            """Download mission waypoints from flight controller"""
            try:
                if self.is_connected and self.serial_port:
                    # This would request mission from FC
                    waypoints = self.latest_data.get('mission_waypoints', [])
                    
                    return jsonify({
                        'status': 'success',
                        'message': f'Mission downloaded with {len(waypoints)} waypoints',
                        'waypoints': waypoints
                    })
                else:
                    return jsonify({
                        'status': 'error',
                        'message': 'Not connected to flight controller'
                    })
            except Exception as e:
                logger.error(f"Download mission error: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
    
        @self.app.route('/api/flight_data')
        def get_flight_data():
            """Get current flight data"""
            try:
                # Debug logging
                logger.info(f"API request - flight_data: {self.latest_data.get('flight_data', {})}")
                logger.info(f"API request - ahrs: {self.latest_data.get('ahrs', {})}")
                
                return jsonify({
                    'status': 'success',
                    'data': {
                        'roll_angle': self.latest_data['flight_data'].get('roll_angle', 0),
                        'pitch_angle': self.latest_data['flight_data'].get('pitch_angle', 0),
                        'yaw_angle': self.latest_data['flight_data'].get('yaw_angle', 0),
                        'barometric_altitude': self.latest_data['flight_data'].get('barometric_altitude', 0),
                        'battery_voltage': self.latest_data['power_system'].get('battery_voltage', 0),
                        'connection_status': self.latest_data.get('connection_status', 'Disconnected'),
                        'switches': {
                            'swa': self.latest_data.get('switches', {}).get('swa', 0),
                            'swc': self.latest_data.get('switches', {}).get('swc', 0)
                        }
                    }
                })
            except Exception as e:
                logger.error(f"Error getting flight data: {e}")
                return jsonify({'status': 'error', 'message': str(e)})
                
        @self.app.route('/api/request_pid', methods=['POST'])
        def request_pid():
            data = request.json
            gain_type = data.get('type', 6)  # Default to all gains
            
            if self.request_pid_gain(gain_type):
                return jsonify({'status': 'success', 'message': f'Requested PID gains type {gain_type}'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to request PID gains'})
                
        @self.app.route('/api/send_terminal', methods=['POST'])
        def send_terminal():
            data = request.json
            message = data.get('message')
            mode = data.get('mode', 'ascii')  # ascii or hex
            
            if self.send_terminal_message(message, mode):
                return jsonify({'status': 'success', 'message': 'Message sent'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to send message'})
    
        @self.app.route('/api/connection_status')
        def get_connection_status():
            """Get detailed connection status"""
            try:
                return jsonify({
                    'is_connected': self.is_connected,
                    'serial_port': self.serial_port.device if self.serial_port else None,
                    'port_open': self.serial_port.is_open if self.serial_port else False,
                    'baud_rate': self.serial_port.baudrate if self.serial_port else None,
                    'status': 'success'
                })
            except Exception as e:
                logger.error(f"Error getting connection status: {e}")
                return jsonify({
                    'is_connected': False,
                    'serial_port': None,
                    'port_open': False,
                    'baud_rate': None,
                    'status': 'error',
                    'message': str(e)
                }), 500
    
    def reconnect_serial(self):
        """Attempt to reconnect to the serial port"""
        try:
            logger.info("Attempting to reconnect to serial port...")
            
            # Close existing connection if any
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
                self.serial_port = None
            
            # Try to find available ports
            available_ports = list(serial.tools.list_ports.comports())
            if not available_ports:
                logger.warning("No serial ports available")
                return False
            
            # Try to connect to the first available port
            for port_info in available_ports:
                try:
                    logger.info(f"Trying to connect to {port_info.device}")
                    self.serial_port = serial.Serial(
                        port=port_info.device,
                        baudrate=115200,
                        timeout=1,
                        write_timeout=1
                    )
                    
                    if self.serial_port.is_open:
                        logger.info(f"Successfully reconnected to {port_info.device}")
                        self.is_connected = True
                        self.latest_data['connection_status'] = 'Connected'
                        return True
                        
                except Exception as e:
                    logger.warning(f"Failed to connect to {port_info.device}: {e}")
                    continue
            
            logger.error("Failed to reconnect to any serial port")
            return False
            
        except Exception as e:
            logger.error(f"Reconnection error: {e}")
            return False
    
    def calculate_checksum(self, data, checksum_index=None):
        """Calculate checksum: 0xFF - (sum of bytes)"""
        if len(data) < 1:
            return 0
        
        # If checksum_index is provided, sum up to that index
        if checksum_index is not None:
            sum_bytes = sum(data[:checksum_index])
        else:
            # Default: sum all bytes except the last one (original behavior)
            sum_bytes = sum(data[:-1])
            
        checksum = 0xFF - sum_bytes & 0xFF
        return checksum
    
    def send_message(self, message_id, data):
        """Send a message to the flight controller"""
        if not self.is_connected or not self.serial_port:
            logger.error(f"Cannot send message: connected={self.is_connected}, port={self.serial_port is not None}")
            return False
            
        if not self.serial_port.is_open:
            logger.error("Cannot send message: Serial port is closed")
            return False
            
        # Create message frame: GS + ID + data + checksum
        message = bytearray([0x47, 0x53, message_id])  # GS + ID
        message.extend(data)
        
        # Pad to 19 bytes if needed
        while len(message) < 19:
            message.append(0x00)
            
        # Add checksum
        # CHKSUM = 0xFF - (BYTE0~BYTE18) according to protocol
        sum_bytes = sum(message)
        checksum = (0xFF - sum_bytes) & 0xFF  # Handle underflow
        message.append(checksum)
        
        try:
            bytes_written = self.serial_port.write(message)
            logger.info(f"Sent message: {message.hex()} ({bytes_written} bytes written)")
            return True
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
            return False
    
    def send_pid_gain(self, gain_type, p, i, d):
        """Send PID gain to flight controller"""
        logger.info(f"send_pid_gain called: type={gain_type}, P={p}, I={i}, D={d}")
        
        # Check connection status
        if not self.is_connected:
            logger.error("Cannot send PID: Not connected to FC")
            return False
            
        if not self.serial_port:
            logger.error("Cannot send PID: No serial port available")
            return False
            
        if not self.serial_port.is_open:
            logger.error("Cannot send PID: Serial port is closed")
            return False
        
        gain_map = {
            'roll_inner': 0x00,
            'roll_outer': 0x01,
            'pitch_inner': 0x02,
            'pitch_outer': 0x03,
            'yaw_angle': 0x04,
            'yaw_rate': 0x05
        }
        
        if gain_type not in gain_map:
            logger.error(f"Invalid gain type: {gain_type}")
            return False
            
        message_id = gain_map[gain_type]
        data = struct.pack('<fff', p, i, d)  # Pack as little-endian floats
        data += b'\x00' * 12  # Pad to 15 bytes
        
        logger.info(f"Sending PID message: ID=0x{message_id:02X}, data={data.hex()}")
        logger.info(f"Connection status: connected={self.is_connected}, port_open={self.serial_port.is_open if self.serial_port else False}")
        
        result = self.send_message(message_id, data)
        logger.info(f"Send result: {result}")
        return result
    
    def request_pid_gain(self, gain_type):
        """Request PID gain from flight controller"""
        data = bytearray([gain_type])
        data.extend([0x00] * 15)  # Pad to 16 bytes
        
        return self.send_message(0x10, data)
    
    def send_terminal_message(self, message, mode='ascii'):
        """Send terminal message (ASCII or HEX)"""
        if not self.is_connected or not self.serial_port:
            return False
            
        try:
            if mode == 'hex':
                # Convert hex string to bytes
                message = message.strip()
                data = bytes.fromhex(message)
            else:
                # ASCII mode
                data = message.encode('ascii')
                
            self.serial_port.write(data)
            logger.info(f"Terminal sent ({mode}): {message}")
            return True
        except Exception as e:
            logger.error(f"Terminal send failed: {e}")
            return False
    
    def parse_ahrs_message(self, data):
        """Parse AHRS message (ID 0x10) - 50Hz update rate according to protocol"""
        # Handle both 16-byte (full) and 8-byte (sensor only) formats
        if len(data) < 8:  # Minimum required for sensor data
            logger.warning(f"AHRS data too short: {len(data)} bytes")
            return None
            
        try:
            # Log raw data for debugging
            logger.info(f"Parsing AHRS data: {data.hex()}")
            
            # Parse sensor data (first 8 bytes)
            roll_angle = struct.unpack('<h', data[0:2])[0] / 100.0
            pitch_angle = struct.unpack('<h', data[2:4])[0] / 100.0
            yaw_angle = struct.unpack('<H', data[4:6])[0] / 100.0
            altitude = struct.unpack('<h', data[6:8])[0] / 10.0
            
            # Parse setpoint data if available (next 8 bytes)
            if len(data) >= 16:
                roll_setpoint = struct.unpack('<h', data[8:10])[0] / 100.0
                pitch_setpoint = struct.unpack('<h', data[10:12])[0] / 100.0
                yaw_setpoint = struct.unpack('<H', data[12:14])[0] / 100.0
                altitude_setpoint = struct.unpack('<h', data[14:16])[0] / 10.0
            else:
                # Use current values as setpoints if not available
                roll_setpoint = roll_angle
                pitch_setpoint = pitch_angle
                yaw_setpoint = yaw_angle
                altitude_setpoint = altitude
            
            # Validate reasonable ranges
            if abs(roll_angle) > 180 or abs(pitch_angle) > 180 or abs(yaw_angle) > 360:
                logger.warning(f"Invalid angle values: roll={roll_angle}, pitch={pitch_angle}, yaw={yaw_angle}")
                return None
            
            # Calculate data rate
            current_time = datetime.now()
            if hasattr(self, 'last_ahrs_time'):
                time_diff = (current_time - self.last_ahrs_time).total_seconds()
                if time_diff > 0:
                    self.latest_data['system_status']['data_rate_ahrs'] = 1.0 / time_diff
            self.last_ahrs_time = current_time
            
            logger.info(f"Successfully parsed AHRS: roll={roll_angle}, pitch={pitch_angle}, yaw={yaw_angle}, alt={altitude}")
            
            return {
                'roll_angle': roll_angle,
                'pitch_angle': pitch_angle,
                'yaw_angle': yaw_angle,
                'altitude': altitude,
                'roll_setpoint': roll_setpoint,
                'pitch_setpoint': pitch_setpoint,
                'yaw_setpoint': yaw_setpoint,
                'altitude_setpoint': altitude_setpoint,
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse AHRS message: {e}")
            return None
    
    def parse_gps_message(self, data):
        """Parse GPS message (ID 0x11) - According to protocol specification"""
        if len(data) < 16:  # Minimum required for GPS data according to protocol
            logger.warning(f"GPS data too short: {len(data)} bytes")
            return None
            
        try:
            logger.info(f"Parsing GPS data: {data.hex()}")
            
            # Check if this is NMEA data (starts with $)
            if data.startswith(b'$'):
                return self.parse_nmea_gps(data)
            
            # Parse GPS coordinates according to protocol specification
            # Bytes 3-6: GPS Latitude (long, scale factor 10^7)
            latitude_raw = struct.unpack('<l', data[0:4])[0]
            latitude = latitude_raw / 10000000.0
            
            # Bytes 7-10: GPS Longitude (long, scale factor 10^7)  
            longitude_raw = struct.unpack('<l', data[4:8])[0]
            longitude = longitude_raw / 10000000.0
            
            # Bytes 11-12: Battery Voltage (unsigned short, scale factor 100)
            battery_voltage = struct.unpack('<H', data[8:10])[0] / 100.0
            
            # Byte 13: iBus.SwA (0=Up, 1=Down)
            swa = data[10] if len(data) > 10 else 0
            
            # Byte 14: iBus.SwC (0=Up, 1=Mid, 2=Down)
            swc = data[11] if len(data) > 11 else 0
            
            # Byte 15: FS-i6 Fail-safe (0=Normal, 1=Triggered, 2=No iBus data)
            failsafe = data[12] if len(data) > 12 else 0
            
            # Altitude not available in GPS message according to protocol
            altitude = 0.0
            
            # Enhanced GPS debugging
            logger.info(f"Raw GPS bytes - Lat: {data[0:4].hex()}, Lon: {data[4:8].hex()}")
            logger.info(f"Parsed coordinates - Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")
            
            # Check for GPS fix status
            if latitude == 0.0 and longitude == 0.0:
                logger.warning("GPS coordinates are zero - likely no GPS fix or module not connected")
                logger.info("GPS troubleshooting: Check GPS module connection, power, and wait for satellite fix")
                logger.info("For M8N: Ensure UBX NAV-POSLLH messages are being sent")
                logger.info("If using NMEA GPS, ensure proper NMEA sentence configuration")
            
            # Validate reasonable ranges
            if abs(latitude) > 90 or abs(longitude) > 180:
                logger.warning(f"Invalid GPS coordinates: lat={latitude}, lon={longitude}")
                return None
            
            current_time = datetime.now()
            
            logger.info(f"Successfully parsed GPS: lat={latitude}, lon={longitude}, alt={altitude}, bat={battery_voltage}")
            
            return {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'battery_voltage': battery_voltage,
                'swa': swa,
                'swc': swc,
                'failsafe': failsafe,
                'failsafe_triggered': failsafe == 1,  # True if failsafe is triggered
                'gps_fix': 1 if latitude != 0 and longitude != 0 else 0,
                'gps_satellites': 0,  # Not available in this format
                'battery_percentage': min(100, max(0, (battery_voltage - 3.0) * 100 / (4.2 - 3.0))),
                'low_battery_warning': battery_voltage < 3.5,
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse GPS message: {e}")
            return None
    
    def parse_nmea_gps(self, data):
        """Parse NMEA GPS sentences (GPGGA, GPRMC, GPGSV)"""
        try:
            # Convert bytes to string
            nmea_sentence = data.decode('ascii', errors='ignore').strip()
            logger.info(f"Parsing NMEA sentence: {nmea_sentence}")
            
            # Check for valid NMEA sentence format
            if not nmea_sentence.startswith('$') or '*' not in nmea_sentence:
                logger.warning(f"Invalid NMEA sentence format: {nmea_sentence}")
                return None
            
            # Extract sentence type
            sentence_type = nmea_sentence[1:6]  # Skip $ and get next 5 chars
            
            if sentence_type == 'GPGGA':
                return self.parse_gpgga(nmea_sentence)
            elif sentence_type == 'GPRMC':
                return self.parse_gprmc(nmea_sentence)
            elif sentence_type == 'GPGSV':
                return self.parse_gpgsv(nmea_sentence)
            else:
                logger.info(f"Unsupported NMEA sentence type: {sentence_type}")
                return None
                
        except Exception as e:
            logger.error(f"Failed to parse NMEA GPS: {e}")
            return None
    
    def parse_gpgga(self, sentence):
        """Parse GPGGA (Global Positioning System Fix Data) sentence"""
        try:
            fields = sentence.split(',')
            if len(fields) < 15:
                logger.warning(f"GPGGA sentence too short: {len(fields)} fields")
                return None
            
            # Extract GPS fix data
            time_str = fields[1] if fields[1] else "000000.000"
            latitude_str = fields[2] if fields[2] else "0"
            latitude_dir = fields[3] if fields[3] else "N"
            longitude_str = fields[4] if fields[4] else "0"
            longitude_dir = fields[5] if fields[5] else "E"
            fix_quality = int(fields[6]) if fields[6] else 0
            satellites = int(fields[7]) if fields[7] else 0
            hdop = float(fields[8]) if fields[8] else 0.0
            altitude_str = fields[9] if fields[9] else "0"
            altitude_unit = fields[10] if fields[10] else "M"
            
            # Convert latitude from DDMM.MMMM format to decimal degrees
            if latitude_str and latitude_str != "0":
                lat_deg = float(latitude_str[:2])
                lat_min = float(latitude_str[2:])
                latitude = lat_deg + lat_min / 60.0
                if latitude_dir == 'S':
                    latitude = -latitude
            else:
                latitude = 0.0
            
            # Convert longitude from DDDMM.MMMM format to decimal degrees
            if longitude_str and longitude_str != "0":
                lon_deg = float(longitude_str[:3])
                lon_min = float(longitude_str[3:])
                longitude = lon_deg + lon_min / 60.0
                if longitude_dir == 'W':
                    longitude = -longitude
            else:
                longitude = 0.0
            
            # Convert altitude
            altitude = float(altitude_str) if altitude_str else 0.0
            
            # Get battery voltage from other data (not available in GPGGA)
            battery_voltage = 11.5  # Default value
            
            current_time = datetime.now()
            
            logger.info(f"GPGGA parsed - Lat: {latitude}, Lon: {longitude}, Alt: {altitude}, Sat: {satellites}, Fix: {fix_quality}")
            
            return {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'battery_voltage': battery_voltage,
                'swa': 0,  # Not available in NMEA
                'swc': 0,  # Not available in NMEA
                'failsafe': 0,  # Not available in NMEA
                'failsafe_triggered': False,
                'gps_fix': fix_quality,
                'gps_satellites': satellites,
                'battery_percentage': min(100, max(0, (battery_voltage - 3.0) / 1.2 * 100)),
                'low_battery_warning': battery_voltage < 3.5,
                'timestamp': current_time.isoformat()
            }
            
        except Exception as e:
            logger.error(f"Failed to parse GPGGA: {e}")
            return None
    
    def parse_gprmc(self, sentence):
        """Parse GPRMC (Recommended Minimum Specific GPS/Transit Data) sentence"""
        try:
            fields = sentence.split(',')
            if len(fields) < 12:
                logger.warning(f"GPRMC sentence too short: {len(fields)} fields")
                return None
            
            # Extract GPS data
            time_str = fields[1] if fields[1] else "000000.000"
            status = fields[2] if fields[2] else "V"  # A=valid, V=invalid
            latitude_str = fields[3] if fields[3] else "0"
            latitude_dir = fields[4] if fields[4] else "N"
            longitude_str = fields[5] if fields[5] else "0"
            longitude_dir = fields[6] if fields[6] else "E"
            speed_knots = float(fields[7]) if fields[7] else 0.0
            track_angle = float(fields[8]) if fields[8] else 0.0
            date_str = fields[9] if fields[9] else "000000"
            
            # Convert coordinates (same as GPGGA)
            if latitude_str and latitude_str != "0":
                lat_deg = float(latitude_str[:2])
                lat_min = float(latitude_str[2:])
                latitude = lat_deg + lat_min / 60.0
                if latitude_dir == 'S':
                    latitude = -latitude
            else:
                latitude = 0.0
            
            if longitude_str and longitude_str != "0":
                lon_deg = float(longitude_str[:3])
                lon_min = float(longitude_str[3:])
                longitude = lon_deg + lon_min / 60.0
                if longitude_dir == 'W':
                    longitude = -longitude
            else:
                longitude = 0.0
            
            # Determine fix quality based on status
            fix_quality = 1 if status == 'A' else 0
            
            current_time = datetime.now()
            
            logger.info(f"GPRMC parsed - Lat: {latitude}, Lon: {longitude}, Status: {status}, Speed: {speed_knots}")
            
            return {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': 0.0,  # Not available in GPRMC
                'battery_voltage': 11.5,  # Default value
                'swa': 0,
                'swc': 0,
                'failsafe': 0,
                'failsafe_triggered': False,
                'gps_fix': fix_quality,
                'gps_satellites': 0,  # Not available in GPRMC
                'battery_percentage': 100,
                'low_battery_warning': False,
                'timestamp': current_time.isoformat()
            }
            
        except Exception as e:
            logger.error(f"Failed to parse GPRMC: {e}")
            return None
    
    def parse_gpgsv(self, sentence):
        """Parse GPGSV (GPS Satellites in View) sentence"""
        try:
            fields = sentence.split(',')
            if len(fields) < 4:
                logger.warning(f"GPGSV sentence too short: {len(fields)} fields")
                return None
            
            # Extract satellite information
            total_sentences = int(fields[1]) if fields[1] else 1
            current_sentence = int(fields[2]) if fields[2] else 1
            total_satellites = int(fields[3]) if fields[3] else 0
            
            logger.info(f"GPGSV parsed - Total sats: {total_satellites}, Sentence {current_sentence}/{total_sentences}")
            
            # GPGSV doesn't contain position data, just satellite info
            # Return None to indicate no position update
            return None
            
        except Exception as e:
            logger.error(f"Failed to parse GPGSV: {e}")
            return None
    
    def parse_pid_gain_set(self, message_id, data):
        """Parse PID gain set message"""
        if len(data) < 12:
            return None
            
        try:
            p, i, d = struct.unpack('<fff', data[0:12])
            
            gain_names = {
                0x00: 'roll_inner',
                0x01: 'roll_outer',
                0x02: 'pitch_inner',
                0x03: 'pitch_outer',
                0x04: 'yaw_angle',
                0x05: 'yaw_rate'
            }
            
            gain_type = gain_names.get(message_id, f'unknown_{message_id}')
            
            return {
                'type': gain_type,
                'p': p,
                'i': i,
                'd': d,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Failed to parse PID gain set: {e}")
            return None
    
    def parse_pid_gain_ack(self, message_id, data):
        """Parse PID gain acknowledgment message"""
        if len(data) < 12:
            return None
            
        try:
            p, i, d = struct.unpack('<fff', data[0:12])
            
            # Validate PID values to prevent extreme values
            def is_valid_pid_value(value):
                if math.isnan(value) or math.isinf(value):
                    return False
                # PID values should typically be between -1000 and 1000
                if abs(value) > 1000:
                    return False
                return True
            
            # Check if values are reasonable
            if not all(is_valid_pid_value(val) for val in [p, i, d]):
                logger.warning(f"Invalid PID values detected: P={p}, I={i}, D={d}")
                logger.warning(f"Raw data: {data.hex()}")
                return None
            
            gain_names = {
                0x00: 'roll_inner',
                0x01: 'roll_outer',
                0x02: 'pitch_inner',
                0x03: 'pitch_outer',
                0x04: 'yaw_angle',
                0x05: 'yaw_rate'
            }
            
            gain_type = gain_names.get(message_id, f'unknown_{message_id}')
            
            logger.info(f"Valid PID values: {gain_type} - P={p:.3f}, I={i:.3f}, D={d:.3f}")
            
            return {
                'type': gain_type,
                'p': p,
                'i': i,
                'd': d,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Failed to parse PID gain ACK: {e}")
            logger.error(f"Raw data: {data.hex()}")
            return None
    
    def read_serial_data(self):
        """Read and parse serial data from flight controller with auto-reconnect"""
        buffer = bytearray()
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        
        while self.is_connected:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    logger.warning("Serial port not available, attempting to reconnect...")
                    if self.reconnect_serial():
                        reconnect_attempts = 0
                        buffer = bytearray()  # Clear buffer on reconnect
                        continue
                    else:
                        reconnect_attempts += 1
                        if reconnect_attempts >= max_reconnect_attempts:
                            logger.error("Max reconnection attempts reached, stopping...")
                            break
                        time.sleep(2)  # Wait before retry
                        continue
                
                if self.serial_port.in_waiting > 0:
                    byte_data = self.serial_port.read(1)
                    if byte_data:
                        buffer.extend(byte_data)
                        
                        # Check for NMEA sentences first (start with $)
                        if len(buffer) > 0 and buffer[0] == ord('$'):
                            # Look for complete NMEA sentence (ends with \r\n)
                            nmea_end = -1
                            for i in range(1, len(buffer)):
                                if buffer[i] == ord('\n') and i > 0 and buffer[i-1] == ord('\r'):
                                    nmea_end = i + 1
                                    break
                            
                            if nmea_end != -1:
                                # Found complete NMEA sentence
                                nmea_data = buffer[:nmea_end]
                                logger.info(f"Received NMEA data: {nmea_data.decode('ascii', errors='ignore').strip()}")
                                
                                # Process as GPS data (message ID 0x11)
                                self.process_fc_message(nmea_data, message_id=0x11)
                                buffer = buffer[nmea_end:]  # Remove processed NMEA sentence
                                continue
                        
                        # Process 20-byte messages according to protocol
                        if len(buffer) >= 20:
                            # Check for FC sync pattern at the start (0x46 0x43)
                            message = buffer[:20]
                            logger.info(f"Received 20 bytes: {message.hex()}")
                            
                            if message[0:2] == b'\x46\x43':  # FC sync bytes at start
                                logger.info("Found FC sync pattern at start")
                                self.process_fc_message(message)
                                buffer = buffer[20:]  # Remove processed message
                            else:
                                # Look for FC sync pattern anywhere in the buffer
                                fc_pos = -1
                                for i in range(len(buffer) - 1):
                                    if buffer[i:i+2] == b'\x46\x43':
                                        fc_pos = i
                                        break
                                
                                if fc_pos != -1 and fc_pos + 20 <= len(buffer):
                                    # Found FC sync, process 20-byte message starting from sync
                                    message = buffer[fc_pos:fc_pos+20]
                                    logger.info(f"Found FC sync at position {fc_pos}")
                                    self.process_fc_message(message)
                                    buffer = buffer[fc_pos+20:]  # Remove processed message
                                else:
                                    # No FC sync found, remove first byte and continue
                                    buffer = buffer[1:]
                        
                        # Clear buffer if it gets too large (prevent memory issues)
                        if len(buffer) > 100:
                            logger.warning("Buffer too large, clearing...")
                            buffer = bytearray()
                else:
                    # Small delay to prevent CPU overuse
                    time.sleep(0.001)
                                
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                # Don't break immediately, try to reconnect
                if "핸들이 잘못되었습니다" in str(e) or "handle is invalid" in str(e):
                    logger.warning("Serial handle invalid, attempting to reconnect...")
                    if self.reconnect_serial():
                        reconnect_attempts = 0
                        buffer = bytearray()
                        continue
                    else:
                        reconnect_attempts += 1
                        if reconnect_attempts >= max_reconnect_attempts:
                            logger.error("Max reconnection attempts reached, stopping...")
                            break
                        time.sleep(2)
                        continue
                else:
                    # For other errors, wait a bit and continue
                    time.sleep(0.1)
                
        self.is_connected = False
        self.latest_data['connection_status'] = 'Disconnected'
        logger.info("Serial read loop ended")
    
    def process_fc_message(self, message, message_id=None):
        """Process received message FROM flight controller (FC sync bytes at START) or NMEA data"""
        
        # Handle NMEA data (passed with explicit message_id)
        if message_id is not None:
            logger.info(f"Processing NMEA message as GPS data (ID: 0x{message_id:02X})")
            if message_id == 0x11:  # GPS message
                gps_data = self.parse_gps_message(message)
                if gps_data:
                    # Update both legacy and enhanced data structures
                    self.latest_data['gps'] = gps_data
                    self.latest_data['navigation_data'].update({
                        'gps_latitude': gps_data['latitude'],
                        'gps_longitude': gps_data['longitude'],
                        'gps_altitude': gps_data['altitude'],
                        'gps_fix': gps_data['gps_fix'],
                        'gps_satellites': gps_data['gps_satellites']
                    })
                    self.latest_data['power_system'].update({
                        'battery_voltage': gps_data['battery_voltage'],
                        'battery_percentage': gps_data['battery_percentage'],
                        'low_battery_warning': gps_data['low_battery_warning']
                    })
                    self.latest_data['remote_control'].update({
                        'swa': gps_data['swa'],
                        'swc': gps_data['swc'],
                        'failsafe': gps_data['failsafe'],
                        'failsafe_triggered': gps_data['failsafe_triggered']
                    })
                    
                    # Emit real-time updates
                    self.socketio.emit('gps_data', gps_data)
                    self.socketio.emit('navigation_data', self.latest_data['navigation_data'])
                    self.socketio.emit('power_system', self.latest_data['power_system'])
                    self.socketio.emit('remote_control', self.latest_data['remote_control'])
                    self.socketio.emit('system_status', self.latest_data['system_status'])
                    
                    logger.info(f"GPS data updated: {gps_data}")
                else:
                    logger.warning("Failed to parse NMEA GPS message")
            return
        
        # Handle binary FC messages
        if len(message) != 20:
            logger.warning(f"Invalid FC message length: {len(message)}")
            return
            
        # Verify FC sync bytes at the START (0x46, 0x43 = 'FC')
        if message[0:2] != b'\x46\x43':
            logger.warning(f"Invalid FC sync bytes at start: {message[0:2].hex()}")
            return
            
        # Verify checksum (checksum is at byte 19)
        # CHKSUM = 0xFF - (BYTE0~BYTE18) according to protocol
        sum_bytes = sum(message[:19])  # Sum bytes 0-18
        calculated_checksum = (0xFF - sum_bytes) & 0xFF  # Handle underflow
        if message[19] != calculated_checksum:
            logger.warning(f"FC checksum mismatch: expected {calculated_checksum}, got {message[19]}")
            return
            
        message_id = message[2]  # Message ID is at byte 2
        data = message[3:19]      # Data is from byte 3 to 18
        
        logger.info(f"Processing FC message ID: 0x{message_id:02X}, data: {data.hex()}")
        
        # Process based on message ID (data FROM flight controller)
        if message_id == 0x10:  # AHRS - 50Hz
            logger.info("Processing AHRS message from FC")
            ahrs_data = self.parse_ahrs_message(data)
            if ahrs_data:
                # Update both legacy and enhanced data structures
                self.latest_data['ahrs'] = ahrs_data
                self.latest_data['flight_data'].update({
                    'roll_angle': ahrs_data['roll_angle'],
                    'pitch_angle': ahrs_data['pitch_angle'],
                    'yaw_angle': ahrs_data['yaw_angle'],
                    'barometric_altitude': ahrs_data['altitude'],
                    'roll_setpoint': ahrs_data['roll_setpoint'],
                    'pitch_setpoint': ahrs_data['pitch_setpoint'],
                    'yaw_setpoint': ahrs_data['yaw_setpoint'],
                    'altitude_setpoint': ahrs_data['altitude_setpoint']
                })
                self.latest_data['system_status']['last_ahrs_update'] = ahrs_data['timestamp']
                
                # Log AHRS data if logging is enabled
                self.log_ahrs_data(ahrs_data)
                
                # Emit both legacy and enhanced events
                self.socketio.emit('ahrs_data', ahrs_data)
                self.socketio.emit('flight_data', self.latest_data['flight_data'])
                self.socketio.emit('system_status', self.latest_data['system_status'])
                
                logger.info(f"AHRS data updated: {ahrs_data}")
            else:
                logger.warning("Failed to parse AHRS message")
                
        elif message_id == 0x11:  # GPS - 10Hz
            logger.info("Processing GPS message from FC")
            gps_data = self.parse_gps_message(data)
            if gps_data:
                # Update both legacy and enhanced data structures
                self.latest_data['gps'] = gps_data
                self.latest_data['navigation_data'].update({
                    'gps_latitude': gps_data['latitude'],
                    'gps_longitude': gps_data['longitude'],
                    'gps_fix': gps_data['gps_fix'],
                    'gps_satellites': gps_data['gps_satellites']
                })
                self.latest_data['power_system'].update({
                    'battery_voltage': gps_data['battery_voltage'],
                    'battery_percentage': gps_data['battery_percentage'],
                    'low_battery_warning': gps_data['low_battery_warning']
                })
                self.latest_data['remote_control'].update({
                    'ibus_swa': gps_data['swa'],
                    'ibus_swc': gps_data['swc'],
                    'failsafe_status': gps_data['failsafe'],
                    'failsafe_triggered': gps_data['failsafe_triggered']
                })
                self.latest_data['system_status']['last_gps_update'] = gps_data['timestamp']
                
                # Update legacy data for backward compatibility
                self.latest_data['battery_voltage'] = gps_data['battery_voltage']
                self.latest_data['switches'] = {
                    'swa': gps_data['swa'],
                    'swc': gps_data['swc'],
                    'failsafe': gps_data['failsafe']
                }
                
                # Emit multiple events for different data types
                self.socketio.emit('gps_data', gps_data)
                self.socketio.emit('navigation_data', self.latest_data['navigation_data'])
                self.socketio.emit('power_system', self.latest_data['power_system'])
                self.socketio.emit('remote_control', self.latest_data['remote_control'])
                self.socketio.emit('system_status', self.latest_data['system_status'])
                
                logger.info(f"GPS data updated: {gps_data}")
            else:
                logger.warning("Failed to parse GPS message")
                
        elif 0x00 <= message_id <= 0x05:  # PID Gain ACK
            logger.info(f"Processing PID gain ACK from FC for type {message_id}")
            pid_data = self.parse_pid_gain_ack(message_id, data)
            if pid_data:
                self.latest_data['pid_gains'][pid_data['type']] = {
                    'p': pid_data['p'],
                    'i': pid_data['i'],
                    'd': pid_data['d']
                }
                self.socketio.emit('pid_gain_ack', pid_data)
                logger.info(f"PID ACK data updated: {pid_data}")
            else:
                logger.warning("Failed to parse PID gain ACK")
        else:
            logger.warning(f"Unknown FC message ID: 0x{message_id:02X}")
    
    def process_gs_message(self, message):
        """Process received message TO flight controller (GS sync bytes)"""
        if len(message) != 20:
            logger.warning(f"Invalid GS message length: {len(message)}")
            return
            
        # Verify GS sync bytes at the start (0x47, 0x53 = 'GS')
        if message[0:2] != b'GS':
            logger.warning(f"Invalid GS sync bytes at start: {message[0:2]}")
            return
            
        # Verify checksum
        # CHKSUM = 0xFF - (BYTE0~BYTE18) according to protocol
        sum_bytes = sum(message[:19])  # Sum bytes 0-18
        calculated_checksum = (0xFF - sum_bytes) & 0xFF  # Handle underflow
        if message[19] != calculated_checksum:
            logger.warning(f"GS checksum mismatch: expected {calculated_checksum}, got {message[19]}")
            return
            
        message_id = message[2]  # Message ID is at byte 2
        data = message[3:19]     # Data is from byte 3 to 18
        
        logger.info(f"Processing GS message ID: 0x{message_id:02X}")
        
        # Process based on message ID (data TO flight controller)
        if message_id == 0x10:  # PID Gain Request
            logger.info("Processing PID Gain Request to FC")
            # This is a request from GCS to FC
            logger.info("Sent PID gain request to FC")
                
        elif 0x00 <= message_id <= 0x05:  # PID Gain Set
            logger.info(f"Processing PID gain set to FC for type {message_id}")
            pid_data = self.parse_pid_gain_set(message_id, data)
            if pid_data:
                logger.info(f"Sent PID data to FC: {pid_data}")
            else:
                logger.warning("Failed to parse PID gain set")
        else:
            logger.warning(f"Unknown GS message ID: 0x{message_id:02X}")
    
    def process_message(self, message):
        """Process received message"""
        if len(message) != 20:
            logger.warning(f"Invalid message length: {len(message)}")
            return
            
        # Verify sync bytes at the start (GS sync bytes: 0x47, 0x53 = 'GS')
        if message[0:2] != b'GS':
            logger.warning(f"Invalid sync bytes at start: {message[0:2]}")
            return
            
        # Verify checksum
        calculated_checksum = self.calculate_checksum(message)
        if message[19] != calculated_checksum:
            logger.warning(f"Checksum mismatch: expected {calculated_checksum}, got {message[19]}")
            return
            
        message_id = message[2]  # Message ID is at byte 2
        data = message[3:19]     # Data is from byte 3 to 18
        
        logger.info(f"Processing message ID: 0x{message_id:02X}")
        
        # Process based on message ID
        if message_id == 0x10:  # PID Gain Request
            logger.info("Processing PID Gain Request")
            # This is a request from GCS to FC, not data from FC
            logger.info("Received PID gain request from GCS")
                
        elif 0x00 <= message_id <= 0x05:  # PID Gain Set
            logger.info(f"Processing PID gain set for type {message_id}")
            pid_data = self.parse_pid_gain_set(message_id, data)
            if pid_data:
                self.latest_data['pid_gains'][pid_data['type']] = {
                    'p': pid_data['p'],
                    'i': pid_data['i'],
                    'd': pid_data['d']
                }
                self.socketio.emit('pid_gain_set', pid_data)
                logger.info(f"PID data updated: {pid_data}")
            else:
                logger.warning("Failed to parse PID gain set")
        else:
            logger.warning(f"Unknown message ID: 0x{message_id:02X}")
    
    def process_fc_message_reversed(self, message):
        """Process FC message with sync bytes at the start"""
        if len(message) != 20:
            logger.warning(f"Invalid FC message length: {len(message)}")
            return
            
        # Verify FC sync bytes at the START (0x46, 0x43 = 'FC')
        if message[0:2] != b'FC':
            logger.warning(f"Invalid FC sync bytes at start: {message[0:2]}")
            return
            
        message_id = message[2]  # Message ID is at byte 2
        data = message[3:19]     # Data is from byte 3 to 18
        
        logger.info(f"Processing reversed FC message ID: 0x{message_id:02X}, data: {data.hex()}")
        self.process_message_by_id(message_id, data)
    
    def analyze_fc_data_pattern(self, message):
        """Analyze FC data pattern to understand the actual protocol structure"""
        if len(message) != 20:
            return None
            
        # Log the full message for analysis
        logger.info(f"=== FC DATA PATTERN ANALYSIS ===")
        logger.info(f"Full message (hex): {message.hex()}")
        logger.info(f"Full message (bytes): {list(message)}")
        
        # Try to identify patterns
        # Pattern 1: 00a01e65fe1400e2ff74461027764643109f004a
        # Pattern 2: 00000000005a04000000000000074643109f004b
        
        # Look for potential sync bytes or patterns
        for i in range(len(message) - 1):
            if message[i:i+2] == b'FC':
                logger.info(f"Found 'FC' at position {i}: {message[i:i+2].hex()}")
                
        # Look for potential message IDs
        for i in range(len(message)):
            if message[i] in [0x10, 0x11, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05]:
                logger.info(f"Potential message ID 0x{message[i]:02X} at position {i}")
                
        # Look for potential checksums or end markers
        for i in range(len(message)):
            if message[i] in [0x4a, 0x4b, 0x46, 0x43]:  # Common end bytes
                logger.info(f"Potential end marker 0x{message[i]:02X} at position {i}")
                
        logger.info(f"=== END ANALYSIS ===")
        return True

    def process_raw_fc_data(self, message):
        """Process raw FC data according to protocol specification"""
        if len(message) != 20:
            logger.warning(f"Invalid raw message length: {len(message)}")
            return
            
        # Look for FC sync bytes at any position (FC might send data in different format)
        fc_pos = -1
        for i in range(len(message) - 1):
            if message[i:i+2] == b'FC':
                fc_pos = i
                break
        
        if fc_pos != -1:
            # Found FC sync bytes
            message_id = message[fc_pos + 2] if fc_pos + 2 < len(message) else 0
            data_start = fc_pos + 3
            data_end = min(data_start + 16, 20)
            data = message[data_start:data_end]
            
            if fc_pos == 0:
                logger.info(f"Found FC sync at position 0, message ID: 0x{message_id:02X}")
            else:
                logger.info(f"Found FC sync at position {fc_pos}, message ID: 0x{message_id:02X}")
                
            if message_id in [0x10, 0x11, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05]:
                self.process_message_by_id(message_id, data)
            else:
                logger.warning(f"Unknown FC message ID: 0x{message_id:02X}")
        else:
            logger.warning("No FC sync bytes found in message - protocol violation")
    
    def process_message_by_id(self, message_id, data):
        """Process message based on ID"""
        # Process based on message ID (data FROM flight controller)
        if message_id == 0x10:  # AHRS - 50Hz
            logger.info("Processing AHRS message from FC")
            ahrs_data = self.parse_ahrs_message(data)
            if ahrs_data:
                # Update both legacy and enhanced data structures
                self.latest_data['ahrs'] = ahrs_data
                self.latest_data['flight_data'].update({
                    'roll_angle': ahrs_data['roll_angle'],
                    'pitch_angle': ahrs_data['pitch_angle'],
                    'yaw_angle': ahrs_data['yaw_angle'],
                    'barometric_altitude': ahrs_data['altitude'],
                    'roll_setpoint': ahrs_data['roll_setpoint'],
                    'pitch_setpoint': ahrs_data['pitch_setpoint'],
                    'yaw_setpoint': ahrs_data['yaw_setpoint'],
                    'altitude_setpoint': ahrs_data['altitude_setpoint']
                })
                self.latest_data['system_status']['last_ahrs_update'] = ahrs_data['timestamp']
                
                # Log AHRS data if logging is enabled
                self.log_ahrs_data(ahrs_data)
                
                # Emit both legacy and enhanced events
                self.socketio.emit('ahrs_data', ahrs_data)
                self.socketio.emit('flight_data', self.latest_data['flight_data'])
                self.socketio.emit('system_status', self.latest_data['system_status'])
                
                logger.info(f"AHRS data updated: {ahrs_data}")
            else:
                logger.warning("Failed to parse AHRS message")
                
        elif message_id == 0x11:  # GPS - 10Hz
            logger.info("Processing GPS message from FC")
            gps_data = self.parse_gps_message(data)
            if gps_data:
                # Update both legacy and enhanced data structures
                self.latest_data['gps'] = gps_data
                self.latest_data['navigation_data'].update({
                    'gps_latitude': gps_data['latitude'],
                    'gps_longitude': gps_data['longitude'],
                    'gps_fix': gps_data['gps_fix'],
                    'gps_satellites': gps_data['gps_satellites']
                })
                self.latest_data['power_system'].update({
                    'battery_voltage': gps_data['battery_voltage'],
                    'battery_percentage': gps_data['battery_percentage'],
                    'low_battery_warning': gps_data['low_battery_warning']
                })
                self.latest_data['remote_control'].update({
                    'ibus_swa': gps_data['swa'],
                    'ibus_swc': gps_data['swc'],
                    'failsafe_status': gps_data['failsafe'],
                    'failsafe_triggered': gps_data['failsafe_triggered']
                })
                self.latest_data['system_status']['last_gps_update'] = gps_data['timestamp']
                
                # Update legacy data for backward compatibility
                self.latest_data['battery_voltage'] = gps_data['battery_voltage']
                self.latest_data['switches'] = {
                    'swa': gps_data['swa'],
                    'swc': gps_data['swc'],
                    'failsafe': gps_data['failsafe']
                }
                
                # Emit multiple events for different data types
                self.socketio.emit('gps_data', gps_data)
                self.socketio.emit('navigation_data', self.latest_data['navigation_data'])
                self.socketio.emit('power_system', self.latest_data['power_system'])
                self.socketio.emit('remote_control', self.latest_data['remote_control'])
                self.socketio.emit('system_status', self.latest_data['system_status'])
                
                logger.info(f"GPS data updated: {gps_data}")
            else:
                logger.warning("Failed to parse GPS message")
                
        elif 0x00 <= message_id <= 0x05:  # PID Gain ACK
            logger.info(f"Processing PID gain ACK from FC for type {message_id}")
            pid_data = self.parse_pid_gain_ack(message_id, data)
            if pid_data:
                self.latest_data['pid_gains'][pid_data['type']] = {
                    'p': pid_data['p'],
                    'i': pid_data['i'],
                    'd': pid_data['d']
                }
                self.socketio.emit('pid_gain_ack', pid_data)
                logger.info(f"PID gain ACK updated: {pid_data}")
            else:
                logger.warning("Failed to parse PID gain ACK")
        else:
            logger.info(f"Unknown FC message ID: 0x{message_id:02X}")
    
    def generate_test_data(self):
        """Test data generation disabled - only real FC data"""
        logger.info("Test data generation disabled - only real FC data allowed")
        return None
    
    def start_auto_test(self):
        """Auto-test data generation disabled - only real FC data"""
        logger.info("Auto-test data generation disabled - only real FC data allowed")
        pass
    
    def start_data_logging(self):
        """Start logging AHRS data to CSV file in Sensor Log folder"""
        try:
            # Create Sensor Log folder if it doesn't exist
            import os
            sensor_log_dir = "Sensor Log"
            if not os.path.exists(sensor_log_dir):
                os.makedirs(sensor_log_dir)
                logger.info(f"Created Sensor Log directory: {sensor_log_dir}")
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(sensor_log_dir, f"ahrs_log_{timestamp}.csv")
            self.log_file = open(filename, 'w', newline='', encoding='utf-8')
            
            # Write CSV header
            header = "Timestamp,Roll_Angle,Pitch_Angle,Yaw_Angle,Altitude,Roll_Setpoint,Pitch_Setpoint,Yaw_Setpoint,Altitude_Setpoint\n"
            self.log_file.write(header)
            
            self.data_logging_enabled = True
            self.log_start_time = datetime.now()
            logger.info(f"📊 AHRS Data logging started: {filename}")
            return {'status': 'success', 'filename': filename}
        except Exception as e:
            logger.error(f"Error starting data logging: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def stop_data_logging(self):
        """Stop logging AHRS data"""
        try:
            if self.log_file:
                # Add summary footer for CSV
                footer = f"# Logging stopped at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                if self.log_start_time:
                    duration = (datetime.now() - self.log_start_time).total_seconds()
                    footer += f"# Total duration: {duration:.1f} seconds\n"
                    footer += f"# Total data points: {self.data_points_logged if hasattr(self, 'data_points_logged') else 'Unknown'}\n"
                self.log_file.write(footer)
                self.log_file.close()
                self.log_file = None
            
            self.data_logging_enabled = False
            duration = None
            if self.log_start_time:
                duration = (datetime.now() - self.log_start_time).total_seconds()
                self.log_start_time = None
            
            logger.info("📊 AHRS Data logging stopped")
            return {'status': 'success', 'duration': duration}
        except Exception as e:
            logger.error(f"Error stopping data logging: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def log_ahrs_data(self, ahrs_data):
        """Log AHRS data to CSV format file"""
        if not self.data_logging_enabled or not self.log_file:
            return
        
        try:
            # Initialize data points counter if not exists
            if not hasattr(self, 'data_points_logged'):
                self.data_points_logged = 0
            
            # Get timestamp
            timestamp = ahrs_data.get('timestamp', datetime.now().isoformat())
            
            # Get AHRS values
            roll = ahrs_data.get('roll_angle', 0.0)
            pitch = ahrs_data.get('pitch_angle', 0.0)
            yaw = ahrs_data.get('yaw_angle', 0.0)
            altitude = ahrs_data.get('altitude', 0.0)
            roll_sp = ahrs_data.get('roll_setpoint', 0.0)
            pitch_sp = ahrs_data.get('pitch_setpoint', 0.0)
            yaw_sp = ahrs_data.get('yaw_setpoint', 0.0)
            alt_sp = ahrs_data.get('altitude_setpoint', 0.0)
            
            # Write CSV line
            line = f"{timestamp},{roll:.3f},{pitch:.3f},{yaw:.3f},{altitude:.1f},{roll_sp:.3f},{pitch_sp:.3f},{yaw_sp:.3f},{alt_sp:.1f}\n"
            self.log_file.write(line)
            self.log_file.flush()  # Ensure data is written immediately
            
            # Increment data points counter
            self.data_points_logged += 1
            
            # Log every 100 data points for monitoring
            if self.data_points_logged % 100 == 0:
                logger.info(f"📊 Logged {self.data_points_logged} AHRS data points")
                
        except Exception as e:
            logger.error(f"Error logging AHRS data: {e}")
    
    def run(self, host='127.0.0.1', port=5001, debug=False):
        """Run the ground station web server"""
        logger.info(f"Starting KMU Ground Station on {host}:{port}")
        
        # Use regular Flask server instead of SocketIO for now
        self.app.run(host=host, port=port, debug=debug)

if __name__ == '__main__':
    gs = KMUGroundStation()
    gs.run(debug=False)  # Use default port or any available port
