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
            # Enhanced drone telemetry
            'drone_telemetry': {
                'flight_mode': 'MANUAL',
                'armed_status': False,
                'arming_state': 'STANDBY',  # STANDBY, ARMING, ARMED, DISARMING
                'battery_cells': 4,
                'voltage_per_cell': 0.0,
                'battery_current': 0.0,
                'consumption_mah': 0.0,
                'estimated_flight_time': 0.0,
                'esc_status': {
                    'esc_1': {'temperature': 0.0, 'voltage': 0.0, 'current': 0.0, 'rpm': 0},
                    'esc_2': {'temperature': 0.0, 'voltage': 0.0, 'current': 0.0, 'rpm': 0},
                    'esc_3': {'temperature': 0.0, 'voltage': 0.0, 'current': 0.0, 'rpm': 0},
                    'esc_4': {'temperature': 0.0, 'voltage': 0.0, 'current': 0.0, 'rpm': 0}
                },
                'gps_status': {
                    'fix_type': 0,  # 0=No fix, 1=Dead reckoning, 2=2D fix, 3=3D fix, 4=GPS+Dead reckoning, 5+TIMEFIX
                    'hdop': 0.0,    # Horizontal dilution of precision
                    'vdop': 0.0,    # Vertical dilution of precision  
                    'satellites_visible': 0,
                    'home_position_set': False,
                    'home_lat': 0.0,
                    'home_lon': 0.0,
                    'home_alt': 0.0,
                    'distance_to_home': 0.0,
                    'bearing_to_home': 0.0
                },
                'wind_data': {
                    'wind_speed': 0.0,
                    'wind_direction': 0.0,
                    'ground_speed': 0.0,
                    'airspeed': 0.0
                },
                'geofence_status': {
                    'enabled': False,
                    'violation': False,
                    'action': 'NONE'  # NONE, GUIDED, RETURN, TERMINATE
                }
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
        
        # Initialize comprehensive drone telemetry logging
        self.data_logging_enabled = False
        self.log_file = None
        self.log_start_time = None
        
        # Enhanced telemetry logging components
        self.telemetry_loggers = {
            'ahrs': None,      # AHRS data logging
            'gps': None,       # GPS data logging  
            'power': None,     # Battery/system power logging
            'motors': None,    # Motor/ESC data logging
            'flight_modes': None,  # Flight mode changes logging
            'debug': None      # Debug and system messages logging
        }
        
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
                
                # Start a simple test data generator if no real data comes in
                self.start_test_data_generator()
                self.test_mode = True
                
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

        @self.app.route('/api/drone_telemetry')
        def get_drone_telemetry():
            """Get comprehensive drone telemetry data"""
            return jsonify({
                'status': 'success',
                'data': self.latest_data['drone_telemetry']
            })
        
        @self.app.route('/api/drone_power')
        def get_drone_power():
            """Get detailed battery and power telemetry"""
            return jsonify({
                'status': 'success',
                'data': {
                    'battery_cells': self.latest_data['drone_telemetry'].get('battery_cells', 4),
                    'voltage_per_cell': self.latest_data['drone_telemetry'].get('voltage_per_cell', 0.0),
                    'battery_current': self.latest_data['drone_telemetry'].get('battery_current', 0.0),
                    'consumption_mah': self.latest_data['drone_telemetry'].get('consumption_mah', 0),
                    'estimated_flight_time': self.latest_data['drone_telemetry'].get('estimated_flight_time', 0.0),
                    'power_system': self.latest_data['power_system']
                }
            })
        
        @self.app.route('/api/drone_motors')
        def get_drone_motors():
            """Get ESC and motor telemetry"""
            return jsonify({
                'status': 'success',
                'data': self.latest_data['drone_telemetry'].get('esc_status', {})
            })
        
        @self.app.route('/api/drone_gps_enhanced')
        def get_drone_gps_enhanced():
            """Get enhanced GPS telemetry"""
            return jsonify({
                'status': 'success',
                'data': self.latest_data['drone_telemetry'].get('gps_status', {})
            })
        
        @self.app.route('/api/drone_flight_mode')
        def get_drone_flight_mode():
            """Get flight mode and arming status"""
            return jsonify({
                'status': 'success',
                'data': {
                    'flight_mode': self.latest_data['drone_telemetry'].get('flight_mode', 'MANUAL'),
                    'armed_status': self.latest_data['drone_telemetry'].get('armed_status', False),
                    'arming_state': self.latest_data['drone_telemetry'].get('arming_state', 'STANDBY')
                }
            })
        
        @self.app.route('/api/start_drone_logging', methods=['POST'])
        def start_drone_logging():
            """Start comprehensive drone telemetry logging"""
            result = self.start_comprehensive_logging()
            return jsonify(result)
        
        @self.app.route('/api/stop_drone_logging', methods=['POST'])
        def stop_drone_logging():
            """Stop comprehensive drone telemetry logging"""
            result = self.stop_comprehensive_logging()
            return jsonify(result)

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
                    'last_ahrs_update': self.latest_data.get('system_status', {}).get('last_ahrs_update'),
                    'last_gps_update': self.latest_data.get('system_status', {}).get('last_gps_update'),
                    'connection_quality': self.latest_data.get('system_status', {}).get('connection_quality', 0.0),
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
        
        @self.app.route('/api/debug_diagnostic')
        def debug_diagnostic():
            """Get comprehensive diagnostic information"""
            try:
                import serial.tools.list_ports
                
                # Get available ports
                available_ports = []
                for port in serial.tools.list_ports.comports():
                    available_ports.append({
                        'device': port.device,
                        'description': port.description,
                        'hwid': port.hwid
                    })
                
                # Get recent log messages
                recent_logs = []
                if hasattr(self, '_recent_logs'):
                    recent_logs = self._recent_logs[-10:]  # Last 10 log messages
                
                return jsonify({
                    'ground_station_status': {
                        'is_connected': self.is_connected,
                        'data_logging_enabled': self.data_logging_enabled,
                        'test_mode': self.test_mode
                    },
                    'serial_status': {
                        'port': self.serial_port.device if self.serial_port else None,
                        'port_open': self.serial_port.is_open if self.serial_port else False,
                        'baud_rate': self.serial_port.baudrate if self.serial_port else None,
                        'in_waiting': self.serial_port.in_waiting if self.serial_port and self.serial_port.is_open else 0
                    },
                    'available_ports': available_ports,
                    'data_status': {
                        'ahrs_data_received': bool(self.latest_data.get('ahrs')),
                        'gps_data_received': bool(self.latest_data.get('gps')),
                        'battery_data_received': self.latest_data.get('power_system', {}).get('battery_voltage', 0) > 0,
                        'drone_telemetry_populated': bool(self.latest_data.get('drone_telemetry', {}).get('flight_mode'))
                    },
                    'recent_logs': recent_logs,
                    'status': 'success'
                })
            except Exception as e:
                logger.error(f"Error getting diagnostic info: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        @self.app.route('/api/test_data')
        def trigger_test_data():
            """Manually trigger test data generation for debugging"""
            try:
                if not self.is_connected:
                    return jsonify({
                        'status': 'error',
                        'message': 'Must be connected to generate test data'
                    }), 400
                
                self.start_test_data_generator()
                return jsonify({
                    'status': 'success',
                    'message': 'Test data generator started'
                })
            except Exception as e:
                logger.error(f"Error triggering test data: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        @self.app.route('/api/stop_test_data')
        def stop_test_data():
            """Stop test data generation"""
            try:
                self.test_mode = False
                if hasattr(self, '_test_data_thread') and self._test_data_thread.is_alive():
                    # Thread will end when test_mode becomes False
                    pass
                return jsonify({
                    'status': 'success',
                    'message': 'Test data generation stopped'
                })
            except Exception as e:
                logger.error(f"Error stopping test data: {e}")
                return jsonify({
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
    
    def parse_battery_status(self, data):
        """Parse Battery Status message (ID 0x12)"""
        if len(data) < 16:
            logger.warning(f"Battery data too short: {len(data)} bytes")
            return None
            
        try:
            # Parse battery telemetry data
            voltage = struct.unpack('<H', data[0:2])[0] / 100.0  # Volts * 100
            current = struct.unpack('<h', data[2:4])[0] / 100.0  # Amps * 100 (signed)
            consumption = struct.unpack('<I', data[4:8])[0]  # mAh consumed
            cells = data[8] if len(data) > 8 else 4
            remaining_capacity = struct.unpack('<H', data[9:11])[0]  # mAh remaining
            
            # Calculate flight time estimation
            flight_time = 0.0
            if current > 0.1:  # Avoid division by zero
                flight_time = remaining_capacity / (current * 1000) * 60  # minutes
            
            current_time = datetime.now()
            
            return {
                'voltage': voltage,
                'current': current,
                'consumption_mah': consumption,
                'cells': cells,
                'remaining_capacity': remaining_capacity,
                'estimated_flight_time': flight_time,
                'voltage_per_cell': voltage / cells if cells > 0 else 0,
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse battery status: {e}")
            return None
    
    def parse_esc_status(self, data):
        """Parse ESC/Motor Status message (ID 0x13)"""
        if len(data) < 16:
            logger.warning(f"ESC data too short: {len(data)} bytes")
            return None
            
        try:
            # Parse ESC data for 4 motors
            esc_data = {}
            
            for i in range(4):
                start_idx = i * 3
                if start_idx + 3 <= len(data):
                    # Each ESC: temperature, voltage, current
                    temp = struct.unpack('<B', data[start_idx:start_idx+1])[0]
                    voltage = struct.unpack('<B', data[start_idx+1:start_idx+2])[0] / 10.0  # Convert to volts
                    current = struct.unpack('<B', data[start_idx+2:start_idx+3])[0] / 10.0  # Convert to amps
                    
                    esc_data[f'esc_{i+1}'] = {
                        'temperature': temp,
                        'voltage': voltage,
                        'current': current,
                        'rpm': 0  # RPM would require additional data
                    }
            
            current_time = datetime.now()
            
            return {
                'esc_status': esc_data,
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse ESC status: {e}")
            return None
    
    def parse_flight_mode_status(self, data):
        """Parse Flight Mode Status message (ID 0x14)"""
        if len(data) < 8:
            logger.warning(f"Flight mode data too short: {len(data)} bytes")
            return None
            
        try:
            flight_modes = {0: 'MANUAL', 1: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'RTL', 5: 'LAND'}
            
            mode_id = data[0]
            arming_state = data[1]
            armed_status = (arming_state & 0x01) != 0  # Bit 0 = arming status
            
            arming_states = {
                0: 'STANDBY',
                1: 'ARMING', 
                2: 'ARMED',
                3: 'DISARMING'
            }
            
            current_time = datetime.now()
            
            return {
                'flight_mode': flight_modes.get(mode_id, 'UNKNOWN'),
                'armed_status': armed_status,
                'arming_state': arming_states.get((arming_state >> 1) & 0x03, 'UNKNOWN'),
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse flight mode status: {e}")
            return None
    
    def parse_gps_enhanced_status(self, data):
        """Parse Enhanced GPS Status message (ID 0x15)"""
        if len(data) < 16:
            logger.warning(f"Enhanced GPS data too short: {len(data)} bytes")
            return None
            
        try:
            # Parse enhanced GPS data
            fix_type = data[0]
            satellites_visible = data[1]
            hdop = struct.unpack('<H', data[2:4])[0] / 100.0
            vdop = struct.unpack('<H', data[4:6])[0] / 100.0
            
            # Home position (if available)
            home_lat = struct.unpack('<l', data[6:10])[0] / 10000000.0
            home_lon = struct.unpack('<l', data[10:14])[0] / 10000000.0
            home_alt = struct.unpack('<h', data[14:16])[0] / 10.0
            
            # Calculate home distance and bearing if current position available
            distance_to_home = 0.0
            bearing_to_home = 0.0
            
            if hasattr(self, 'latest_data') and 'navigation_data' in self.latest_data:
                current_lat = self.latest_data['navigation_data'].get('gps_latitude', 0)
                current_lon = self.latest_data['navigation_data'].get('gps_longitude', 0)
                
                if current_lat != 0 and current_lon != 0 and home_lat != 0 and home_lon != 0:
                    # Simple lat/lon distance calculation
                    lat_diff = abs(current_lat - home_lat)
                    lon_diff = abs(current_lon - home_lon)
                    distance_to_home = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough meters
            
            current_time = datetime.now()
            
            return {
                'fix_type': fix_type,
                'satellites_visible': satellites_visible,
                'hdop': hdop,
                'vdop': vdop,
                'home_position_set': home_lat != 0 and home_lon != 0,
                'home_lat': home_lat,
                'home_lon': home_lon,
                'home_alt': home_alt,
                'distance_to_home': distance_to_home,
                'bearing_to_home': bearing_to_home,
                'timestamp': current_time.isoformat()
            }
                
        except Exception as e:
            logger.error(f"Failed to parse enhanced GPS status: {e}")
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
                
        elif message_id == 0x12:  # Drone Battery Status - 5Hz
            logger.info("Processing Battery Status from drone")
            battery_data = self.parse_battery_status(data)
            if battery_data:
                self.update_battery_telemetry(battery_data)
                self.log_telemetry_data('power', battery_data)
                logger.info(f"Battery status updated: {battery_data}")
            else:
                logger.warning("Failed to parse battery status")
                
        elif message_id == 0x13:  # ESC/Motor Status - 10Hz  
            logger.info("Processing ESC/Motor status from drone")
            esc_data = self.parse_esc_status(data)
            if esc_data:
                self.update_esc_telemetry(esc_data)
                self.log_telemetry_data('motors', esc_data)
                logger.info(f"ESC status updated: {esc_data}")
            else:
                logger.warning("Failed to parse ESC status")
                
        elif message_id == 0x14:  # Flight Mode Status - Event based
            logger.info("Processing Flight Mode change from drone")
            flight_mode_data = self.parse_flight_mode_status(data)
            if flight_mode_data:
                self.update_flight_mode_telemetry(flight_mode_data)
                self.log_telemetry_data('flight_modes', flight_mode_data)
                # Emit flight mode change event
                self.socketio.emit('flight_mode_change', flight_mode_data)
                logger.info(f"Flight mode change: {flight_mode_data}")
            else:
                logger.warning("Failed to parse flight mode status")
                
        elif message_id == 0x15:  # GPS Enhanced Status - 1Hz
            logger.info("Processing Enhanced GPS status from drone")
            gps_enhanced_data = self.parse_gps_enhanced_status(data)
            if gps_enhanced_data:
                self.update_gps_enhanced_telemetry(gps_enhanced_data)
                self.log_telemetry_data('gps', gps_enhanced_data)
                logger.info(f"GPS enhanced status updated: {gps_enhanced_data}")
            else:
                logger.warning("Failed to parse enhanced GPS status")
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
        """Generate realistic test data for debugging when no FC is connected"""
        import random
        import math
        
        # Generate realistic drone telemetry data
        current_time = datetime.now()
        t = current_time.timestamp()

        # Simulate more realistic flight data with smoother transitions
        base_freq_roll = 0.8  # Slower roll oscillations
        base_freq_pitch = 0.6  # Even slower pitch
        
        # Add multiple frequency components for more realistic movement
        roll_angle = (math.sin(t * base_freq_roll) * 12 + 
                     math.sin(t * base_freq_roll * 3) * 3 +
                     random.uniform(-1.5, 1.5))
        
        pitch_angle = (math.cos(t * base_freq_pitch * 0.7) * 8 +
                      math.sin(t * base_freq_pitch * 2) * 2 +
                      random.uniform(-1.0, 1.0))
        
        # Yaw with some drift and realistic movement
        yaw_drift = (t * 0.8) % 360  # Slow yaw drift
        yaw_noise = math.sin(t * 0.3) * 2  # Small periodic yaw adjustment
        yaw_angle = yaw_drift + yaw_noise
        
        # Altitude with gradual climbing patterns
        altitude_base = 45.0 + math.sin(t * 0.05) * 15  # Slow altitude changes
        altitude_noise = random.uniform(-2, 2)
        altitude = altitude_base + altitude_noise
        
        # More realistic battery simulation
        base_voltage = 16.2 - (t / 7200) * 0.3  # Gradual discharge
        voltage_noise = math.sin(t * 0.2) * 0.05 + random.uniform(-0.05, 0.05)
        battery_voltage = max(11.5, base_voltage + voltage_noise)
        
        # Current varies with flight mode simulation
        base_current = 12.0 + math.sin(t * 0.4) * 2.5
        battery_current = max(6.0, base_current + random.uniform(-0.5, 0.5))
        
        # GPS with small realistic movement
        lat_center = 37.5665
        lon_center = 126.9780
        # Simulate small flight pattern
        lat_offset = math.sin(t * 0.1) * 0.0015 + random.uniform(-0.0002, 0.0002)
        lon_offset = math.cos(t * 0.1) * 0.0015 + random.uniform(-0.0002, 0.0002)
        
        return {
            'roll_angle': roll_angle,
            'pitch_angle': pitch_angle,
            'yaw_angle': yaw_angle,
            'altitude': altitude,
            'roll_setpoint': 0.0,
            'pitch_setpoint': 0.0,
            'yaw_setpoint': yaw_angle,
            'altitude_setpoint': altitude,
            'battery_voltage': battery_voltage,
            'battery_current': battery_current,
            'gps_latitude': lat_center + lat_offset,
            'gps_longitude': lon_center + lon_offset,
            'gps_fix': 1,
            'gps_satellites': random.randint(10, 14),
            'timestamp': current_time.isoformat()
        }
    
    def start_test_data_generator(self):
        """Start test data generation thread"""
        if not hasattr(self, '_test_data_thread') or not self._test_data_thread.is_alive():
            self._test_data_thread = threading.Thread(target=self._test_data_loop, daemon=True)
            self._test_data_thread.start()
            logger.info("🧪 Test data generator started")
    
    def _test_data_loop(self):
        """Test data generation loop"""
        logger.info("🧪 Starting test data generation loop")
        no_data_timer = 0
        
        while self.is_connected:
            try:
                # Check if real data is being received
                last_update = self.latest_data.get('system_status', {}).get('last_ahrs_update')
                
                if last_update:
                    # Real data detected, convert to datetime and check age
                    try:
                        if isinstance(last_update, str):
                            last_update_time = datetime.fromisoformat(last_update.replace('Z', '+00:00'))
                        else:
                            last_update_time = last_update
                            
                        time_since_update = (datetime.now().replace(tzinfo=last_update_time.tzinfo) - last_update_time).total_seconds()
                        
                        if time_since_update < 2.0:  # Data received within last 2 seconds
                            no_data_timer = 0
                            time.sleep(0.1)
                            continue
                    except:
                        pass
                
                # No real data, increment timer
                no_data_timer += 0.1
                
                # Generate test data at 10Hz if no real data for > 100ms
                if no_data_timer >= 0.1:
                    test_data = self.generate_test_data()
                    no_data_timer = 0  # Reset timer after generating data
                    if test_data:
                        # Update latest data
                        self.latest_data['ahrs'] = {
                            'roll_angle': test_data['roll_angle'],
                            'pitch_angle': test_data['pitch_angle'],
                            'yaw_angle': test_data['yaw_angle'],
                            'altitude': test_data['altitude'],
                            'roll_setpoint': test_data['roll_setpoint'],
                            'pitch_setpoint': test_data['pitch_setpoint'],
                            'yaw_setpoint': test_data['yaw_setpoint'],
                            'altitude_setpoint': test_data['altitude_setpoint'],
                            'timestamp': test_data['timestamp']
                        }
                        
                        self.latest_data['flight_data'].update({
                            'roll_angle': test_data['roll_angle'],
                            'pitch_angle': test_data['pitch_angle'],
                            'yaw_angle': test_data['yaw_angle'],
                            'barometric_altitude': test_data['altitude'],
                            'roll_setpoint': test_data['roll_setpoint'],
                            'pitch_setpoint': test_data['pitch_setpoint'],
                            'yaw_setpoint': test_data['yaw_setpoint'],
                            'altitude_setpoint': test_data['altitude_setpoint']
                        })
                        
                        # Update drone telemetry
                        self.latest_data['drone_telemetry'].update({
                            'flight_mode': 'STABILIZE',
                            'armed_status': True if test_data['battery_voltage'] > 12.0 else False,
                            'arming_state': 'ARMED' if test_data['battery_voltage'] > 12.0 else 'STANDBY',
                            'battery_cells': 4,
                            'voltage_per_cell': test_data['battery_voltage'] / 4,
                            'battery_current': test_data['battery_current'],
                            'consumption_mah': int(test_data['battery_current'] * 1000),  # Rough estimate
                            'estimated_flight_time': ((test_data['battery_voltage'] - 11.0) / (16.8 - 11.0)) * 15  # 15 min flight time estimate
                        })
                        
                        # Update power system
                        self.latest_data['power_system'].update({
                            'battery_voltage': test_data['battery_voltage'],
                            'battery_percentage': min(100, max(0, (test_data['battery_voltage'] - 11.0) / (16.8 - 11.0) * 100)),
                            'low_battery_warning': test_data['battery_voltage'] < 12.5,
                            'total_voltage': test_data['battery_voltage'],
                            'cell_voltage': [test_data['battery_voltage'] / 4] * 4  # Simulate 4 cells
                        })
                        
                        # Update navigation data
                        self.latest_data['navigation_data'].update({
                            'gps_latitude': test_data['gps_latitude'],
                            'gps_longitude': test_data['gps_longitude'],
                            'gps_fix': test_data['gps_fix'],
                            'gps_satellites': test_data['gps_satellites']
                        })
                        
                        # Update system status
                        self.latest_data['system_status'].update({
                            'data_rate_ahrs': 20.0,  # Simulate 20Hz
                            'data_rate_gps': 1.0,    # Simulate 1Hz
                            'last_ahrs_update': test_data['timestamp'],
                            'connection_quality': 85.0
                        })
                        
                        # Emit via SocketIO - Reduced logging frequency
                        if hasattr(self, 'socketio'):
                            self.socketio.emit('ahrs_data', self.latest_data['ahrs'])
                            self.socketio.emit('flight_data', self.latest_data['flight_data'])
                            self.socketio.emit('drone_telemetry', self.latest_data['drone_telemetry'])
                            self.socketio.emit('power_system', self.latest_data['power_system'])
                            self.socketio.emit('navigation_data', self.latest_data['navigation_data'])
                            self.socketio.emit('system_status', self.latest_data['system_status'])
                        
                        # Log test data every 5 seconds instead of every update
                        if int(time.time()) % 5 == 0:
                            logger.info(f"🧪 Generated test data: Roll={test_data['roll_angle']:.1f}°, Pitch={test_data['pitch_angle']:.1f}°, Alt={test_data['altitude']:.1f}m, Vbat={test_data['battery_voltage']:.1f}V")
                
                time.sleep(0.1)  # 10Hz update rate - smoother data flow
                
            except Exception as e:
                logger.error(f"Error in test data loop: {e}")
                time.sleep(1)
        
        logger.info("🧪 Test data generation loop ended")
    
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
    
    def update_battery_telemetry(self, battery_data):
        """Update battery telemetry data"""
        self.latest_data['drone_telemetry'].update({
            'battery_cells': battery_data.get('cells', 4),
            'voltage_per_cell': battery_data.get('voltage_per_cell', 0.0),
            'battery_current': battery_data.get('current', 0.0),
            'consumption_mah': battery_data.get('consumption_mah', 0),
            'estimated_flight_time': battery_data.get('estimated_flight_time', 0.0)
        })
        
        # Also update legacy power system data
        self.latest_data['power_system'].update({
            'battery_voltage': battery_data.get('voltage', 0.0),
            'battery_percentage': min(100, max(0, (battery_data.get('voltage', 0) - 3.0) * 100 / (4.2 - 3.0))),
            'low_battery_warning': battery_data.get('voltage_per_cell', 4.0) < 3.6  # Warn below 3.6V per cell
        })
    
    def update_esc_telemetry(self, esc_data):
        """Update ESC/motor telemetry data"""
        if 'esc_status' in esc_data:
            self.latest_data['drone_telemetry']['esc_status'] = esc_data['esc_status']
    
    def update_flight_mode_telemetry(self, flight_mode_data):
        """Update flight mode telemetry data"""
        self.latest_data['drone_telemetry'].update({
            'flight_mode': flight_mode_data.get('flight_mode', 'MANUAL'),
            'armed_status': flight_mode_data.get('armed_status', False),
            'arming_state': flight_mode_data.get('arming_state', 'STANDBY')
        })
        
        # Also update legacy flight mode
        self.latest_data['flight_mode'] = flight_mode_data.get('flight_mode', 'MANUAL')
    
    def update_gps_enhanced_telemetry(self, gps_enhanced_data):
        """Update enhanced GPS telemetry data"""
        self.latest_data['drone_telemetry']['gps_status'].update({
            'fix_type': gps_enhanced_data.get('fix_type', 0),
            'hdop': gps_enhanced_data.get('hdop', 0.0),
            'vdop': gps_enhanced_data.get('vdop', 0.0),
            'satellites_visible': gps_enhanced_data.get('satellites_visible', 0),
            'home_position_set': gps_enhanced_data.get('home_position_set', False),
            'home_lat': gps_enhanced_data.get('home_lat', 0.0),
            'home_lon': gps_enhanced_data.get('home_lon', 0.0),
            'home_alt': gps_enhanced_data.get('home_alt', 0.0),
            'distance_to_home': gps_enhanced_data.get('distance_to_home', 0.0),
            'bearing_to_home': gps_enhanced_data.get('bearing_to_home', 0.0)
        })
        
        # Update navigation data if available
        if gps_enhanced_data.get('satellites_visible', 0) > 0:
            self.latest_data['navigation_data']['gps_satellites'] = gps_enhanced_data.get('satellites_visible', 0)
            self.latest_data['navigation_data']['gps_fix'] = gps_enhanced_data.get('fix_type', 0)
    
    def log_telemetry_data(self, data_type, data):
        """Log drone telemetry data to appropriate loggers"""
        if not self.data_logging_enabled:
            return
            
        try:
            if data_type == 'power' and self.telemetry_loggers.get('power'):
                # Log power data
                timestamp = data.get('timestamp', datetime.now().isoformat())
                voltage = data.get('voltage', 0.0)
                current = data.get('current', 0.0)
                consumption = data.get('consumption_mah', 0)
                cells = data.get('cells', 4)
                flight_time = data.get('estimated_flight_time', 0.0)
                
                power_line = f"{timestamp},{voltage:.2f},{current:.2f},{consumption},{cells},{flight_time:.1f}\n"
                self.telemetry_loggers['power'].write(power_line)
                self.telemetry_loggers['power'].flush()
                
            elif data_type == 'motors' and self.telemetry_loggers.get('motors'):
                # Log ESC/motor data
                timestamp = data.get('timestamp', datetime.now().isoformat())
                esc_status = data.get('esc_status', {})
                
                esc_line = f"{timestamp},"
                for esc_id in ['esc_1', 'esc_2', 'esc_3', 'esc_4']:
                    if esc_id in esc_status:
                        esc_data = esc_status[esc_id]
                        esc_line += f"{esc_data.get('temperature', 0)},{esc_data.get('voltage', 0):.1f},{esc_data.get('current', 0):.1f},{esc_data.get('rpm', 0)}"
                    else:
                        esc_line += "0,0.0,0.0,0"
                    if esc_id != 'esc_4':
                        esc_line += ","
                
                esc_line += "\n"
                self.telemetry_loggers['motors'].write(esc_line)
                self.telemetry_loggers['motors'].flush()
                
            elif data_type == 'flight_modes' and self.telemetry_loggers.get('flight_modes'):
                # Log flight mode changes
                timestamp = data.get('timestamp', datetime.now().isoformat())
                flight_mode = data.get('flight_mode', 'UNKNOWN')
                armed_status = data.get('armed_status', False)
                arming_state = data.get('arming_state', 'STANDBY')
                
                mode_line = f"{timestamp},{flight_mode},{armed_status},{arming_state}\n"
                self.telemetry_loggers['flight_modes'].write(mode_line)
                self.telemetry_loggers['flight_modes'].flush()
                
            elif data_type == 'gps' and self.telemetry_loggers.get('gps'):
                # Log enhanced GPS data
                timestamp = data.get('timestamp', datetime.now().isoformat())
                fix_type = data.get('fix_type', 0)
                satellites = data.get('satellites_visible', 0)
                hdop = data.get('hdop', 0.0)
                vdop = data.get('vdop', 0.0)
                home_set = data.get('home_position_set', False)
                distance_home = data.get('distance_to_home', 0.0)
                
                gps_line = f"{timestamp},{fix_type},{satellites},{hdop:.2f},{vdop:.2f},{home_set},{distance_home:.1f}\n"
                self.telemetry_loggers['gps'].write(gps_line)
                self.telemetry_loggers['gps'].flush()
                
        except Exception as e:
            logger.error(f"Error logging {data_type} data: {e}")
    
    def start_comprehensive_logging(self):
        """Start comprehensive drone telemetry logging"""
        try:
            import os
            sensor_log_dir = "Sensor Log"  # Following memory requirement [[memory:7979913]]
            if not os.path.exists(sensor_log_dir):
                os.makedirs(sensor_log_dir)
                logger.info(f"Created Sensor Log directory: {sensor_log_dir}")
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Create separate log files for each telemetry type
            log_files = {
                'ahrs': f"{sensor_log_dir}/drone_ahrs_{timestamp}.csv",
                'gps': f"{sensor_log_dir}/drone_gps_{timestamp}.csv", 
                'power': f"{sensor_log_dir}/drone_battery_{timestamp}.csv",
                'motors': f"{sensor_log_dir}/drone_motors_{timestamp}.csv",
                'flight_modes': f"{sensor_log_dir}/drone_flight_modes_{timestamp}.csv",
                'debug': f"{sensor_log_dir}/drone_debug_{timestamp}.txt"
            }
            
            # Initialize loggers with headers
            for log_type, filename in log_files.items():
                if log_type != 'debug':
                    self.telemetry_loggers[log_type] = open(filename, 'w', newline='', encoding='utf-8')
                    
                    # Write headers
                    if log_type == 'ahrs':
                        header = "Timestamp,Roll_Angle,Pitch_Angle,Yaw_Angle,Altitude,Setpoints\n"
                    elif log_type == 'gps':
                        header = "Timestamp,Fix_Type,Satellites,HDOP,VDOP,Home_Set,Distance_Home\n"
                    elif log_type == 'power':
                        header = "Timestamp,Voltage,Current,Consumption_mAh,Cells,Flight_Time_Min\n"
                    elif log_type == 'motors':
                        header = "Timestamp,ESC1_Temp,ESC1_Volt,ESC1_Curr,ESC1_RPM,ESC2_Temp,ESC2_Volt,ESC2_Curr,ESC2_RPM,ESC3_Temp,ESC3_Volt,ESC3_Curr,ESC3_RPM,ESC4_Temp,ESC4_Volt,ESC4_Curr,ESC4_RPM\n"
                    elif log_type == 'flight_modes':
                        header = "Timestamp,Flight_Mode,Armed,Arming_State\n"
                    
                    self.telemetry_loggers[log_type].write(header)
                else:  # debug file
                    self.telemetry_loggers[log_type] = open(filename, 'w', newline='', encoding='utf-8')
            
            self.data_logging_enabled = True
            self.log_start_time = datetime.now()
            
            logger.info(f"📊 Comprehensive drone telemetry logging started at {self.log_start_time}")
            logger.info(f"📊 Log files created: {list(log_files.values())}")
            
            return {'status': 'success', 'files': log_files}
            
        except Exception as e:
            logger.error(f"Error starting comprehensive logging: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def stop_comprehensive_logging(self):
        """Stop comprehensive drone telemetry logging"""
        try:
            duration = None
            if self.log_start_time:
                duration = (datetime.now() - self.log_start_time).total_seconds()
            
            # Close all loggers
            for log_type, logger_file in self.telemetry_loggers.items():
                if logger_file:
                    self.telemetry_loggers[log_type].close()
                    self.telemetry_loggers[log_type] = None
            
            self.data_logging_enabled = False
            self.log_start_time = None
            
            logger.info(f"📊 Comprehensive drone telemetry logging stopped after {duration:.1f} seconds")
            return {'status': 'success', 'duration': duration}
            
        except Exception as e:
            logger.error(f"Error stopping comprehensive logging: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def start_real_time_broadcast(self):
        """Start real-time data broadcasting via WebSocket"""
        if hasattr(self, '_broadcast_thread') and self._broadcast_thread.is_alive():
            return
        
        self._broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self._broadcast_thread.start()
        logger.info("📡 Real-time data broadcasting started (10Hz)")
    
    def _broadcast_loop(self):
        """Continuous broadcast loop for real-time updates"""
        import time
        logger.info("📡 Starting real-time broadcast loop")
        
        while True:
            try:
                if self.is_connected:
                    # Broadcast all current data via SocketIO if available
                    if hasattr(self, 'socketio'):
                        # Create flattened data structure that the frontend expects
                        flat_data = self._create_flattened_telemetry()
                        
                        # Emit multiple events for different parts
                        self.socketio.emit('telemetry_update', flat_data)
                        self.socketio.emit('ahrs_data', self.latest_data['ahrs'])
                        self.socketio.emit('flight_data', self.latest_data['flight_data'])
                        self.socketio.emit('drone_telemetry', self.latest_data['drone_telemetry'])
                        self.socketio.emit('power_system', self.latest_data['power_system'])
                        if 'navigation_data' in self.latest_data:
                            self.socketio.emit('navigation_data', self.latest_data['navigation_data'])
                        self.socketio.emit('system_status', self.latest_data['system_status'])
                
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                logger.error(f"Error in broadcast loop: {e}")
                time.sleep(1)
    
    def _create_flattened_telemetry(self):
        """Create flattened telemetry data structure that matches frontend expectations"""
        flat_data = {}
        
        # Add AHRS data directly to top level
        if 'ahrs' in self.latest_data and self.latest_data['ahrs']:
            ahrs = self.latest_data['ahrs']
            flat_data.update({
                'roll_angle': ahrs.get('roll_angle', 0),
                'pitch_angle': ahrs.get('pitch_angle', 0),
                'yaw_angle': ahrs.get('yaw_angle', 0),
                'altitude': ahrs.get('altitude', 0),
                'barometric_altitude': ahrs.get('altitude', 0)  # Legacy field name
            })
        
        # Add drone telemetry data
        if 'drone_telemetry' in self.latest_data and self.latest_data['drone_telemetry']:
            drone = self.latest_data['drone_telemetry']
            flat_data.update({
                'battery_current': drone.get('battery_current', 0),
                'flight_mode': drone.get('flight_mode', 'Unknown'),
                'armed_status': drone.get('armed_status', False),
                'gps_fix': drone.get('gps_status', {}).get('fix_type', 0),
                'gps_satellites': drone.get('gps_status', {}).get('satellites_visible', 0),
                'vertical_speed': drone.get('vertical_speed', 0)
            })
            
            # Add GPS coordinates if available
            if 'gps_status' in drone and drone['gps_status']:
                gps_status = drone['gps_status']
                flat_data.update({
                    'latitude': gps_status.get('home_latitude', 0),
                    'longitude': gps_status.get('home_longitude', 0),
                    'gps_speed': gps_status.get('ground_speed', 0)
                })
        
        # Add navigation data if available
        if 'navigation_data' in self.latest_data and self.latest_data['navigation_data']:
            nav = self.latest_data['navigation_data']
            flat_data.update({
                'latitude': nav.get('gps_latitude', 0),  # Charts expect this field name
                'longitude': nav.get('gps_longitude', 0),  # Charts expect this field name
                'gps_heading': nav.get('heading', 0),
                'estimated_speed': nav.get('ground_speed', 0),
                'gps_fix': nav.get('gps_fix', 0),
                'gps_satellites': nav.get('gps_satellites', 0)
            })
        
        # Add system status
        if 'system_status' in self.latest_data and self.latest_data['system_status']:
            status = self.latest_data['system_status']
            flat_data.update({
                'data_rate_ahrs': status.get('data_rate_ahrs', 1),
                'data_rate_gps': status.get('data_rate_gps', 1),
                'connection_quality': status.get('connection_quality', 100)
            })
        
        # Add power system data
        if 'power_system' in self.latest_data and self.latest_data['power_system']:
            power = self.latest_data['power_system']
            flat_data.update({
                'battery_voltage': power.get('battery_voltage', 0),  # Charts expect this field name
                'cell_voltage': power.get('cell_voltage', []),
                'total_voltage': power.get('total_voltage', 0),
                'remaining_capacity': power.get('remaining_capacity', 100)
            })
        
        flat_data['timestamp'] = datetime.now().isoformat()
        return flat_data
    
    def run(self, host='127.0.0.1', port=5001, debug=False):
        """Run the ground station web server"""
        logger.info(f"Starting KMU Ground Station on {host}:{port}")
        
        # Start real-time broadcasting
        self.start_real_time_broadcast()
        
        # Use SocketIO for real-time updates
        self.socketio.run(self.app, host=host, port=port, debug=debug)

if __name__ == '__main__':
    gs = KMUGroundStation()
    gs.run(debug=False)  # Use default port or any available port
