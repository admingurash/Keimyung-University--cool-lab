# KMU Ground Station

A professional ground station software for real-time drone telemetry, attitude (AHRS), GPS navigation, PID gain control, and sensor data logging, designed for research and advanced model aircraft projects.

## 🚀 Features

### **Real-time Telemetry & Visualization**
- **Live AHRS data** - Roll, Pitch, Yaw with smooth 10Hz updates
- **GPS navigation** - Real-time position tracking with map integration
- **Battery monitoring** - Voltage, current, and flight time estimation
- **Clean graph visualization** - Professional charts with double-click data point toggle
- **Flight instruments** - Professional HUD with attitude indicators

### **Advanced Communication**
- **Serial communication** with flight controllers using custom 20-byte binary protocol
- **WebSocket real-time updates** - 10Hz data broadcasting for lag-free experience
- **MAVLink compatibility** - Support for standard drone communication protocols
- **Multi-rotor telemetry** - ESC status, motor health, and flight mode monitoring

### **Data Management**
- **Comprehensive logging** - CSV, TXT, and JSON formats with separate files for each data type
- **AHRS data logging** - Dedicated sensor log folder for attitude data
- **Real-time data export** - Live telemetry streaming and analysis
- **Log analysis tools** - Built-in data visualization and playback

### **User Interface**
- **Modern web interface** - Responsive HTML5/CSS3/JavaScript frontend
- **Interactive charts** - Chart.js with zoom, pan, and data point controls
- **Professional HUD** - Flight instrument displays with real-time updates
- **Terminal interface** - Direct command sending and debugging capabilities

## 📁 Directory Structure

```
KMU Ground Station/
├── ground_station.py              # Main server application
├── manage_instruments.py          # Instrument management utilities
├── config.ini                     # Application configuration
├── settings.json                  # User settings and preferences
├── templates/
│   └── index.html                 # Web interface template
├── static/
│   ├── css/                       # Stylesheets
│   ├── js/                        # JavaScript libraries
│   └── img/                       # Images and icons
├── Gain Log/                      # PID gain change logs
├── Sensor Log/                    # AHRS and telemetry data logs
├── source_files/                  # Supporting source files
└── Documentation/
    ├── FLIGHT_INSTRUMENTS_DOCUMENTATION.md
    ├── PHASE2_ADVANCED_VISUALIZATION.md
    └── (한글)FC↔GCS Data protocol doc v0.9.1.pdf
```

## 🛠️ Installation

### Prerequisites
- **Python 3.7+** (Tested with Python 3.8+)
- **Required packages**: Flask, Flask-SocketIO, PySerial, NumPy
- **Web browser** (Chrome, Firefox, Safari, Edge)
- **Serial connection** (USB, Bluetooth, or telemetry radio)

### Quick Setup

1. **Clone the repository:**
```bash
git clone https://github.com/admingurash/Keimyung-University--cool-lab.git
cd Keimyung-University--cool-lab
```

2. **Install dependencies:**
```bash
pip install flask flask-socketio pyserial numpy
```

3. **Run the ground station:**
```bash
python ground_station.py
```

4. **Open your browser:**
Running on http://127.0.0.1:5001

## 🎮 Usage

### **Basic Operation**
1. **Connect** - Select your serial port and establish connection
2. **Monitor** - View real-time telemetry data and flight instruments
3. **Log** - Start/stop data logging for analysis
4. **Control** - Send commands and tune PID parameters

### **Advanced Features**
- **Double-click charts** to toggle data point visibility
- **Zoom and pan** on graphs for detailed analysis
- **Export logs** in multiple formats (CSV, JSON, TXT)
- **Real-time diagnostics** via `/api/debug_diagnostic` endpoint

### **Graph Controls**
- **Clean view**: Charts display smooth lines without data points
- **Show points**: Double-click any chart to reveal individual data points
- **Hide points**: Double-click again to return to clean line view
- **Interactive**: Hover for values, zoom for details

## 📊 Data Protocol

The ground station supports multiple communication protocols:

### **Custom Binary Protocol**
- **20-byte messages** with sync bytes (0x46 0x43 for FC, 0x47 0x53 for GS)
- **Checksum validation** (0xFF - sum of bytes)
- **Message types**: AHRS (0x10), GPS (0x11), Battery (0x12), ESC (0x13), Flight Mode (0x14), Enhanced GPS (0x15)

### **NMEA GPS Support**
- **Standard GPS sentences** (GGA, RMC, VTG)
- **Real-time position** and navigation data
- **Satellite information** and fix quality

## 🔧 Configuration

### **Serial Settings** (`config.ini`)
```ini
[serial]
port = COM1
baudrate = 57600
timeout = 1.0
```

### **Web Interface** (`settings.json`)
```json
{
    "chart_update_rate": 10,
    "log_directory": "Sensor Log",
    "max_data_points": 200
}
```

## 📈 Performance

- **10Hz real-time updates** - Smooth, lag-free data visualization
- **WebSocket broadcasting** - Efficient real-time communication
- **Optimized rendering** - Clean graphs with minimal CPU usage
- **Memory efficient** - Automatic data point management

## 🐛 Troubleshooting

### **Common Issues**
1. **No data received**: Check serial port connection and baud rate
2. **Charts not updating**: Verify WebSocket connection in browser console
3. **Lag in graphs**: Ensure test data generator is running (10Hz mode)

### **Debug Tools**
- **Diagnostic endpoint**: `GET /api/debug_diagnostic`
- **Test data generator**: `POST /api/test_data`
- **Connection status**: Real-time connection quality monitoring

## 🤝 Contributing

We welcome contributions! Please:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Commit your changes** (`git commit -m 'Add amazing feature'`)
4. **Push to the branch** (`git push origin feature/amazing-feature`)
5. **Open a Pull Request**

### **Development Guidelines**
- Follow PEP 8 Python style guidelines
- Add comments for complex functions
- Test with real flight controller when possible
- Update documentation for new features

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Keimyung University** - Research and development support
- **Open source community** - Chart.js, Flask, and other libraries
- **Drone community** - Feedback and testing support

## 📞 Support

For questions, issues, or contributions:
- **GitHub Issues**: [Create an issue](https://github.com/admingurash/Keimyung-University--cool-lab/issues)
- **Documentation**: Check the documentation files in the repository
- **Protocol Reference**: See `(한글)FC↔GCS Data protocol doc v0.9.1.pdf`

---

**Built with ❤️ for the drone research community at Keimyung University**
