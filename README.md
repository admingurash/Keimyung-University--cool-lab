Here is a recommended README for your "KMU Ground Station" repository, tailored to the structure and purpose of your project as seen in your directory:

***

# KMU Ground Station

A professional ground station software for real-time drone telemetry, attitude (AHRS), GPS navigation, PID gain control, and sensor data logging, designed for research and advanced model aircraft projects.

## Features

- **Serial communication** with the flight controller using 3DR Telemetry or compatible modules
- **Live visualization** of attitude (roll, pitch, yaw), heading, altitude, and battery data
- **Real-time map tracking** using OpenStreetMap and Leaflet.js, with position history
- **Switch and failsafe status** monitoring for RC transmitters (SwA, SwC, failsafe)
- **PID gain tuning** over serial using a web-based interface
- **Data logging** to CSV with start/stop controls and automatic log file management
- **Terminal interface** for sending direct commands and debugging
- **Python (Flask + Socket.IO) backend** and modern responsive HTML/CSS/JavaScript frontend

## Directory Structure

| Folder/File                                      | Purpose                                       |
|--------------------------------------------------|-----------------------------------------------|
| `Gain Log/`                                      | PID gain change logs                          |
| `source_files/`                                  | Supporting source files and references        |
| `static/`                                        | Frontend JavaScript, CSS, static assets       |
| `templates/`                                     | HTML templates for Flask web interface        |
| `config.ini`                                     | Application configuration options             |
| `ground_station.py`                              | Main server application and backend logic     |
| `manage_instruments.py`                          | Instrument management utilities               |
| `settings.json`                                  | Saved user/configuration settings             |
| `FLIGHT_INSTRUMENTS_DOCUMENTATION.md`            | Instrument system documentation               |
| `PHASE2_ADVANCED_VISUALIZATION.md`               | Advanced display/visualization documentation  |
| `sample_table_log.txt`                           | Example log output                            |
| (한글)FC↔GCS Data protocol doc v0.9.1.pdf       | Communication protocol specification (Korean) |

## Getting Started

### Prerequisites

- Python 3.7+
- Required Python packages (see below)
- 3DR telemetry radio or compatible serial link
- Web browser (for frontend)

### Installation

1. **Clone this repository:**
   ```
   git clone https://github.com/your-org/kmu-ground-station.git
   cd kmu-ground-station
   ```

2. **Install dependencies:**
   ```
   pip install flask flask-socketio pyserial
   ```

3. **Connect your telemetry radio** and ensure the correct serial port is available.

### Running the Ground Station

Start the server (default port: 5001):
```
python ground_station.py
```


Then open your browser and go to http://127.0.0.1:5001/

### Basic Usage

- Use the interface to connect to your flight controller.
- View real-time AHRS and GPS data.
- Tune PID gains and request updates from the controller.
- Start/stop flight data logging as needed.

## Documentation

- Protocol documentation is provided in (한글)FC↔GCS Data protocol doc v0.9.1.pdf.
- See `FLIGHT_INSTRUMENTS_DOCUMENTATION.md` and `PHASE2_ADVANCED_VISUALIZATION.md` for instrument system details and usage tips.

## Contributing

Pull requests and suggestions are welcome! Please include test logs and a summary of your changes.

## License



***

This template clearly explains your project's functionality, folder structure, setup steps, and usage for new users and collaborators.[1]
