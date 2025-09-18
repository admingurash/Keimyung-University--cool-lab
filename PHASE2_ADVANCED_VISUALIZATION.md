# Phase 2: Advanced Visualization Features

## Overview
This document describes the advanced visualization features implemented in the KMU Ground Station Phase 2 upgrade. These features provide enhanced data analysis, 3D visualization, and real-time frequency analysis capabilities.

## Features Implemented

### 1. Enhanced Charts with Multi-Series Data Plotting

#### Multi-Series Chart
- **Location**: Advanced Charts tab
- **Features**:
  - Real-time plotting of multiple data series simultaneously
  - Interactive legend to toggle series visibility
  - Dual Y-axis support (attitude degrees and voltage)
  - Zoom and pan functionality
  - Data export to CSV format
  - Configurable data point limits (200 points by default)

#### Supported Data Series:
- **Attitude Data**: Roll, Pitch, Yaw angles
- **Power System**: Battery voltage, current, percentage
- **Navigation**: GPS coordinates, altitude
- **PID Data**: PID gain values and outputs

#### Controls:
- `Attitude` button: Toggle roll/pitch/yaw series
- `Navigation` button: Toggle GPS and altitude data
- `Power` button: Toggle battery voltage series
- `PID` button: Toggle PID-related data
- `Export Data` button: Download current chart data as CSV

### 2. Real-time Spectrogram

#### Spectrogram Analysis
- **Location**: Advanced Charts tab
- **Features**:
  - Real-time frequency analysis of sensor data
  - FFT-based spectrum analysis (1024-point FFT)
  - Hot colormap visualization
  - Multiple sensor source selection
  - Configurable update rate (100ms intervals)

#### Sensor Sources:
- **Gyroscope**: Roll, pitch, yaw angular velocities
- **Accelerometer**: Derived acceleration data
- **Magnetometer**: Magnetic field strength variations
- **Pressure**: Altitude and pressure sensor data

#### Controls:
- `Start Analysis`: Begin real-time spectrogram generation
- `Stop Analysis`: Pause spectrogram updates
- `Clear`: Reset spectrogram display
- `Source Dropdown`: Select sensor data source

#### Technical Details:
- Uses FFT-JS library for frequency analysis
- 1024-point FFT buffer for high-resolution analysis
- Real-time canvas rendering with hot colormap
- Automatic buffer management to prevent memory issues

### 3. Log Analysis Tools

#### Historical Data Analysis
- **Location**: Log Analysis tab
- **Features**:
  - Multi-file log analysis support
  - CSV, TXT, and JSON file format support
  - Statistical analysis and reporting
  - Interactive chart visualization
  - Export analysis results

#### Supported File Formats:
- **CSV**: Comma-separated values (primary format)
- **TXT**: Text-based log files
- **JSON**: JSON-structured data files

#### Analysis Features:
- **Data Statistics**: Min/max values, ranges, duration
- **Visualization**: Multi-series charts for all parameters
- **Export**: Analysis reports in text format
- **Multi-file Support**: Combine multiple log files

#### Controls:
- `Load Log Files`: Select and load log files
- `Analyze Data`: Process loaded files and generate charts
- `Export Analysis`: Download analysis report

#### Statistics Generated:
- Total data points
- Flight duration
- Roll/Pitch/Yaw ranges
- Altitude range
- Data quality metrics

### 4. Cesium-based 3D Visualization

#### 3D Flight Visualization
- **Location**: 3D Visualization tab
- **Features**:
  - Real-time 3D flight path tracking
  - Terrain awareness with elevation data
  - Waypoint visualization
  - Obstacle detection and display
  - Camera following and control

#### 3D Features:
- **Flight Path**: Real-time yellow polyline showing aircraft trajectory
- **Terrain**: World terrain with elevation data
- **Waypoints**: Mission waypoint markers
- **Obstacles**: 3D obstacle visualization (buildings, towers)
- **Camera Control**: Automatic following with manual override

#### Controls:
- `Reset View`: Return to default camera position
- `Toggle Terrain`: Enable/disable terrain rendering
- `Toggle Flight Path`: Show/hide flight trajectory
- `Toggle Waypoints`: Show/hide mission waypoints
- `Toggle Obstacles`: Show/hide obstacle markers

#### Technical Implementation:
- Uses Cesium.js for 3D rendering
- World terrain provider for realistic elevation
- Entity-based visualization system
- Real-time position updates from GPS data
- Automatic camera following with configurable orientation

### 5. Terrain Awareness Features

#### Elevation and Obstacle Detection
- **Features**:
  - Real-time terrain elevation display
  - Obstacle detection and visualization
  - Safety altitude warnings
  - Terrain following capabilities

#### Obstacle Types:
- **Buildings**: 3D box representations
- **Towers**: Cylindrical obstacle markers
- **Terrain**: Natural elevation changes
- **Custom**: User-defined obstacle markers

#### Safety Features:
- Visual obstacle warnings
- Altitude clearance indicators
- Terrain proximity alerts
- Flight path conflict detection

## Technical Architecture

### Data Flow
1. **Real-time Data**: AHRS and GPS data feed all visualization components
2. **Multi-series Chart**: Receives attitude and power data
3. **Spectrogram**: Processes sensor data through FFT analysis
4. **3D Visualization**: Updates flight path and camera position
5. **Log Analysis**: Processes historical data files

### Performance Optimizations
- **Data Point Limits**: Configurable maximum data points per chart
- **Update Rates**: Optimized refresh rates for different components
- **Memory Management**: Automatic buffer cleanup and garbage collection
- **Canvas Optimization**: Efficient rendering with device pixel ratio support

### Browser Compatibility
- **Modern Browsers**: Chrome, Firefox, Safari, Edge
- **WebGL Support**: Required for Cesium 3D visualization
- **Canvas Support**: Required for spectrogram and charts
- **File API**: Required for log analysis features

## Usage Instructions

### Getting Started
1. **Connect to Flight Controller**: Establish serial connection
2. **Navigate to Advanced Charts**: Click on "Advanced Charts" tab
3. **Start Visualizations**: Use control buttons to activate features
4. **Monitor Data**: Observe real-time updates in all visualization components

### Advanced Features
1. **Spectrogram Analysis**: Select sensor source and start analysis
2. **3D Visualization**: Use camera controls to explore flight path
3. **Log Analysis**: Load historical data files for analysis
4. **Data Export**: Export charts and analysis results

### Troubleshooting
- **3D Visualization Not Loading**: Check internet connection for Cesium library
- **Spectrogram Not Working**: Ensure sensor data is being received
- **Log Analysis Errors**: Verify file format compatibility
- **Performance Issues**: Reduce data point limits or update rates

## Future Enhancements

### Planned Features
- **Machine Learning Integration**: Anomaly detection and pattern recognition
- **Advanced Filtering**: Kalman filtering and signal processing
- **Custom Visualizations**: User-defined chart configurations
- **Real-time Alerts**: Configurable warning systems
- **Data Streaming**: WebSocket-based real-time data streaming

### Performance Improvements
- **Web Workers**: Background processing for heavy computations
- **Data Compression**: Efficient data storage and transmission
- **Caching**: Intelligent data caching for improved performance
- **Progressive Loading**: Incremental data loading for large datasets

## Conclusion

Phase 2: Advanced Visualization provides comprehensive data analysis and visualization capabilities for the KMU Ground Station. These features enhance the user experience with real-time analysis, 3D visualization, and historical data processing capabilities.

The implementation follows modern web standards and provides a solid foundation for future enhancements and customizations.
