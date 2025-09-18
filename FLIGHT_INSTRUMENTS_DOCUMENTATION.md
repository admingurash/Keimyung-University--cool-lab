# KMU Ground Station - Flight Instruments Documentation

## Overview
This document describes the complete flight instruments implementation for the KMU Ground Station, based on the jQuery Flight Indicators library.

## File Structure

```
KMU Ground station/
├── static/
│   ├── css/
│   │   └── flightindicators.css          # Original jQuery Flight Indicators CSS
│   ├── js/
│   │   └── jquery.flightindicators.js    # Original jQuery Flight Indicators JS
│   └── img/                              # All SVG instrument graphics
│       ├── fi_box.svg                    # Instrument box with shadows
│       ├── fi_circle.svg                 # Circular backgrounds
│       ├── fi_needle.svg                 # Main instrument needles
│       ├── fi_needle_small.svg           # Small needles (altimeter)
│       ├── fi_tc_airplane.svg            # Turn coordinator airplane
│       ├── horizon_back.svg              # Artificial horizon background
│       ├── horizon_ball.svg              # Artificial horizon ball
│       ├── horizon_circle.svg            # Attitude indicator circle
│       ├── horizon_mechanics.svg         # Attitude mechanics
│       ├── heading_yaw.svg               # Heading indicator
│       ├── heading_mechanics.svg         # Heading mechanics
│       ├── speed_mechanics.svg           # Airspeed indicator
│       ├── vertical_mechanics.svg        # Variometer
│       ├── turn_coordinator.svg          # Turn coordinator
│       ├── altitude_ticks.svg            # Altimeter ticks
│       └── altitude_pressure.svg         # Altimeter pressure setting
├── source_files/                         # Adobe Illustrator source files
│   ├── altitude.ai                       # Altimeter source design
│   ├── heading.ai                        # Heading indicator source design
│   ├── horizon.ai                        # Artificial horizon source design
│   ├── speed.ai                          # Airspeed indicator source design
│   └── vertical.ai                       # Variometer source design
└── templates/
    └── index.html                        # Main web interface with flight instruments
```

## Flight Instruments

### 1. Attitude Indicator (Artificial Horizon)
- **Purpose**: Shows aircraft roll and pitch angles
- **Data Source**: `roll_angle`, `pitch_angle` from AHRS
- **Display**: Brown ground, blue sky, white horizon line
- **Features**: Realistic artificial horizon with pitch markings

### 2. Heading Indicator
- **Purpose**: Shows aircraft heading/yaw
- **Data Source**: `yaw_angle` from AHRS
- **Display**: Compass rose with NSEW markings
- **Features**: Rotating compass card

### 3. Altimeter
- **Purpose**: Shows barometric altitude
- **Data Source**: `barometric_altitude` from AHRS
- **Display**: Altitude in feet (aviation standard)
- **Features**: Two needles (main and 10,000ft), pressure setting

### 4. Airspeed Indicator
- **Purpose**: Shows aircraft speed
- **Data Source**: `gps_speed` or estimated speed
- **Display**: Speed in knots (aviation standard)
- **Features**: Speed markings and needle

### 5. Vertical Speed Indicator (Variometer)
- **Purpose**: Shows climb/descent rate
- **Data Source**: `vertical_speed` from AHRS
- **Display**: Vertical speed in feet per minute
- **Features**: Climb/descent indicators

### 6. Turn Coordinator
- **Purpose**: Shows turn rate
- **Data Source**: `yaw_rate` from AHRS
- **Display**: Turn rate in degrees per second
- **Features**: Airplane symbol, turn indicators

## Source Files Usage

### Adobe Illustrator Files (.ai)
The `source_files/` directory contains the original Adobe Illustrator designs for each instrument:

- **altitude.ai**: Complete altimeter design with all layers
- **heading.ai**: Heading indicator with compass rose
- **horizon.ai**: Artificial horizon with ground/sky gradients
- **speed.ai**: Airspeed indicator design
- **vertical.ai**: Variometer design

### Customization Options
1. **Modify Colors**: Open .ai files in Adobe Illustrator to change colors
2. **Add Markings**: Add custom markings or text
3. **Resize Elements**: Adjust sizes for different display requirements
4. **Export to SVG**: Export modified designs as SVG for web use

### Export Process
1. Open the desired .ai file in Adobe Illustrator
2. Make your modifications
3. File → Export → SVG
4. Replace the corresponding file in `static/img/`

## Technical Implementation

### JavaScript Integration
```javascript
// Initialize flight instruments
var attitude = $.flightIndicator('#attitude-instrument', 'attitude', {
    size: 200,
    roll: 0,
    pitch: 0,
    showBox: true,
    img_directory: '/static/img/'
});

// Update with real data
attitude.setRoll(data.roll_angle);
attitude.setPitch(data.pitch_angle);
```

### CSS Customization
The `static/css/flightindicators.css` file contains all styling:
- Instrument sizing and positioning
- Animation effects
- Color schemes
- Responsive design

### Data Mapping
```javascript
// AHRS data to instrument mapping
updateFlightInstruments({
    roll_angle: data.roll_angle,        // Attitude indicator
    pitch_angle: data.pitch_angle,      // Attitude indicator
    yaw_angle: data.yaw_angle,          // Heading indicator
    barometric_altitude: data.altitude,  // Altimeter
    gps_speed: data.speed,              // Airspeed indicator
    vertical_speed: data.vario,         // Variometer
    yaw_rate: data.turn_rate            // Turn coordinator
});
```

## Features

### Real-time Updates
- 20+ Hz update rate
- Smooth animations
- Visual feedback on data changes

### Responsive Design
- Works on desktop and mobile
- Adaptive grid layout
- Touch-friendly controls

### Theme Support
- Dark/light theme toggle
- Customizable colors
- Professional aviation styling

### Data Validation
- Range checking for all instruments
- Fallback displays if data is missing
- Error handling for invalid values

## Usage Instructions

1. **Start the Ground Station**:
   ```bash
   python ground_station.py
   ```

2. **Access the Web Interface**:
   - Open browser to `http://localhost:5000`
   - Navigate to the Flight Instruments section

3. **Connect to Flight Controller**:
   - Use the connection panel
   - Instruments will automatically display real data

4. **Customize Display**:
   - Use Dark/Light theme buttons
   - Monitor data rate
   - Check instrument status indicators

## Troubleshooting

### Instruments Not Displaying
- Check browser console for JavaScript errors
- Verify SVG files are loading correctly
- Ensure jQuery is loaded before flight indicators

### Data Not Updating
- Check Socket.IO connection
- Verify AHRS data is being received
- Check data format matches expected structure

### Performance Issues
- Reduce update frequency if needed
- Optimize SVG file sizes
- Use hardware acceleration for animations

## Future Enhancements

1. **Additional Instruments**: Add more aviation instruments
2. **Custom Themes**: Create KMU-specific color schemes
3. **3D Instruments**: Implement 3D instrument displays
4. **Recording**: Add instrument data recording capabilities
5. **Alerts**: Add visual/audio alerts for critical values

## Credits

- **Original Library**: jQuery Flight Indicators by Sébastien Matton
- **License**: GPLv3
- **Source**: https://github.com/sebmatton/jQuery-Flight-Indicators

## Support

For issues or questions about the flight instruments implementation, refer to:
- Original library documentation
- Browser developer tools for debugging
- Flight controller data format specifications
