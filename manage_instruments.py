#!/usr/bin/env python3
"""
KMU Ground Station - Flight Instruments Source File Manager

This script helps manage the Adobe Illustrator source files and SVG exports
for the flight instruments.

Usage:
    python manage_instruments.py --list          # List all source files
    python manage_instruments.py --backup        # Backup current SVG files
    python manage_instruments.py --restore       # Restore from backup
    python manage_instruments.py --validate      # Validate SVG files
"""

import os
import shutil
import argparse
import xml.etree.ElementTree as ET
from datetime import datetime

class InstrumentManager:
    def __init__(self):
        self.source_dir = "source_files"
        self.svg_dir = "static/img"
        self.backup_dir = "backup_svg"
        
    def list_source_files(self):
        """List all available source files"""
        print("📁 Available Source Files:")
        print("=" * 50)
        
        if os.path.exists(self.source_dir):
            for file in os.listdir(self.source_dir):
                if file.endswith('.ai'):
                    file_path = os.path.join(self.source_dir, file)
                    size = os.path.getsize(file_path) / 1024  # KB
                    print(f"🎨 {file:<20} ({size:.1f} KB)")
        else:
            print("❌ Source files directory not found")
            
        print("\n📁 Available SVG Files:")
        print("=" * 50)
        
        if os.path.exists(self.svg_dir):
            for file in os.listdir(self.svg_dir):
                if file.endswith('.svg'):
                    file_path = os.path.join(self.svg_dir, file)
                    size = os.path.getsize(file_path) / 1024  # KB
                    print(f"🖼️  {file:<20} ({size:.1f} KB)")
        else:
            print("❌ SVG files directory not found")
    
    def backup_svg_files(self):
        """Backup current SVG files"""
        if not os.path.exists(self.svg_dir):
            print("❌ SVG directory not found")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = f"{self.backup_dir}_{timestamp}"
        
        try:
            shutil.copytree(self.svg_dir, backup_path)
            print(f"✅ SVG files backed up to: {backup_path}")
        except Exception as e:
            print(f"❌ Backup failed: {e}")
    
    def restore_from_backup(self):
        """Restore SVG files from latest backup"""
        backup_dirs = [d for d in os.listdir('.') if d.startswith(self.backup_dir)]
        
        if not backup_dirs:
            print("❌ No backup directories found")
            return
            
        latest_backup = max(backup_dirs)
        
        try:
            if os.path.exists(self.svg_dir):
                shutil.rmtree(self.svg_dir)
            shutil.copytree(latest_backup, self.svg_dir)
            print(f"✅ Restored from backup: {latest_backup}")
        except Exception as e:
            print(f"❌ Restore failed: {e}")
    
    def validate_svg_files(self):
        """Validate SVG files for proper structure"""
        print("🔍 Validating SVG Files:")
        print("=" * 50)
        
        if not os.path.exists(self.svg_dir):
            print("❌ SVG directory not found")
            return
            
        required_files = [
            'fi_box.svg', 'fi_circle.svg', 'fi_needle.svg', 'fi_needle_small.svg',
            'horizon_back.svg', 'horizon_ball.svg', 'horizon_circle.svg',
            'heading_yaw.svg', 'speed_mechanics.svg', 'vertical_mechanics.svg',
            'turn_coordinator.svg', 'altitude_ticks.svg', 'altitude_pressure.svg'
        ]
        
        for file in required_files:
            file_path = os.path.join(self.svg_dir, file)
            if os.path.exists(file_path):
                try:
                    # Try to parse SVG
                    tree = ET.parse(file_path)
                    root = tree.getroot()
                    
                    # Check for basic SVG structure
                    if root.tag.endswith('svg'):
                        size = os.path.getsize(file_path) / 1024
                        print(f"✅ {file:<20} ({size:.1f} KB) - Valid SVG")
                    else:
                        print(f"❌ {file:<20} - Invalid SVG structure")
                except Exception as e:
                    print(f"❌ {file:<20} - Parse error: {e}")
            else:
                print(f"❌ {file:<20} - Missing")
    
    def create_customization_guide(self):
        """Create a guide for customizing instruments"""
        guide = """
# Flight Instruments Customization Guide

## Adobe Illustrator Workflow

### 1. Opening Source Files
- Open Adobe Illustrator
- File → Open → Select the .ai file from `source_files/`
- Each file contains layers for different instrument components

### 2. Making Modifications
- **Colors**: Select elements and change fill/stroke colors
- **Text**: Edit text elements for markings and labels
- **Shapes**: Modify or add geometric shapes
- **Layers**: Organize elements using layers

### 3. Exporting to SVG
- File → Export → Export As...
- Choose SVG format
- Settings:
  - SVG Profiles: SVG 1.1
  - Fonts: Convert to Outlines
  - Images: Link
  - Preserve Illustrator Editing Capabilities: Unchecked

### 4. Replacing Files
- Copy the exported SVG to `static/img/`
- Replace the corresponding file
- Restart the ground station to see changes

## Instrument-Specific Modifications

### Attitude Indicator (horizon.ai)
- Modify ground/sky colors in gradient
- Adjust horizon line thickness
- Change pitch markings

### Heading Indicator (heading.ai)
- Modify compass rose colors
- Change NSEW text styling
- Adjust tick mark spacing

### Altimeter (altitude.ai)
- Modify altitude markings
- Change needle colors
- Adjust pressure setting display

### Airspeed Indicator (speed.ai)
- Modify speed markings
- Change needle styling
- Adjust color scheme

### Variometer (vertical.ai)
- Modify climb/descent indicators
- Change color coding
- Adjust sensitivity markings

### Turn Coordinator (turn_coordinator.svg)
- Modify airplane symbol
- Change turn rate indicators
- Adjust ball movement

## Best Practices

1. **Keep Originals**: Always keep backup of original files
2. **Test Incrementally**: Make small changes and test frequently
3. **Consistent Styling**: Maintain consistent colors and fonts
4. **Performance**: Optimize SVG file sizes for web use
5. **Compatibility**: Test across different browsers

## Troubleshooting

### SVG Not Displaying
- Check file permissions
- Verify SVG syntax
- Check browser console for errors

### Performance Issues
- Optimize SVG file sizes
- Remove unnecessary elements
- Use efficient color schemes

### Visual Problems
- Check viewBox settings
- Verify element positioning
- Test different screen sizes
"""
        
        with open("INSTRUMENT_CUSTOMIZATION_GUIDE.md", "w") as f:
            f.write(guide)
        
        print("✅ Created customization guide: INSTRUMENT_CUSTOMIZATION_GUIDE.md")

def main():
    parser = argparse.ArgumentParser(description="Manage flight instrument source files")
    parser.add_argument("--list", action="store_true", help="List all source files")
    parser.add_argument("--backup", action="store_true", help="Backup current SVG files")
    parser.add_argument("--restore", action="store_true", help="Restore from backup")
    parser.add_argument("--validate", action="store_true", help="Validate SVG files")
    parser.add_argument("--guide", action="store_true", help="Create customization guide")
    
    args = parser.parse_args()
    
    manager = InstrumentManager()
    
    if args.list:
        manager.list_source_files()
    elif args.backup:
        manager.backup_svg_files()
    elif args.restore:
        manager.restore_from_backup()
    elif args.validate:
        manager.validate_svg_files()
    elif args.guide:
        manager.create_customization_guide()
    else:
        print("KMU Ground Station - Flight Instruments Manager")
        print("=" * 50)
        print("Available commands:")
        print("  --list     List all source files")
        print("  --backup   Backup current SVG files")
        print("  --restore  Restore from backup")
        print("  --validate Validate SVG files")
        print("  --guide    Create customization guide")

if __name__ == "__main__":
    main()
