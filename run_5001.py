#!/usr/bin/env python3
"""
Simple test to run ground station without port restrictions
"""

import os
import sys

# Set environment variables
os.environ['FLASK_APP'] = 'ground_station.py'
os.environ['FLASK_ENV'] = 'development'

print("Environment variables set:")
print(f"FLASK_APP: {os.environ.get('FLASK_APP')}")

# Now import and run
from ground_station import KMUGroundStation

print("Creating ground station...")
gs = KMUGroundStation()

print("Starting ground station...")
gs.run(debug=False)
