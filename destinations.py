# Define drone flight destinations
# Format: 'name': (latitude, longitude)

DESTINATIONS = {
    'A': (37.7749, -122.4194),      # San Francisco example
    'B': (34.0522, -118.2437),      # Los Angeles example
    'home': (40.7128, -74.0060),    # New York example
    'base': (51.5074, -0.1278),     # London example
}

# Default takeoff/landing location
HOME_LOCATION = (40.7128, -74.0060)

# Flight parameters (easy to modify)
TAKEOFF_ALTITUDE = 5  # meters
FLIGHT_SPEED = 1.0  # velocity in m/s
YAW_SENSITIVITY = 0.01  # yaw response to bearing difference
OBSTACLE_TURN_RATE = 0.5  # rotation speed when obstacle detected

# Object detection sensitivity
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3

# Obstacle avoidance objects
# NOTE: This is overridden by config.json in drone_control.py
# All 103 obstacle classes from obstacles.names are loaded automatically
OBSTACLE_OBJECTS = ['UMBRELLA', 'PERSON', 'CAR', 'TRUCK', 'BICYCLE']