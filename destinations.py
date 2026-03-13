# Define drone flight destinations
# Format: 'name': (latitude, longitude)

DESTINATIONS = {
    'A': (37.7749, -122.4194),      # San Francisco example
    'B': (34.0522, -118.2437),      # Los Angeles example
    'home': (40.7128, -74.0060),    # New York example
    'base': (51.5074, -0.1278),     # London example
}

HOME_LOCATION = (40.7128, -74.0060)

TAKEOFF_ALTITUDE = 5  # meters
FLIGHT_SPEED = 1.0  # velocity in m/s
YAW_SENSITIVITY = 0.01 
OBSTACLE_TURN_RATE = 0.5

CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3

OBSTACLE_OBJECTS = ['UMBRELLA', 'PERSON', 'CAR', 'TRUCK', 'BICYCLE']