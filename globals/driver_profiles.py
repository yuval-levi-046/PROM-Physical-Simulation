#config/driver_profiles.py

# Basic driver profile used by default spawners or simulation setups
BASIC_DRIVER = {
    "name": "basic",
    "acceleration": 30,   # px/s^2
    "max_speed": 300,       # px/s
   "threshold": 5,        # safe distance in meters
    "reaction_time": 0
}

# In the future, define more profiles:
# CAUTIOUS_DRIVER = {...}
# AGGRESSIVE_DRIVER = {...}


DRIVER_PROFILES = {
    "basic": BASIC_DRIVER
}