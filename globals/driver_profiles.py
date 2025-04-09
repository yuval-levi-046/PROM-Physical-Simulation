BASIC_DRIVER = {
    "name": "basic",
    "desired_speed": 0,
    "idm_params": {
        "a": 2.0,
        "b": 2.0,
        "delta": 4,
        "s0": 2.0,
        "T": 1.5
    },
    "politeness": 0.3,
    "lane_change_threshold": 0.2,
    "bias_left": -0.3,
    "bias_right": 0.3
}

CAUTIOUS_DRIVER = {
    "desired_speed": -10,
    "idm_params": {
        "a": 1.2,
        "b": 2.5,
        "delta": 4,
        "s0": 3.0,
        "T": 2.0
    },
    "politeness": 0.5,
    "lane_change_threshold": 0.1,
    "bias_left": -0.2,
    "bias_right": 0.2
}

AGGRESSIVE_DRIVER = {
    "speed_difference": 20,
    "idm_params": {
        "a": 3.5,
        "b": 1.5,
        "delta": 4,
        "s0": 1.0,
        "T": 1.0
    },
    "politeness": 0.1,
    "lane_change_threshold": 0.3,
    "bias_left": -0.3,
    "bias_right": 0.3
}


DRIVER_PROFILES = {
    "basic": BASIC_DRIVER,
    "random": BASIC_DRIVER,
    "cautious": CAUTIOUS_DRIVER,
    "aggressive": AGGRESSIVE_DRIVER,
}

