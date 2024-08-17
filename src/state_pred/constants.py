"""This module contains simulation constants/settings
"""

import numpy as np

# PHYSICAL PROPERTIES

DIST_REAR_AXEL = 0.85  # m
DIST_FRONT_AXEL = 0.85  # m
WHEEL_BASE = DIST_REAR_AXEL + DIST_FRONT_AXEL

MASS = 15  # kg

CORNERING_STIFF_FRONT = 16.0 * 2.0  # N/rad
CORNERING_STIFF_REAR = 17.0 * 2.0  # N/rad

YAW_INERTIA = 22  # kg/(m^2)

# INPUT CONSTRAINTS

SPEED_MIN = 2  # m/s
SPEED_MAX = 10  # m/s

STEER_ANGLE_MAX = np.radians(37)  # rad
STEER_ANGLE_DELTA = np.radians(1)  # rad

ACCELERATION_MIN = -5  # m/(s^2)
ACCELERATION_MAX = 10  # m/(s^2)

STEER_ACC_MIN = -5
STEER_ACC_MAX = 5

DT = 0.1  # (s)
