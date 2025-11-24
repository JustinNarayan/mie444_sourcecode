import numpy as np
import math
from encoder_reading import EncoderReading

# Robot parameters
L = 3.569  # inches - DISTANCE_FROM_OBJECT_CENTER_TO_WHEEL_MIDPOINT

DEG_TO_RAD = math.pi / 180.0
THETA_1 = 180 * DEG_TO_RAD  # radians
THETA_2 = 60 * DEG_TO_RAD  # radians
THETA_3 = 300 * DEG_TO_RAD  # radians

A = np.array([
    [-math.sin(THETA_1), math.cos(THETA_1), L],
    [-math.sin(THETA_2), math.cos(THETA_2), L],
    [-math.sin(THETA_3), math.cos(THETA_3), L]
])

# Inverse of A
A_INV = np.linalg.inv(A)

def inverse_kinematics(initial: EncoderReading, current: EncoderReading):
    """
    Compute delta_x, delta_y, delta_theta between two encoder readings.
    """
    inital_readings = initial.get_readings()
    final_readings = current.get_readings()
    motor_displacements = np.array([
        inital_readings[0] - final_readings[0],
        inital_readings[1] - final_readings[1],
        inital_readings[2] - final_readings[2],
    ])
    Robot_displacement = A_INV @ motor_displacements
    delta_x, delta_y, delta_theta = Robot_displacement[0], Robot_displacement[1], Robot_displacement[2]
    return delta_x, delta_y, delta_theta