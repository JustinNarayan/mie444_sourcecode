# Placeholder for localization logic
from encoder_reading import EncoderReading
from lidar_reading import LidarReading
from mcl2.mcl_main import step_localization
import math
import numpy as np

# Global variables
current_encoder_reading: EncoderReading = EncoderReading()
last_sent_encoder_reading: EncoderReading | None = None
post_lidar_encoder_reading: EncoderReading | None = None

# Robot parameters
L = 3.73771654 # inches, 94.938 mm - DISTANCE_FROM_OBJECT_CENTER_TO_WHEEL_MIDPOINT
# Thetas, defined as angle from robot x-axis to motor allignment
# X-axis is defined as the forward direction of the robot (bisecting two of the drive wheels perpincular to third drive wheel)

DEG_TO_RAD = math.pi/180
THETA_1 = 180 * DEG_TO_RAD # radians
THETA_2 = 60 * DEG_TO_RAD # radians
THETA_3 = 300 * DEG_TO_RAD # radians

A = np.array([
    [-math.sin(THETA_1), math.cos(THETA_1), L],
    [-math.sin(THETA_2), math.cos(THETA_2), L],
    [-math.sin(THETA_3), math.cos(THETA_3), L]
])

# Inverse of A
A_INV = np.linalg.inv(A)

def prepare_info_for_localization_step(lidar_reading: LidarReading):
    """
    Called whenever a LidarComplete message is received.
    Updates post_lidar_encoder_reading and calls get_delta_position_orientation.
    """
    global post_lidar_encoder_reading, last_sent_encoder_reading, current_encoder_reading

    # Set post-lidar encoder reading
    post_lidar_encoder_reading = current_encoder_reading.copy()

    # Compute delta using last sent encoder
    if last_sent_encoder_reading is not None:
        delta_x, delta_y, delta_theta = get_delta_position_orientation(
            post_lidar_encoder_reading,
            last_sent_encoder_reading
        )
        
        # Step localization
        step_localization(delta_x, delta_y, delta_theta, lidar_reading)

    # Update last sent reading
    last_sent_encoder_reading = post_lidar_encoder_reading.copy()

    # Clear post-lidar reading
    post_lidar_encoder_reading = None

def get_delta_position_orientation(post_lidar: EncoderReading, last_sent: EncoderReading):
    """
    Placeholder function: compute delta_x, delta_y, delta_theta
    between two encoder readings.
    """
    
    # Displacments are 3x1 vector for each motor
    # Robot_displacment = Inverse_Kinematics_Matrix * Motor_displacment
    # where Inverse_Kinematics_Matrix is: Calculated from robot geometry
    # L = DISTANCE_FROM_OBJECT_CENTER_TO_WHEEL_MIDPOINT
    # A = [ [-sin(THETA_1), cos(THETA_1), L],
    #         [-sin(THETA_2), cos(THETA_2), L],
    #         [-sin(THETA_3), cos(THETA_3), L] ]

    inital_readings = post_lidar.get_readings()
    final_readings = last_sent.get_readings()
    motor_displacements = np.array([
        inital_readings[0] - final_readings[0],
        inital_readings[1] - final_readings[1],
        inital_readings[2] - final_readings[2],
    ])
    Robot_displacement = A_INV @ motor_displacements
    # Return delta_x, delta_y, delta_theta
    delta_x, delta_y, delta_theta = Robot_displacement[0], Robot_displacement[1], Robot_displacement[2]
    
    return delta_x, delta_y, delta_theta