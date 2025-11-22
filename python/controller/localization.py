# Placeholder for localization logic
from encoder_reading import EncoderReading
from lidar_reading import LidarReading
from mcl2.mcl_main import step_localization, is_localized
import math
import numpy as np

# Global variables
current_encoder_reading: EncoderReading = EncoderReading()
last_sent_encoder_reading: EncoderReading | None = None
post_lidar_encoder_reading: EncoderReading | None = None

# Robot parameters
L = 3.569 # inches- DISTANCE_FROM_OBJECT_CENTER_TO_WHEEL_MIDPOINT
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

# Drivetrain commands
DRIVETRAIN_COMMAND_MAX_DX = 6 # inches
DRIVETRAIN_COMMAND_MAX_DY = 6 # inches
DRIVETRAIN_COMMAND_MAX_DTHETA = 90 # degrees
DRIVETRAIN_COMMAND_ASSUMED_RADIUS = 5 # inches
PREFERRED_DIRECTION = 0.0   # forward along +X

MIN_LIDAR_DENSITY = 0.25     # fraction of points required
DENSITY_CHECK_ANGLE = 60.0  # degrees on either side of candidate direction

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
        delta_x, delta_y, delta_theta = inverse_kinematics(
            post_lidar_encoder_reading,
            last_sent_encoder_reading
        )
        
        # Step localization
        step_localization(delta_x, delta_y, delta_theta, lidar_reading)

    # Update last sent reading
    last_sent_encoder_reading = post_lidar_encoder_reading.copy()

    # Clear post-lidar reading
    post_lidar_encoder_reading = None

def inverse_kinematics(initial: EncoderReading, current: EncoderReading):
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

    inital_readings = initial.get_readings()
    final_readings = current.get_readings()
    motor_displacements = np.array([
        inital_readings[0] - final_readings[0],
        inital_readings[1] - final_readings[1],
        inital_readings[2] - final_readings[2],
    ])
    Robot_displacement = A_INV @ motor_displacements
    # Return delta_x, delta_y, delta_theta
    delta_x, delta_y, delta_theta = Robot_displacement[0], Robot_displacement[1], Robot_displacement[2]
    
    return delta_x, delta_y, delta_theta

def get_free_direction_drivetrain_command(lidar_reading: LidarReading):
    """
    Given a dense LidarReading object, returns (dX, dY, dTheta) for the best free
    direction to move, constrained by drivetrain limits and minimum lidar density.
    """
    points = lidar_reading.get_points()
    if len(points) == 0:
        assert False, "No lidar!"
        return 0.0, 0.0, 0.0

    # --- Create polar histogram for density ---
    angles = np.array([p.angle % 360 for p in points])
    distances = np.array([p.distance for p in points])
    
    # Candidate directions to evaluate (0..359 degrees)
    candidate_angles = np.arange(0, 360, 1)
    best_score = -1
    best_direction = 0.0
    
    for candidate in candidate_angles:
        # Check # of points within +/- DENSITY_CHECK_ANGLE of candidate
        lower = (candidate - DENSITY_CHECK_ANGLE) % 360
        upper = (candidate + DENSITY_CHECK_ANGLE) % 360
        if lower < upper:
            mask = (angles >= lower) & (angles <= upper)
        else:  # wrap-around
            mask = (angles >= lower) | (angles <= upper)

        num_density_check_points = 2*DENSITY_CHECK_ANGLE # since 360 degrees
        density = np.sum(mask) / num_density_check_points

        # Compute free space: average distance in this sector
        # if density < MIN_LIDAR_DENSITY:
        #     continue

        avg_distance = np.mean(distances[mask])
        # Prefer directions closer to forward
        angle_diff = min(abs((candidate - PREFERRED_DIRECTION) % 360),
                         abs((PREFERRED_DIRECTION - candidate) % 360))
        score = avg_distance * density - 0.1 * angle_diff  # penalize turning

        if score > best_score:
            best_score = score
            best_direction = candidate
            
    if best_score <= 0:
        assert False, "No free direction found"
        # No free direction found
        return 0.0, 0.0, 0.0

    # --- Convert best direction to dX, dY ---
    # Move as far as possible without exceeding max
    move_distance = min(best_score - DRIVETRAIN_COMMAND_ASSUMED_RADIUS,
                        DRIVETRAIN_COMMAND_MAX_DX,
                        DRIVETRAIN_COMMAND_MAX_DY)
    move_distance = max(move_distance, 0.0)

    # Convert polar to dX, dY
    theta_rad = math.radians(best_direction)
    dX = move_distance * math.cos(theta_rad)
    dY = move_distance * math.sin(theta_rad)

    # Simple rotation heuristic: face the direction we want to move
    dTheta = best_direction
    # Limit rotation to max allowed
    if dTheta > DRIVETRAIN_COMMAND_MAX_DTHETA:
        dTheta = DRIVETRAIN_COMMAND_MAX_DTHETA
    if dTheta < -DRIVETRAIN_COMMAND_MAX_DTHETA:
        dTheta = -DRIVETRAIN_COMMAND_MAX_DTHETA
    assert False, f"{dX}, {dY}, {dTheta}"
    return dX, dY, dTheta

def get_drivetrain_command(lidar_reading: LidarReading):
    # if not is_localized():
    return get_free_direction_drivetrain_command(lidar_reading)