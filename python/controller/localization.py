from encoder_reading import EncoderReading
from lidar_reading import LidarReading
from mcl2.mcl_main import step_localization, is_localized
import math
import numpy as np

# Global variables
current_encoder_reading: EncoderReading = EncoderReading()
last_sent_encoder_reading: EncoderReading | None = None
post_lidar_encoder_reading: EncoderReading | None = None
last_best_direction = None

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

# Drivetrain command constraints (tweakable)
DRIVETRAIN_COMMAND_MAX_DX = 6  # inches (max forward when allowed)
DRIVETRAIN_COMMAND_ASSUMED_RADIUS = 5.5  # inches (robot radius approximation)

# Angle/rotation thresholds
TARGET_DTHETA_TO_ZERO = 5  # degrees: if misaligned by more than this -> rotate-only
PREFERRED_DIRECTION = 0     # forward = 0 degrees

# Rotation limits requested
ROTATE_ONLY_MAX_DTHETA = 30       # degrees (rotate-only mode, max)
ROTATE_AND_MOVE_MAX_DTHETA = 5    # degrees (max rotation when also moving)

# Lidar / scoring constants
MIN_LIDAR_DENSITY_FORWARD = 0.2   # minimum density inside front +/- FRONT_HALF_ANGLE
FRONT_HALF_ANGLE = 10.0           # degrees
TOTAL_HALF_ANGLE = 30.0           # degrees (total checked = +/-30 => 60 deg window)
# peripheral region is the ±30 band excluding the central ±15 band
MIN_LIDAR_DENSITY_PERIPHERAL = 0.3

MIN_CLEARANCE_FROM_WALL = 0.65     # inches minimum desired clearance (very conservative default)
MIN_EDGE_CLEARANCE = 1.5          # inches - minimum forward clearance for robot edges to consider moving

MOVE_PREF_FORWARD = 1.0    # prefer +dX
MOVE_PREF_BACKWARD = 0.5   # prefer -dX
MOVE_PREF_STRAFE = 0.25     # prefer lateral movement (dY)

# scoring weights (higher -> more important)
W_FORWARD_DISTANCE = 1.0
W_FORWARD_DENSITY = 2.0
W_PERIPHERAL_DENSITY = 0.8
W_SQUARITY = 2.0      # prefer being square to walls (smaller difference left/right)
W_EDGE_CLEARANCE = 3.0  # strongly penalize small edge clearance

# small numeric constants
EPS = 1e-6


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


def _compute_cartesian(points):
    """
    Convert lidar points (angles in degrees, distances in inches) to numpy arrays x,y.
    x is forward, y is left (robot frame).
    Returns arrays (x, y, angles_deg, distances).
    """
    angles = np.array([p.angle % 360 for p in points])
    distances = np.array([p.distance for p in points])
    angles_rad = np.deg2rad(angles)
    x = distances * np.cos(angles_rad)  # forward component
    y = distances * np.sin(angles_rad)  # left component
    return x, y, angles, distances


def _sector_mask(angles, center_deg, half_width_deg):
    """
    Boolean mask for points whose angle lies within center_deg +/- half_width_deg, with wrap-around.
    Angles in degrees (0..360).
    """
    lower = (center_deg - half_width_deg) % 360
    upper = (center_deg + half_width_deg) % 360
    if lower < upper:
        return (angles >= lower) & (angles <= upper)
    else:
        # wrap
        return (angles >= lower) | (angles <= upper)


def get_free_direction_drivetrain_command(lidar_reading: LidarReading):
    """
    Overhauled logic:
      - For each candidate direction (1° step), compute:
         * forward density in +/- FRONT_HALF_ANGLE
         * peripheral density in the outer +/- TOTAL_HALF_ANGLE excluding front
         * forward distance projected for three "edges": center, left edge, right edge
         * left/right wall clearance and squareness measure
      - Require minimum forward & peripheral densities.
      - Disqualify candidate if any edge-forward-distance < MIN_EDGE_CLEARANCE.
      - Score candidates using weighted sum preferring larger forward distance,
        higher densities, larger edge clearance, and smaller left/right imbalance.
      - After picking best_direction, decide rotate-only vs rotate+move per thresholds and rotation limits.
    Returns (dX, dY, dTheta) as integers.
    """
    global last_best_direction

    points = lidar_reading.get_points()
    if len(points) == 0:
        return 0, 0, 0

    # Precompute cartesian coords
    x_pts, y_pts, angles_deg, distances = _compute_cartesian(points)

    candidate_angles = np.arange(0, 360, 1)
    best_score = -float("inf")
    best_direction = None

    # robot geometry used for lateral offset checks
    robot_radius = DRIVETRAIN_COMMAND_ASSUMED_RADIUS  # inches

    # Precompute left/right wall summary (use +/- 20 deg sectors around 90 and 270)
    left_sector_mask = _sector_mask(angles_deg, 90, 20)
    right_sector_mask = _sector_mask(angles_deg, 270, 20)
    left_wall_dist = np.mean(distances[left_sector_mask]) if np.any(left_sector_mask) else float('inf')
    right_wall_dist = np.mean(distances[right_sector_mask]) if np.any(right_sector_mask) else float('inf')
    squareness = abs(left_wall_dist - right_wall_dist)  # smaller -> more square

    # For each candidate, compute metrics
    for candidate in candidate_angles:
        # Masks for forward and peripheral
        mask_total = _sector_mask(angles_deg, candidate, TOTAL_HALF_ANGLE)
        mask_front = _sector_mask(angles_deg, candidate, FRONT_HALF_ANGLE)

        # Peripheral is total minus front
        mask_peripheral = mask_total & (~mask_front)

        # Densities normalized by angular width (degrees)
        front_density = np.sum(mask_front) / (2 * FRONT_HALF_ANGLE + EPS)
        peripheral_density = np.sum(mask_peripheral) / (2 * (TOTAL_HALF_ANGLE - FRONT_HALF_ANGLE) + EPS)

        # Require minimum densities
        if front_density < MIN_LIDAR_DENSITY_FORWARD or peripheral_density < MIN_LIDAR_DENSITY_PERIPHERAL:
            continue

        # Forward distances (projected) for points in the total sector:
        sector_indices = np.nonzero(mask_total)[0]
        if sector_indices.size == 0:
            continue

        # Candidate unit vectors (forward and perpendicular)
        cand_rad = math.radians(candidate)
        ufx = math.cos(cand_rad)   # forward x component
        ufy = math.sin(cand_rad)   # forward y component (left positive)
        # perp vector to the left of forward (positive points left)
        upx = -ufy
        upy = ufx
        
        # movement preference term
        movement_bias = 0.0
        if ufx > 0:
            movement_bias += MOVE_PREF_FORWARD * ufx
        else:
            movement_bias += MOVE_PREF_BACKWARD * -ufx
        movement_bias += MOVE_PREF_STRAFE * abs(ufy)

        # For each lidar point compute forward projection (dot with u) and lateral (dot with perp)
        px = x_pts[sector_indices]
        py = y_pts[sector_indices]
        frwd = px * ufx + py * ufy
        lat = px * upx + py * upy

        # Consider only points with positive forward projection (in front)
        forward_mask = frwd > 0
        if not np.any(forward_mask):
            continue

        frwd = frwd[forward_mask]
        lat = lat[forward_mask]

        # We want minimum forward distance that would intersect the robot edges if the robot travels along candidate.
        # Compute for three lateral offsets corresponding to center (0), left (+robot_radius), right (-robot_radius).
        # For an obstacle point to block an edge, its lateral coordinate must be approximately equal to that offset
        # within half the robot width tolerance. Use a small tolerance equal to robot_radius * 0.6.
        lat_tol = max(0.5, robot_radius * 0.6)

        def min_forward_for_lateral_offset(offset):
            # select points where |lat - offset| <= lat_tol
            sel = np.abs(lat - offset) <= lat_tol
            if not np.any(sel):
                return float('inf')
            return float(np.min(frwd[sel]))

        min_center = min_forward_for_lateral_offset(0.0)
        min_left_edge = min_forward_for_lateral_offset(robot_radius)
        min_right_edge = min_forward_for_lateral_offset(-robot_radius)

        # The limiting clearance along this candidate is the minimum of the three edges
        limiting_edge_clearance = min(min_center, min_left_edge, min_right_edge)

        # If the limiting clearance is too small, disqualify (robot would hit)
        if limiting_edge_clearance < MIN_EDGE_CLEARANCE:
            continue

        # Wall proximity penalty:
        # Predict lateral shift: if candidate has a lateral component positive -> movement tends to move left side closer to left wall.
        # Use rough test: if moving forward by some nominal move (e.g., min(limiting_edge_clearance, DRIVETRAIN_COMMAND_MAX_DX))
        nominal_move = min(limiting_edge_clearance - DRIVETRAIN_COMMAND_ASSUMED_RADIUS, DRIVETRAIN_COMMAND_MAX_DX)
        nominal_move = max(0.0, nominal_move)
        # lateral displacement after move (left positive)
        lateral_disp = nominal_move * ufy

        # compute predicted left and right clearances after moving
        predicted_left_clearance = left_wall_dist - max(0.0, lateral_disp)
        predicted_right_clearance = right_wall_dist - max(0.0, -lateral_disp)

        # If either predicted clearance is below MIN_CLEARANCE_FROM_WALL, de-prioritize / skip
        if predicted_left_clearance < MIN_CLEARANCE_FROM_WALL or predicted_right_clearance < MIN_CLEARANCE_FROM_WALL:
            # strongly penalize candidate (skip)
            continue

        # Now compute a combined score
        # prefer larger limiting_edge_clearance and forward distance and densities, and prefer being square (small squareness)
        score = 0.0
        score += W_FORWARD_DISTANCE * limiting_edge_clearance
        score += W_FORWARD_DENSITY * front_density
        score += W_PERIPHERAL_DENSITY * peripheral_density
        # reward squareness (smaller difference -> higher score); invert squareness
        score += W_SQUARITY * (1.0 / (1.0 + squareness))
        score += W_EDGE_CLEARANCE * (limiting_edge_clearance / (1.0 + limiting_edge_clearance))
        score += movement_bias  # add to total score

        # small momentum term: prefer last_best_direction
        if last_best_direction is not None:
            score -= 0.02 * abs(candidate - last_best_direction)

        if score > best_score:
            best_score = score
            best_direction = candidate
            best_limiting_edge_clearance = limiting_edge_clearance
            best_front_density = front_density
            best_peripheral_density = peripheral_density

    # If no candidate survived
    if best_direction is None:
        return 0, 0, 0

    # Save last best
    last_best_direction = best_direction

    # Decide rotation vs move
    diff = ((best_direction - PREFERRED_DIRECTION + 180) % 360) - 180
    abs_diff = abs(diff)

    # If too misaligned -> rotate-only (limit ROTATE_ONLY_MAX_DTHETA)
    if abs_diff > TARGET_DTHETA_TO_ZERO:
        rotate_amount = max(-ROTATE_ONLY_MAX_DTHETA, min(ROTATE_ONLY_MAX_DTHETA, diff))
        return 0, 0, int(round(rotate_amount))

    # Otherwise allow small rotation (ROTATE_AND_MOVE_MAX_DTHETA) AND forward move (DRIVETRAIN_COMMAND_MAX_DX)
    dTheta = int(round(max(-ROTATE_AND_MOVE_MAX_DTHETA, min(ROTATE_AND_MOVE_MAX_DTHETA, diff))))

    # Move distance is limited by worst-case edge clearance minus robot radius, and by max DX.
    max_safe_move = best_limiting_edge_clearance - DRIVETRAIN_COMMAND_ASSUMED_RADIUS
    move_distance = max(0.0, min(max_safe_move, DRIVETRAIN_COMMAND_MAX_DX))

    # As a final safety check: if moving would asymmetrically reduce clearance to one wall below MIN_CLEARANCE_FROM_WALL, prefer a small corrective rotation instead of moving
    angle_rad = math.radians(best_direction)
    ufy = math.sin(angle_rad)
    lateral_disp = move_distance * ufy
    predicted_left_clearance = left_wall_dist - max(0.0, lateral_disp)
    predicted_right_clearance = right_wall_dist - max(0.0, -lateral_disp)
    if predicted_left_clearance < MIN_CLEARANCE_FROM_WALL:
        # rotate right a bit
        return 0, 0, -min(ROTATE_AND_MOVE_MAX_DTHETA, TARGET_DTHETA_TO_ZERO)
    if predicted_right_clearance < MIN_CLEARANCE_FROM_WALL:
        # rotate left a bit
        return 0, 0, min(ROTATE_AND_MOVE_MAX_DTHETA, TARGET_DTHETA_TO_ZERO)

    dX = int(round(move_distance * ufx))
    dY = int(round(move_distance * ufy))

    return dX, dY, dTheta


def get_drivetrain_command(lidar_reading: LidarReading):
    # if not is_localized():
    return get_free_direction_drivetrain_command(lidar_reading)
