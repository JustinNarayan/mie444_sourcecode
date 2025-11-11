import pygame
import random
import math
import numpy as np
from filterpy.monte_carlo import stratified_resample
import pickle
from lidar_reading import LidarReading, LidarPointReading

### CONFIGURABLES

# Map size constants
GRID_WIDTH = 96 + 2   # inches
GRID_HEIGHT = 48 + 2  # 
PPI = 12  # pixels per inch (DO NOT CHANGE -> THIS IMPACTS SIMULATED LIDAR READINGS)

# Load precomputed sensor lookup table (indexed by [x_inches][y_inches][angle_deg])
with open(
    "C:/Users/justi/GitHub/mie444_sourcecode/python/controller/mcl/sensor_data_12_PPI_15_beam_angle.pkl", 
    "rb"
) as f:
    loaded_sensor_readings = pickle.load(f)


### TUNABLE PARAMETERS
NUM_PARTICLES = 2000            # number of particles
NUM_SCAN_ANGLES = 30           # number of beams used per particle (then downsampled)
MOVEMENT_NOISE_LINEAR = 0.25      # inches (std dev of translational motion noise)
MOVEMENT_NOISE_ANGULAR = 0.15     # radians (std dev of rotational motion noise)
SENSOR_STD_INCHES = 5.0          # std dev of measurement noise (in inches); used in Gaussian likelihood
MIN_WEIGHT = 1e-100               # floor weight to avoid zeroing out particles (tunable)
RESAMPLE_JITTER_POS = 0.25        # inches, positional jitter after resampling
RESAMPLE_JITTER_THETA = 0.15     # radians, angular jitter after resampling

# LIDAR reasonable bounds (inches)
LIDAR_RANGE_MIN = 3.0
LIDAR_RANGE_MAX = 80.0

# Debug / certainty variables (kept for compatibility)
certainty = 0.0

# Drawing
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE  = (0, 0, 255)
ORIENTATION_LINE_LENGTH_PARTICLE = 2
ORIENTATION_LINE_LENGTH_PREDICTED_STATE = 3
ORIENTATION_LINE_WIDTH = 1
CIRCLE_RADIUS_PARTICLE = PPI // 3
CIRCLE_RADIUS_PREDICTED_STATE = PPI // 2


### MAP HELPERS

def init_grid():
    """
    Initialize reduced grid (grid cell resolution is 1 cell per inch).
    Values:
      - 0 : free/drivable
      - 1 : obstacle
      - 2 : unmovable (e.g., border or outside)
    Units: each grid cell represents 1 inch (so indexing by integer coordinate = inches).
    NOTE: This function creates the *reduced* grid at 1 cell = 1 inch resolution.
    """
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)

    # Free spaces (0)
    grid[4:46, 4:10] = 0
    grid[4:22, 10:22] = 0
    grid[4:10, 22:40] = 0
    grid[28:40, 28:34] = 0
    grid[4:22, 40:46] = 0
    grid[16:22, 46:64] = 0
    grid[4:46, 64:70] = 0
    grid[40:46, 10:64] = 0
    grid[16:22, 64:88] = 0
    grid[4:46, 88:94] = 0

    # Borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1
    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Other maze obstacles (1)
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def expand_grid(grid, factor: int):
    """
    Expand the reduced grid (1 inch per cell) into a higher resolution grid where
    each cell expands into (factor x factor) pixels. This is the *expanded grid*
    used for drawing in pygame where each inch may map to multiple pixels.
    """
    return np.repeat(np.repeat(grid, factor, axis=0), factor, axis=1)


def get_initial_valid_positions(expanded_grid):
    """
    Get a list of all positions in the maze that are not obstacles.
    Particles will then be initially sampled from this set.
    """
    valid_positions = []
    for i in range(len(expanded_grid)):
        for j in range(len(expanded_grid[0])):
            if expanded_grid[i][j] == 0:
                valid_positions.append([j, i])
    return valid_positions

### DRAWING UTILITIES

def draw_state_estimate_to_screen(
    window, x_in_inches, y_in_inches, theta_in_rad, is_final_predicted_state=False
):
    # Select drawing parameters
    colour = GREEN if is_final_predicted_state else RED
    radius = CIRCLE_RADIUS_PREDICTED_STATE if is_final_predicted_state else CIRCLE_RADIUS_PARTICLE
    length = ORIENTATION_LINE_LENGTH_PREDICTED_STATE if is_final_predicted_state else ORIENTATION_LINE_LENGTH_PARTICLE 
    
    # Draw circle at position with line in orientation
    x_px, y_px = int(x_in_inches * PPI), int(y_in_inches * PPI)
    pygame.draw.circle(window, colour, (x_px, y_px), max(1, radius))
    line_end_x_px, line_end_y_px = (
		x_px + (length * math.cos(theta_in_rad) * PPI),
		y_px + (length * math.sin(theta_in_rad) * PPI)
	)
    pygame.draw.line(window, colour, (x_px, y_px), (line_end_x_px, line_end_y_px), ORIENTATION_LINE_WIDTH)

def draw_grid(window, grid_expanded):
    """
    Draw an expanded grid as pixels (grid_expanded is expected to be expanded via expand_grid).
    This function preserves the same call you'd used before (draw_grid(window, reduced_grid)),
    but now `reduced_grid` passed from main should be the expanded grid.
    """
    for y in range(len(grid_expanded)):
        for x in range(len(grid_expanded[0])):
            cell = grid_expanded[y][x]
            if cell == 1:
                color = BLACK
            elif cell == 2:
                color = BLUE
            elif cell != 0:
                color = GREEN
            else:
                color = WHITE
            pygame.draw.rect(window, color, (x * PPI, y * PPI, PPI, PPI))

### STATISTICS UTILITIES

def normal_pdf(x, mean, std_dev):
    """
    Gaussian PDF at x (units consistent with mean/std_dev).
    NOTE: For numerical stability in the weight update we use log-likelihoods instead of directly multiplying PDFs.
    """
    if std_dev <= 0:
        return 0.0
    coef = 1.0 / (std_dev * math.sqrt(2.0 * math.pi))
    exponent = math.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coef * exponent


def particle_variance(particles, weights, pred_x, pred_y):
    """
    Compute weighted variance of the particle positions (in inches^2).
    - particles: list of Particle objects (particle.x, particle.y are inches)
    - weights: normalized weights that sum to 1
    Returns a single scalar average variance (in inches^2).
    """
    xs = np.array([p.x for p in particles])
    ys = np.array([p.y for p in particles])
    weights = np.array(weights)
    var_x = np.average((xs - pred_x) ** 2, weights=weights)
    var_y = np.average((ys - pred_y) ** 2, weights=weights)
    return 0.5 * (var_x + var_y)

def get_normalized_weights(particles):
	# Normalize weights. Assign uniform if all weights zero
    weights = np.array([p.weight for p in particles], dtype=float)
    total_weight = float(np.sum(weights))
    if total_weight <= 0:
        weights = np.ones_like(weights) / len(weights) # uniform
    else:
        weights = weights / total_weight # normalized
    return weights

def estimate(particles):
    """
    Compute weighted mean pose (x inches, y inches, theta radians).
    Uses circular mean for theta.
    """
    weights = get_normalized_weights(particles)

	# Get raw state values of particles
    xs = np.array([p.x for p in particles])
    ys = np.array([p.y for p in particles])
    thetas = np.array([p.theta for p in particles])

	# Compute average of position
    mean_x = float(np.average(xs, weights=weights))
    mean_y = float(np.average(ys, weights=weights))

    # Use circular mean for angles
    sin_mean = float(np.average(np.sin(thetas), weights=weights))
    cos_mean = float(np.average(np.cos(thetas), weights=weights))
    mean_theta = math.atan2(sin_mean, cos_mean)

	# Return estiamted position
    return mean_x, mean_y, mean_theta


def normalize_angle_deg(angle_deg: float):
    """
    Convert a radian angle to degrees with LIDAR zero offset applied.
    Input: angle in radians.
    Return: integer degree in [0, 359] used for lookup indexing.
    Note: This function is used when indexing the sensor lookup table.
    """
    return int(
        (
            (angle_deg * 180.0 / math.pi)
        ) % 360
    )
    
def is_valid_point_in_grid(x,y,grid):
    return (
        (0 <= x < len(grid[0])) and 
        (0 <= y < len(grid)) and
        (grid[y][x] == 0)
	)

### RESAMPLING

def resample_particles(particles, reduced_grid, pred_x=None, pred_y=None):
    """
    Resample particles using stratified resampling, then add jitter to each resampled particle.
    - particles: list of Particle objects (positions in inches)
    - reduced_grid: reduced grid (inches resolution) used to validate jittered positions
    Returns: (new_particles_list, variance)
    """
    weights = get_normalized_weights(particles)

    # Compute variance, provided valid pred_x, pred_y
    if pred_x is not None and pred_y is not None:
        variance = particle_variance(particles, weights, pred_x, pred_y)
    else:
        # fallback: unweighted variance
        xs = np.array([p.x for p in particles])
        ys = np.array([p.y for p in particles])
        variance = 0.5 * (np.var(xs) + np.var(ys))
        
    # Compute certainty
    certainty = PPI**2 * 2 / (variance)
    print(f"Certainty: {min(certainty, 1)}")
    # adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 1000)

    # Get inidices of a stratified resample using weights
    indexes = stratified_resample(weights)  # returns an array of indexes length of weights

    # Clone the resampled particles
    new_particles = []
    for i in indexes:
        src = particles[int(i)]
        p = Particle.__new__(Particle)  # create empty object then set fields (bypass __init__, which chooses randomly)
        # copy essential fields
        p.x = float(src.x)
        p.y = float(src.y)
        p.theta = float(src.theta)
        p.weight = 1.0 / len(particles)  # reset weights to uniform for next update
        new_particles.append(p)

    # Jitter particles in units of inches and radians
    for p in new_particles:
        # Choose jitter from Gaussians
        jitter_x = random.gauss(0, RESAMPLE_JITTER_POS) # inches
        jitter_y = random.gauss(0, RESAMPLE_JITTER_POS) # inches
        jitter_theta = random.gauss(0, RESAMPLE_JITTER_THETA) # radians

		# Apply jitter
        new_x = p.x + jitter_x
        new_y = p.y + jitter_y
        new_theta = (p.theta + jitter_theta) % (2 * math.pi)

        # Keep particle in valid region; if jitter knocks particle out, do not update position
        x_i = int(round(new_x))
        y_i = int(round(new_y))
        if is_valid_point_in_grid(x_i, y_i, reduced_grid):
            p.x = new_x
            p.y = new_y

		# Update orientation regardless
        p.theta = new_theta
        
        # Note that particles have already been reweighted

    return new_particles, variance

### PARTICLE
class Particle:
    """
    Particle X,Y in **inches**, Theta in **radians**.
    NOTE: Initialization picks an inch-grid valid cell.
    """

    def __init__(self, reduced_grid, valid_positions_expanded_grid):
        """
        reduced_grid: reduced grid at 1 cell = 1 inch resolution (as returned by init_grid()).
        valid_positions_in_pixels: list of (x_in_pixels, y_in_pixels) tuples that are free.
        """
        # Pick a random valid position for each particle within the maze
        while True:
            pos_in_pixels = random.choice(valid_positions_expanded_grid)
            x_in_inches, y_in_inches = int(pos_in_pixels[0] / PPI), int(pos_in_pixels[1] / PPI)
            
            if is_valid_point_in_grid(x_in_inches, y_in_inches, reduced_grid):
                self.x = float(x_in_inches)
                self.y = float(y_in_inches)
                break
            
		# Uniform weighting and random orientation
        self.theta = random.uniform(0.0, 2.0 * math.pi)
        self.weight = 1.0 / NUM_PARTICLES
        

    # MOVEMENT
    def move(
        self, delta_x_inches: float, delta_y_inches: float, delta_theta_rad: float, reduced_grid
    ):
        """
        Move the particle according to commanded deltas (units: inches, inches, radians).
        """
        # Add Gaussian motion noise (inches / radians)
        dx_noisy = delta_x_inches + random.gauss(0.0, MOVEMENT_NOISE_LINEAR) # inches
        dy_noisy = delta_y_inches + random.gauss(0.0, MOVEMENT_NOISE_LINEAR) # inches
        dtheta_noisy = delta_theta_rad + random.gauss(0.0, MOVEMENT_NOISE_ANGULAR) # radians

        # Transform robot-frame x/y into world-frame x/y
        world_dx = dx_noisy * math.cos(self.theta) - dy_noisy * math.sin(self.theta)
        world_dy = dx_noisy * math.sin(self.theta) + dy_noisy * math.cos(self.theta)

		# Compute particle position with robot control plus noise
        new_x = self.x + world_dx
        new_y = self.y + world_dy
        new_theta = (self.theta + dtheta_noisy) % (2.0 * math.pi)

        # Keep particle in valid region; if control knocks particle out, do not update position
        nx_i = int(round(new_x))
        ny_i = int(round(new_y))
        if 0 <= nx_i < GRID_WIDTH and 0 <= ny_i < GRID_HEIGHT and reduced_grid[ny_i][nx_i] == 0:
            self.x = new_x
            self.y = new_y
            
		# Update orientation regardless
        self.theta = new_theta

    # LIDAR_SCAN
    def lidar_scan(self, num_points: int = NUM_SCAN_ANGLES):
        """
        Create a LidarReading simulated by sampling the precomputed lookup table.
        - num_points: number of beam angles to simulate (we output angles in degrees 0..360)
        - The lookup table (loaded_sensor_readings) is indexed by [x_in_inches][y_in_inches][angle_deg]
          and returns a distance measure (in meters, must be converted to inches)
        RETURNS:
          LidarReading where angles are beam angles (deg) and distances are inches.
        """
        # Space angle readings uniformly
        beam_angles_deg = np.linspace(0.0, 360.0, num_points, endpoint=False)

		# Initialize lidar reading
        reading = LidarReading()
        x_in_pixels = int(round(self.x)) * PPI
        y_in_pixels = int(round(self.y)) * PPI
        # Compute LIDAR scan
        for angle_deg in beam_angles_deg:
            # The angle of this beam is the particle orientation + the beam's relative angle
            world_beam_angle_rad = (
                + (self.theta + math.radians(angle_deg)) ### IMPORTANT -> keep this sign convention
                % (2.0 * math.pi)
			)
            lookup_deg = int(normalize_angle_deg(world_beam_angle_rad))

            # Fetch from lookup table
            simulated_reading_inches = loaded_sensor_readings[x_in_pixels][y_in_pixels][lookup_deg]

            # Constrain reading
            simulated_reading_inches = max(
                LIDAR_RANGE_MIN, 
                min(LIDAR_RANGE_MAX, simulated_reading_inches)
            )

            # Add point at the beam angle (we store beam's angle in degrees; distance in inches)
            point = LidarPointReading(
                angle_deg, 
                simulated_reading_inches, 
                needs_mm_to_inch_conversion=False
            )
            reading.add_point(point, is_real_lidar_data=False)

        return reading

    # REWEIGHT
    def update_weight(
        self, simulated_reading: LidarReading, real_reading: LidarReading, min_weight: float = MIN_WEIGHT
    ):
        """
        Update particle's weight based on similarity between `simulated_reading` (predicted)
        and `observed_reading` (actual). DO NOT mutate input readings.

        Behavior:
          - Downsample both readings using LidarReading.downsample() (same num_points)
          - Compare only buckets (angles) that are present in both downsampled readings.
          - Use a Gaussian measurement model per beam:
                p(z_obs | z_pred) = Normal(z_obs; mean=z_pred, std = sensor_std)
            (units: inches)
          - Compute total log-likelihood over all compared beams:
                log P = sum_i log p_i
            Then set weight = exp(log P)
          - If there are zero overlapping beams, we apply `min_weight` (do not set to zero).
          - We clamp final weight to be at least min_weight.
        Numerical stability:
          - We compute in log-space to avoid underflow when multiplying many tiny probabilities.
        """
        
        # Downsample simulated and real LIDAR readings for speed
        simulated_lidar_downsampled = simulated_reading.get_downsampled()
        real_lidar_downsampled = real_reading.get_downsampled()
        
        # Normalize readings to account for differences in modeled space
        simulated_lidar_normalized = simulated_lidar_downsampled.get_normalized()
        real_lidar_normalized = real_lidar_downsampled.get_normalized()
        
		# Extract downsample Lidar Point Readings
        simulated_lidar_points = simulated_lidar_normalized.get_points()
        real_lidar_points = real_lidar_normalized.get_points()

        # Construct dictionaries of points to identify angles contained in both sets
        # The real LIDAR reading may not always compute all angles
        simulated_lidar_map = {
            int(round(p.angle)) % 360: p.distance for p in simulated_lidar_points
        }
        real_lidar_map = {
            int(round(p.angle)) % 360: p.distance for p in real_lidar_points
		}
        angles_to_compare = sorted(
            set(simulated_lidar_map.keys()).intersection(set(real_lidar_map.keys()))
        )

		# If no angles are shared, weight particle as little as possible
        if len(angles_to_compare) == 0:
            self.weight = float(min_weight)
            return

		# Use log-likelihood to avoid underflows
        # Sum the log of the probability distribution functions
        log_likelihood = 0.0
        
        # Precompute gaussian coefficient term for speed: log(1/(sigma*sqrt(2pi)))
        sensor_std = SENSOR_STD_INCHES / LIDAR_RANGE_MAX # normalized
        gaussian_log_prefactor = -0.5 * math.log(2.0 * math.pi * (sensor_std ** 2))

		# Iterate through angles and get log-likelihoods
        for ang in angles_to_compare:
            z_simulated = simulated_lidar_map[ang]   # predicted distance (inches)
            z_real = real_lidar_map[ang]    # observed distance (inches)

            # Compute log PDF of normal distribution
            diff = z_simulated - z_real
            log_pdf = gaussian_log_prefactor - 0.5 * ((diff ** 2) / (sensor_std ** 2))
            log_likelihood += log_pdf

		# Convert back into weight space and clamp the minimum weight
        try:
            weight = math.exp(log_likelihood)
        except OverflowError:
            # If log_likelihood is too small/large, clamp to min_weight
            weight = float(min_weight)

        # Again, enforce floor to avoid degenerate zero
        self.weight = max(weight, float(min_weight))
