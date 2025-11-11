import pygame
import random
import math
import numpy as np
from filterpy.monte_carlo import stratified_resample
import pickle
from lidar_reading import LidarReading, LidarPointReading, DOWNSAMPLE_POINTS

# Constants
GRID_WIDTH = 96 + 2  # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2  # inches (one extra inch on each side for border)
ppi = 12
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
MAX_SENSOR_READING = 6000.0 / 127.0
MIN_SENSOR_READING = 5.0 / 2.54

# Simulation parameters
NUM_PARTICLES = 1 #2000
NUM_SCAN_ANGLES = 40

# Noise parameters
MOVEMENT_NOISE = 0.5
ANGULAR_NOISE = 0.125
MIN_SENSOR_STD = 0.5
MAX_SENSOR_STD = 8
certainty = 0

# Units
M_TO_INCH = 39.3701
LIDAR_ZERO_OFFSET = -210  # degrees


with open(
    "C:/Users/justi/GitHub/mie444_sourcecode/python/controller/mcl/sensor_data_12_ppi_15_beam_angle.pkl",
    "rb",
) as f:
    loaded_sensor_readings = pickle.load(f)


def init_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)

    # Set free movable space
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

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def expand_grid(grid, factor):
    """Expands each cell in the 2D grid into a larger cell of `factor x factor`.

    Args:
        grid (np.array): The original 2D numpy array (grid).
        factor (int): The size to expand each cell into (default is 12).

    Returns:
        np.array: A new 2D numpy array where each cell from the original grid is expanded.
    """
    # Use numpy's repeat function to expand rows and columns
    expanded_grid = np.repeat(np.repeat(np.array(grid), factor, axis=0), factor, axis=1)
    return expanded_grid


def display_grid(reduced_grid):
    for i in range(len(reduced_grid)):
        for j in range(len(reduced_grid[0])):
            if reduced_grid[i][j] == 0:
                print(reduced_grid[i][j], end=" ")
            elif reduced_grid[i][j] == 1:
                print("-", end=" ")
            else:
                print("*", end=" ")
        print()


def create_valid_positions(grid):
    valid_positions = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                valid_positions.append([j, i])
    return valid_positions


def draw_grid(window, grid):
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 1:
                color = BLACK
            elif grid[y][x] == 2:
                color = BLUE
            elif grid[y][x] != 0:
                color = GREEN
            else:
                color = WHITE
            pygame.draw.rect(window, color, (x * ppi, y * ppi, ppi, ppi))


def normal_pdf(x, mean, std_dev):
    """Calculate the PDF of a normal distribution at x given mean and std_dev."""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent


def particle_variance(particles, weights, pred_x, pred_y):
    variance_x = np.average(
        [(p.x - pred_x) ** 2 / ppi**2 for p in particles], weights=weights
    )
    variance_y = np.average(
        [(p.y - pred_y) ** 2 / ppi**2 for p in particles], weights=weights
    )

    # Average the variances to get a single spread measure
    return (variance_x + variance_y) / 2


def estimate(particles):
    particles_x = [p.x for p in particles]
    particles_y = [p.y for p in particles]
    particles_theta = [p.theta for p in particles]
    weights = [p.weight for p in particles]
    mean_x = np.average(particles_x, weights=weights)
    mean_y = np.average(particles_y, weights=weights)
    mean_theta = np.average(particles_theta, weights=weights)

    return mean_x, mean_y, mean_theta


def normalize_angle(angle_rad):
    return int(((angle_rad * 180 / math.pi) + LIDAR_ZERO_OFFSET) % 360)


def resample_particles(particles, grid, valid_positions, pred_x, pred_y):
    weights = [p.weight for p in particles]
    total_weight = sum(weights)
    # Normalize weights
    weights = [w / total_weight for w in weights]
    variance = particle_variance(particles, weights, pred_x, pred_y)
    certainty = ppi**2 * 2 / (variance)
    print(f"Certainty: {min(certainty, 1)}")
    adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 1000)

    # resampling algorithm
    indexes = stratified_resample(weights)

    # resample from index
    particles = [particles[i] for i in indexes[:adjusted_num_particles]]
    # print(len(particles))

    # Jitter the particles' positions and orientations to keep them close to their original pose
    def jitter_particle(particle):
        # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
        noise_scale = max(1 - certainty, 0)
        new_x = particle.x + np.random.normal(
            0, 0.5 + 2 * noise_scale
        )  # Jitter in x-axis
        new_y = particle.y + np.random.normal(
            0, 0.5 + 2 * noise_scale
        )  # Jitter in y-axis
        new_theta = particle.theta + np.random.normal(0, 0.1)  # Jitter in orientation

        # Create a new particle with a similar pose
        new_particle = Particle(grid, valid_positions)
        if (
            0 <= int(new_x) < len(grid[0])
            and 0 <= int(new_y) < len(grid)
            and grid[int(new_y)][int(new_x)] == 0
        ):
            new_particle.x = new_x
            new_particle.y = new_y
        else:
            new_particle.x = particle.x
            new_particle.y = particle.y
        new_particle.theta = new_theta
        new_particle.weight = 1.0 / len(particles)  # Reset weights to be equal
        return new_particle

    # Apply jitter to resampled particles and also add some random jitter for new exploration
    jittered_particles = [jitter_particle(p) for p in particles]

    return jittered_particles, variance


# Helper function to draw orientation lines
def draw_orientation(window, x, y, theta, length=10, color=BLACK):
    """Draw an orientation line based on the angle theta."""
    end_x = x + length * math.cos(theta)
    end_y = y + length * math.sin(theta)
    pygame.draw.line(window, color, (x, y), (end_x, end_y), 2)


# Particle class
class Particle:
    def __init__(self, grid, valid_positions):
        while True:
            pos = random.choice(valid_positions)
            self.x = pos[0]
            self.y = pos[1]
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        self.weight = 1.0 / NUM_PARTICLES

    def update_weight(
        self, this_lidar_reading: LidarReading, true_lidar_reading: LidarReading
    ):
        """
        Compare two LidarReading instances.
        Only compare points with exactly matching angles.
        """

        # Calculate how closely the lidar readings match expected points
        sensor_std = MIN_SENSOR_STD + (MAX_SENSOR_STD - MIN_SENSOR_STD) * max(
            1 - certainty, 0
        )

        this_points = this_lidar_reading.downsample().get_points()
        true_points = true_lidar_reading.downsample().get_points()

        self.weight = 1.0
        i = j = 0
        n_this = len(this_points)
        n_true = len(true_points)

        # similarity_measure = 0
        # num_assessed = 0

        while i < n_this and j < n_true:
            angle_this = this_points[i].angle
            angle_true = true_points[j].angle

            if angle_this == angle_true:
                # Matching angle → apply weighting function
                reading_this = this_points[i].distance
                reading_true = true_points[j].distance
                self.weight *= normal_pdf(reading_this, reading_true, sensor_std * ppi)
                # print(angle_this, reading_this, reading_true)
                # if (reading_this > 0 and reading_true > 0):
                #     smaller_reading = min(reading_this, reading_true)
                #     larger_reading = max(reading_this, reading_true)
                #     measure = (smaller_reading / larger_reading)**3
                #     similarity_measure += measure
                #     num_assessed += 1

                i += 1
                j += 1
            elif angle_this < angle_true:
                i += 1  # this point angle is behind, move forward
            else:
                j += 1  # true point angle is behind, move forward
        # if num_assessed > 0:
        # 	self.weight = (similarity_measure / num_assessed) + 1e-300
                
        self.weight += 1.0e-200

    def lidar_scan_tuned(self, num_points=NUM_SCAN_ANGLES):
        """Simulate a lidar scan with a tuned accuraracy"""
        scan_angles = np.linspace(
            360 / (2 * num_points), 360 - 360 / (2 * num_points), num_points
        )

        lidar_reading = LidarReading()

        for angle in scan_angles:
            angle_in_rad = angle * math.pi / 180
            scan_angle = normalize_angle(self.theta - angle_in_rad)
            simulated_reading = (
                loaded_sensor_readings[int(self.x)][int(self.y)][scan_angle] * M_TO_INCH
            )
            current_point = LidarPointReading(int(angle), int(simulated_reading), needs_mm_to_inch_conversion=False)
            lidar_reading.add_point(current_point)

        return lidar_reading

    def move(self, delta_x_command, delta_y_command, delta_theta_command, grid):
        """Move particle based on the same commanded delta movement (dx, dy, dtheta), with its own noise model."""

        # Add movement noise (Motion Model)
        dx_noise = random.gauss(0, MOVEMENT_NOISE)
        dy_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(
            0, math.radians(ANGULAR_NOISE)
        )  # Using the same noise parameter

        # The particle's movement is the commanded movement + noise
        adjusted_dx_command = delta_x_command * math.cos(
            self.theta
        ) + delta_y_command * math.sin(self.theta)
        adjusted_dy_command = delta_x_command * math.sin(
            self.theta
        ) + delta_y_command * -math.cos(self.theta)

        actual_dx = adjusted_dx_command + dx_noise
        actual_dy = adjusted_dy_command + dy_noise
        actual_dtheta = delta_theta_command + angular_noise  # sign convention

        # Calculate new position (Direct addition in world coordinates)
        new_x = self.x + actual_dx
        new_y = self.y + actual_dy
        new_theta = self.theta + actual_dtheta

        # Ensure particle stays within the drivable area
        if (
            0 <= int(new_x) < len(grid[0])
            and 0 <= int(new_y) < len(grid)
            and grid[int(new_y)][int(new_x)] == 0
        ):
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
