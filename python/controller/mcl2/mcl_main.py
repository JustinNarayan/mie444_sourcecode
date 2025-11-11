import pygame
import math
import copy
import random
import mcl2.mcl_helper as mh
from mcl2.mcl_helper import Particle, GRID_WIDTH, GRID_HEIGHT, PPI
from lidar_reading import LidarReading

### Initialize pygame for visualization
pygame.init()

### CONSTANTS
WINDOW_WIDTH = GRID_WIDTH * PPI
WINDOW_HEIGHT = GRID_HEIGHT * PPI

# Colors
WHITE = (255, 255, 255)
RED   = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

# MCL parameters
FPS = 10                # display speed (frames per second)

### GLOBALS
grid = None
reduced_grid = None
valid_positions = None
pred_x = None
pred_y = None
pred_theta = None
window = None
step_count = None
clock = None
particles = None

### FUNCTIONS
def begin_localization():
    global grid, reduced_grid, valid_positions, pred_x, pred_y, pred_theta, window, step_count, clock, particles

    # Construct grid and generate particle starting positions within
    grid = mh.init_grid()
    reduced_grid = copy.deepcopy(grid)
    expanded_grid = mh.expand_grid(grid, PPI)
    valid_positions = mh.get_initial_valid_positions(expanded_grid)
    
    # Initialize random particles and initial position guess
    particles = [Particle(reduced_grid, valid_positions) for _ in range(mh.NUM_PARTICLES)]
    pred_x, pred_y, pred_theta = mh.estimate(particles)

    # Create display window
    window = pygame.display.set_mode((len(expanded_grid[0]), len(expanded_grid)))
    pygame.display.set_caption("Monte Carlo Localization")

	# Main loop step parameters
    step_count = 0
    clock = pygame.time.Clock()

    # Run initial update (e.g., with dummy or first lidar reading)
    step_localization()


def step_localization(
    delta_x=None, delta_y=None, delta_theta=None, lidar_reading: LidarReading = None
):
    """
    Performs one iteration of MCL:
        1. Motion update (apply movement deltas to all particles)
        2. Measurement update (compare lidar vs expected lidar)
        3. Resampling (select likely particles)
        4. Estimation and visualization
    """
    global grid, reduced_grid, valid_positions, pred_x, pred_y, pred_theta, window, step_count, clock, particles

    # Motion update: apply given delta commands to each particle
    if delta_x is not None and delta_y is not None and delta_theta is not None:
        for particle in particles:
            particle.move(delta_x, delta_y, delta_theta, reduced_grid)

    # Measurement update: if new lidar data is provided, update particle weights
    if lidar_reading is not None:
        for particle in particles:
            simulated_lidar = particle.lidar_scan()
            particle.update_weight(simulated_lidar, lidar_reading)

    # Resampling & Estimation
    particles, variance = mh.resample_particles(
        particles, reduced_grid, pred_x, pred_y
    )
    pred_x, pred_y, pred_theta = mh.estimate(particles)

    # Visualization
    window.fill(WHITE)
    mh.draw_grid(window, reduced_grid)

    # Draw particles to screen (RED)
    for particle in particles:
        mh.draw_state_estimate_to_screen(
            window, particle.x, particle.y, particle.theta, 
            is_final_predicted_state=False
        )

    # Draw estimated pose (green)
    mh.draw_state_estimate_to_screen(
		window, pred_x, pred_y, pred_theta, 
		is_final_predicted_state=True
	)

    pygame.display.flip()
    clock.tick(FPS)
    step_count += 1
    print(f"[MCL] Step {step_count}: Predicted position = ({pred_x:.1f}, {pred_y:.1f}), θ = {math.degrees(pred_theta):.1f}°")
