import pygame
import math
import copy
import random
import mcl.mcl_helper_OG as mh
from mcl.mcl_helper_OG import Particle
from lidar_reading import LidarReading

# Initialize pygame
pygame.init()

# --- Constants ---
# (Keep constants from mcl_OG.py)
GRID_WIDTH = 96 + 2  # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2  # inches (one extra inch on each side for border)
ppi = 12
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

NUM_PARTICLES = mh.NUM_PARTICLES
# Note: Original FORWARD_VELOCITY and ANGULAR_VELOCITY are no longer directly used for movement
SIMULATION_STEPS = (
    200  # Number of steps to run the controlled simulation (now unused for loop limit)
)
STEP_DURATION = 0.5  # Duration of each step in seconds (not used for delta commands)
# -----------------

# Globals
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

def begin_localization():
    global grid, reduced_grid, valid_positions, pred_x, pred_y, pred_theta, window, step_count, clock, particles

    grid = mh.init_grid()
    reduced_grid = copy.deepcopy(grid)
    grid = mh.expand_grid(grid, ppi)
    valid_positions = mh.create_valid_positions(grid)

    # Initialize particles randomly across the map
    particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
    pred_x, pred_y, pred_theta = mh.estimate(particles)

    # Initialize window
    window = pygame.display.set_mode((len(grid[0]), len(grid)))
    pygame.display.set_caption("MCL")

    # Main loop for controlled steps
    step_count = 0
    clock =  pygame.time.Clock()
    
    # First step with dummy data
    step_localization()


def step_localization(
    delta_x: float = None, delta_y: float = None, delta_theta: float = None, lidar_reading: LidarReading = None
):
    global grid, reduced_grid, valid_positions, pred_x, pred_y, pred_theta, window, step_count, clock, particles

    # Particle movement must be done manually for delta commands,
    # as the old mh.update_particles used velocities.
    if (delta_x != None and delta_y != None and delta_theta != None):
        for particle in particles:
			# The Particle class's delta-based move method is named 'move'
			# and accepts three delta arguments
            particle.move(delta_x, delta_y, delta_theta, grid)

    if lidar_reading != None:
        # Update Weights
        for particle in particles:
            particle_lidar_reading = particle.lidar_scan_tuned()
            particle.update_weight(particle_lidar_reading, lidar_reading)
            

    # Resample and Estimate
    particles, variance = mh.resample_particles(
        particles, grid, valid_positions, pred_x, pred_y
    )
    pred_x, pred_y, pred_theta = mh.estimate(particles)

    # 6. Drawing and Display (No changes here)
    window.fill(WHITE)
    mh.draw_grid(window, reduced_grid)

    # Draw particles
    for particle in particles:
        particle_pos = (int(particle.x), int(particle.y))
        pygame.draw.circle(window, RED, particle_pos, ppi / 2)
        mh.draw_orientation(
            window,
            particle_pos[0],
            particle_pos[1],
            particle.theta,
            length=ppi * 2,
            color=RED,
        )

    # Draw estimated position (e.g., in Green)
    pygame.draw.circle(window, GREEN, (pred_x, pred_y), ppi)
    mh.draw_orientation(
        window, pred_x, pred_y, pred_theta, length=ppi * 2, color=GREEN
    )

    pygame.display.flip()
    clock.tick(10)  # Run at a visible speed (e.g., 10 FPS)
    step_count += 1
    print(f"Simulation Step: {step_count}")
