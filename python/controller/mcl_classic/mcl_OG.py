import pygame
import math
import copy
import random
import mcl_helper_OG as mh
from mcl_helper_OG import Rover, Particle

# Initialize pygame
pygame.init()

# --- Constants ---
# (Keep constants from mcl_OG.py)
GRID_WIDTH = 96 + 2 # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2 # inches (one extra inch on each side for border)
ppi = 12
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

NUM_PARTICLES = mh.NUM_PARTICLES
# Note: Original FORWARD_VELOCITY and ANGULAR_VELOCITY are no longer directly used for movement
SIMULATION_STEPS = 200 # Number of steps to run the controlled simulation (now unused for loop limit)
STEP_DURATION = 0.5 # Duration of each step in seconds (not used for delta commands)
# -----------------
    
grid = mh.init_grid()
reduced_grid = copy.deepcopy(grid)
grid = mh.expand_grid(grid, ppi)
valid_positions = mh.create_valid_positions(grid)

# --- Modified Rover Initialization (Fixed Initial Position) ---
# Select a single random valid position for the rover start
start_pos = random.choice(valid_positions)
rover = Rover(grid, valid_positions)
rover.x = start_pos[0]
rover.y = start_pos[1]
rover.theta = random.uniform(0, 2 * math.pi)
# ---------------------------------------------------------------

# Initialize particles randomly across the map
particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
pred_x, pred_y, pred_theta = mh.estimate(particles)

# Initialize window
window = pygame.display.set_mode((len(grid[0]), len(grid)))
pygame.display.set_caption("Controlled MCL Simulation (Delta Commands)")

# Main loop for controlled steps
running = True
clock = pygame.time.Clock()
step_count = 0

# --- Define a sequence of movement commands using Delta X, Delta Y, Delta Theta ---
# Each tuple is (delta_x, delta_y, delta_theta) in pixels/radians
# ppi = 12, so 12 pixels is 1 inch.
delta_commands = [
    (12, 0, 0),                       # Move 2 inches right (24 pixels)
    (0, 0, math.pi / 4),              # Rotate 45 degrees counter-clockwise
    (0, 12, 0),                       # Move 2 inches down (24 pixels)
    (0, 0, -math.pi / 4),             # Rotate 45 degrees clockwise
    (-12, 0, 0),                      # Move 1 inch left (12 pixels)
    (0, 0, -math.pi / 2),             # Rotate 90 degrees clockwise
    (0, -12, 0),                      # Move 1 inch up (12 pixels)
    (0, 0, math.pi / 2),              # Rotate 90 degrees counter-clockwise
    (12, 12, math.pi / 8),            # Move 1 inch right and 1 inch down, rotate slightly
    
    (1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),
    (1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),(1,-1,1),
    (-1,1,1),
    
] 
# Removed the repetition logic: * (SIMULATION_STEPS // 9 + 1)
# --- New loop limit variable ---
MAX_STEPS = len(delta_commands)
print(f"Simulation will run for {MAX_STEPS} steps based on the delta_commands list.")
# -----------------------------------

# --- Modified while loop condition ---
while running and step_count < MAX_STEPS:
# -------------------------------------
    # 1. Get the current command: (delta_x, delta_y, delta_theta)
    # The current_command_index is now simply step_count
    current_command_index = step_count 
    delta_x, delta_y, delta_theta = delta_commands[current_command_index]
        
    # 2. Prediction Step (Move Rover and Particles)
    # Rover movement uses the dedicated move_delta method
    rover.move_delta(delta_x, delta_y, delta_theta, grid)
    
    # Particle movement must be done manually for delta commands,
    # as the old mh.update_particles used velocities.
    for particle in particles:
        # The Particle class's delta-based move method is named 'move' 
        # and accepts three delta arguments
        particle.move(delta_x, delta_y, delta_theta, grid) 
    
    # 3. Measurement Step (Lidar Scan)
    lidar_distances = rover.lidar_scan_fast()
    
    # 4. Update Weights
    for particle in particles:
        particle_lidar_distances = particle.lidar_scan_fast()
        particle.update_weight(particle_lidar_distances, lidar_distances)
    
    # 5. Resample and Estimate
    particles, variance = mh.resample_particles(particles, grid, valid_positions, pred_x, pred_y)        
    pred_x, pred_y, pred_theta = mh.estimate(particles)

    # 6. Drawing and Display (No changes here)
    window.fill(WHITE)
    mh.draw_grid(window, reduced_grid)

    # Draw particles
    for particle in particles:
        particle_pos = (int(particle.x), int(particle.y))
        pygame.draw.circle(window, RED, particle_pos, ppi / 2)
        mh.draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=ppi * 2, color=RED)

    # Draw estimated position (e.g., in Green)
    pred_pos = (int(pred_x), int(pred_y))
    pygame.draw.circle(window, GREEN, pred_pos, ppi)
    mh.draw_orientation(window, pred_pos[0], pred_pos[1], pred_theta, length=ppi * 2, color=GREEN)

    # Draw true rover position (e.g., in Black)
    rover_pos = (int(rover.x), int(rover.y))
    rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)
    pygame.draw.rect(window, BLACK, rover_rect)
    
    # Event handling to quit the simulation
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(10) # Run at a visible speed (e.g., 10 FPS)
    step_count += 1
    print(f"Simulation Step: {step_count}/{MAX_STEPS}")

pygame.quit()