import math
import numpy as np
from mcl2.mcl_main import get_pathfind_vars

# Coords
LOADING_ZONE = (4, 21, 4, 21) # xmax, ymax, xmax, ymax

# Tunable parameters
PATHFIND_MAX_DX = 6       # inches
PATHFIND_MAX_DY = 6       # inches
PATHFIND_MAX_DTHETA = 15  # degrees

WEIGHT_DX = 1.0
WEIGHT_DY = 0.5
WEIGHT_DTHETA = 0.3

MIN_CLEARANCE = 1.5  # inches from obstacles/walls

# Directions for candidate moves (dx, dy)
CANDIDATE_MOVES = [
    (PATHFIND_MAX_DX, 0),    # +X
    (0, PATHFIND_MAX_DY),    # +Y
    (0, -PATHFIND_MAX_DY),   # -Y
]

def compute_wall_squareness(grid, x, y, check_range=5):
    
    max_x, max_y = grid.shape
    ix, iy = int(round(x)), int(round(y))

    # horizontal squareness
    left_dist = 0
    for dx in range(1, check_range + 1):
        nx = max(ix - dx, 0)
        if grid[nx, iy] == 1:
            break
        left_dist += 1

    right_dist = 0
    for dx in range(1, check_range + 1):
        nx = min(ix + dx, max_x - 1)
        if grid[nx, iy] == 1:
            break
        right_dist += 1

    squareness_x = abs(left_dist - right_dist)

    # vertical squareness
    top_dist = 0
    for dy in range(1, check_range + 1):
        ny = min(iy + dy, max_y - 1)
        if grid[ix, ny] == 1:
            break
        top_dist += 1

    bottom_dist = 0
    for dy in range(1, check_range + 1):
        ny = max(iy - dy, 0)
        if grid[ix, ny] == 1:
            break
        bottom_dist += 1

    squareness_y = abs(top_dist - bottom_dist)
    return squareness_x + squareness_y  # smaller = better

def pathfind_step_region(target_region):
    """
    Decide a single movement toward a target region.
    target_region: (xmin, xmax, ymin, ymax)
    Returns (dX, dY, dTheta) in inches/degrees
    """
    grid, x, y, theta = get_pathfind_vars()
    print(f"pathfinding from (x={x}, y={y}, theta={theta})")
    
    xmin, xmax, ymin, ymax = target_region

    # Check if current position is already inside target
    if xmin <= x <= xmax and ymin <= y <= ymax:
        return 0, 0, 0

    # Use the center of the region as temporary target
    target_x = (xmin + xmax) / 2
    target_y = (ymin + ymax) / 2

    max_x, max_y = grid.shape
    ix, iy = int(round(x)), int(round(y))

    # Candidate moves: dx, dy, dtheta
    candidate_moves = []

    # 1. Translational moves
    for dx, dy in CANDIDATE_MOVES:
        nx, ny = x + dx, y + dy

        # Check bounds
        if nx < 0 or nx >= max_x or ny < 0 or ny >= max_y:
            continue

        # Check obstacle at target grid cell (1-inch resolution)
        ix_n, iy_n = int(round(nx)), int(round(ny))
        if grid[ix_n, iy_n] == 1:
            continue

        # Clearance: 3x3 neighborhood
        clearance_ok = True
        for cx in range(max(ix_n - 1, 0), min(ix_n + 2, max_x)):
            for cy in range(max(iy_n - 1, 0), min(iy_n + 2, max_y)):
                if grid[cx, cy] == 1:
                    clearance_ok = False
                    break
            if not clearance_ok:
                break
        if not clearance_ok:
            continue

        candidate_moves.append((dx, dy, 0))

    # 2. Rotational move toward target
    target_angle = math.atan2(target_y - y, target_x - x)
    delta_theta = math.degrees(target_angle - theta)
    delta_theta = ((delta_theta + 180) % 360) - 180  # wrap -180..180
    if abs(delta_theta) > 1e-3:
        dtheta = max(-PATHFIND_MAX_DTHETA, min(PATHFIND_MAX_DTHETA, delta_theta))
        candidate_moves.append((0, 0, dtheta))

    # 3. Score candidates
    best_score = -float("inf")
    best_move = (0, 0, 0)

    for dx, dy, dtheta in candidate_moves:
        move_score = WEIGHT_DX * dx + WEIGHT_DY * abs(dy) + WEIGHT_DTHETA * abs(dtheta)

        # Distance to target region (center)
        new_x = x + dx
        new_y = y + dy
        dist_to_target = math.hypot(target_x - new_x, target_y - new_y)
        move_score += 10.0 / (dist_to_target + 0.1)

        # Squareness penalty
        squareness = compute_wall_squareness(grid, new_x, new_y)
        move_score -= squareness

        if move_score > best_score:
            best_score = move_score
            best_move = (dx, dy, dtheta)

    return best_move
