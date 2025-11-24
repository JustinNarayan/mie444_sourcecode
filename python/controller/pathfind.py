import math
import numpy as np
from mcl2.mcl_main import get_pathfind_vars

# Coords
LOADING_ZONE = (4, 21, 4, 21) # xmax, ymax, xmax, ymax

PATHFIND_MAX_DX = 6
PATHFIND_MAX_DY = 6
PATHFIND_MAX_DTHETA = 75  # degrees

def is_aligned_in_maze():
    grid, x, y, theta = get_pathfind_vars()
    max_x, max_y = grid.shape

    ix = int(round(x))
    iy = int(round(y))

    # -------------------------------------------------------------
    # Helper: exact wall position in a direction
    # -------------------------------------------------------------
    def wall_position(dx, dy, max_dist=60):
        for d in range(1, max_dist+1):
            nx = ix + dx*d
            ny = iy + dy*d
            if nx < 0 or nx >= max_x or ny < 0 or ny >= max_y:
                return d-1
            if grid[nx, ny] == 1:
                return d-1
        return max_dist

    dirs = [
        ("+X", 1, 0),
        ("-X", -1, 0),
        ("+Y", 0, 1),
        ("-Y", 0, -1),
    ]
    openness = [(wall_position(dx,dy), name, dx, dy) for (name,dx,dy) in dirs]
    openness.sort(reverse=True)

    best_dist, best_name, bdx, bdy = openness[0]

    # -------------------------------------------------------------
    # Check rotation alignment
    # -------------------------------------------------------------
    desired_theta = math.atan2(bdy, bdx)
    dtheta = desired_theta - theta
    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi

    # How close to aligned?  → treat ≤ 10° as aligned
    if abs(dtheta) > math.radians(10):
        return False

    # -------------------------------------------------------------
    # Check centering alignment
    # -------------------------------------------------------------
    if best_name in ("+X", "-X"):
        up  = wall_position(0, +1)
        down = wall_position(0, -1)
        wall_up_y = y + up
        wall_down_y = y - down
        corridor_center_y = (wall_up_y + wall_down_y) * 0.5
        center_error = abs(corridor_center_y - y)
        return center_error < 1.0   # tolerance
    else:
        right = wall_position(+1, 0)
        left  = wall_position(-1, 0)
        wall_right_x = x + right
        wall_left_x  = x - left
        corridor_center_x = (wall_right_x + wall_left_x) * 0.5
        center_error = abs(corridor_center_x - x)
        return center_error < 1.0


def align_in_maze(_unused):
    grid, x, y, theta = get_pathfind_vars()
    max_x, max_y = grid.shape

    ix = int(round(x))
    iy = int(round(y))

    # -------------------------------------------------------------
    # Helper: exact wall position in a direction
    # -------------------------------------------------------------
    def wall_position(dx, dy, max_dist=60):
        """Returns world coord value of wall in given direction."""
        for d in range(1, max_dist+1):
            nx = ix + dx*d
            ny = iy + dy*d
            if nx < 0 or nx >= max_x or ny < 0 or ny >= max_y:
                return d-1
            if grid[nx, ny] == 1:
                return d-1
        return max_dist

    # Check openness
    dirs = [
        ("+X", 1, 0),
        ("-X", -1, 0),
        ("+Y", 0, 1),
        ("-Y", 0, -1),
    ]
    openness = [(wall_position(dx,dy), name, dx, dy) for (name,dx,dy) in dirs]
    openness.sort(reverse=True)

    # Most open direction
    best_dist, best_name, bdx, bdy = openness[0]

    # -------------------------------------------------------------
    # 1. Rotation toward the open direction
    # -------------------------------------------------------------
    desired_theta = math.atan2(bdy, bdx)
    dtheta = desired_theta - theta
    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi

    max_dtheta_rad = math.radians(PATHFIND_MAX_DTHETA)
    dtheta = np.clip(dtheta, -max_dtheta_rad, max_dtheta_rad)

    # -------------------------------------------------------------
    # 2. Compute exact corridor center
    # -------------------------------------------------------------
    dx_cmd = 0
    dy_cmd = 0

    if best_name in ("+X", "-X"):
        # Horizontal corridor -> center on Y axis
        up  = wall_position(0, +1)
        down = wall_position(0, -1)

        # True world coords
        wall_up_y = y + up
        wall_down_y = y - down
        corridor_center_y = (wall_up_y + wall_down_y) * 0.5

        dy_cmd = corridor_center_y - y
        dy_cmd = np.clip(dy_cmd, -PATHFIND_MAX_DY, PATHFIND_MAX_DY)

    else:
        # Vertical corridor -> center on X axis
        right = wall_position(+1, 0)
        left  = wall_position(-1, 0)

        wall_right_x = x + right
        wall_left_x  = x - left
        corridor_center_x = (wall_right_x + wall_left_x) * 0.5

        dx_cmd = corridor_center_x - x
        dx_cmd = np.clip(dx_cmd, -PATHFIND_MAX_DX, PATHFIND_MAX_DX)

    # -------------------------------------------------------------
    # DO NOT block translation when rotating
    # Only damp if rotation is extremely large (> 135°)
    # -------------------------------------------------------------
    if abs(dtheta) > math.radians(135):
        dx_cmd *= 0.5
        dy_cmd *= 0.5

    return dx_cmd, dy_cmd, math.degrees(dtheta)

def simple_pathfind_step(target_region):
    target_x,target_y = (target_region[0] + target_region[1])/2, (target_region[2] + target_region[3])/2
    
    grid, x, y, theta = get_pathfind_vars()
    
    if (x > target_region[0] and x < target_region[1] and y > target_region[2] and y < target_region[3]):
        print("here!")
        return 0,0,0

    # Direction vector toward target
    dx = target_x - x
    dy = target_y - y

    # Clip motion
    dx_cmd = np.clip(dx, -PATHFIND_MAX_DX, PATHFIND_MAX_DX)
    dy_cmd = np.clip(dy, -PATHFIND_MAX_DY, PATHFIND_MAX_DY)

    # Optional: face direction of travel
    desired_theta = math.atan2(dy, dx)
    dtheta = desired_theta - theta
    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi
    dtheta_cmd = np.clip(dtheta, -math.radians(PATHFIND_MAX_DTHETA), math.radians(PATHFIND_MAX_DTHETA))

    return dx_cmd, dy_cmd, math.degrees(dtheta_cmd)


def pathfind_step_region(target_region):
    if not is_aligned_in_maze():
        print(f"Aligning!")
        align_in_maze(target_region)
    else:
        print(f"Pathfinding!")
        simple_pathfind_step(target_region)