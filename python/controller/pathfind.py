import math
import numpy as np
from mcl2.mcl_main import get_pathfind_vars
from collections import deque

# Coords
LOADING_ZONE = (4, 21, 4, 21) # xmax, ymax, xmax, ymaximport math

PATHFIND_MAX_DX = 6
PATHFIND_MAX_DY = 6
PATHFIND_MAX_DTHETA = 75  # degrees

# -------------------------------------------------------------
# 1️⃣ Alignment function
# -------------------------------------------------------------
def align_in_maze():
    grid, x, y, theta = get_pathfind_vars()
    max_y, max_x = grid.shape
    ix, iy = int(round(x)), int(round(y))

    def wall_position(dx, dy, max_dist=60):
        for d in range(1, max_dist+1):
            nx = ix + dx*d
            ny = iy + dy*d
            if nx < 0 or nx >= max_x or ny < 0 or ny >= max_y:
                return d-1
            if grid[ny, nx] == 1:
                return d-1
        return max_dist

    dirs = [("+X",1,0), ("-X",-1,0), ("+Y",0,1), ("-Y",0,-1)]
    openness = [(wall_position(dx,dy), name, dx, dy) for (name,dx,dy) in dirs]
    openness.sort(reverse=True)
    best_dist, best_name, bdx, bdy = openness[0]

    # Rotate toward most open direction
    desired_theta = math.atan2(bdy, bdx)
    dtheta = desired_theta - theta
    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi
    dtheta = np.clip(dtheta, -math.radians(PATHFIND_MAX_DTHETA), math.radians(PATHFIND_MAX_DTHETA))

    dx_cmd, dy_cmd = 0.0, 0.0
    if best_name in ("+X","-X"):
        up  = wall_position(0,+1)
        down = wall_position(0,-1)
        wall_up_y = y + up
        wall_down_y = y - down
        corridor_center_y = (wall_up_y + wall_down_y) * 0.5
        dy_cmd = np.clip(corridor_center_y - y, -PATHFIND_MAX_DY, PATHFIND_MAX_DY)
    else:
        right = wall_position(+1,0)
        left  = wall_position(-1,0)
        wall_right_x = x + right
        wall_left_x  = x - left
        corridor_center_x = (wall_right_x + wall_left_x) * 0.5
        dx_cmd = np.clip(corridor_center_x - x, -PATHFIND_MAX_DX, PATHFIND_MAX_DX)

    # Damp translation if huge rotation
    if abs(dtheta) > math.radians(135):
        dx_cmd *= 0.5
        dy_cmd *= 0.5

    return dx_cmd, dy_cmd, math.degrees(dtheta)

# -------------------------------------------------------------
# 2️⃣ Check alignment
# -------------------------------------------------------------
def is_aligned_in_maze():
    dx, dy, dtheta = align_in_maze()
    # aligned if both translation and rotation are small
    return abs(dx) < 0.5 and abs(dy) < 0.5 and abs(dtheta) < 10.0

# -------------------------------------------------------------
# 3️⃣ BFS shortest path to target
# -------------------------------------------------------------
def bfs_shortest_path(grid, start, targets):
    """Return a list of (x,y) from start to nearest target."""
    max_y, max_x = grid.shape
    visited = set()
    q = deque()
    q.append((start, [start]))

    while q:
        (cx, cy), path = q.popleft()
        if (cx, cy) in visited:
            continue
        visited.add((cx, cy))

        if any(tx0 <= cx <= tx1 and ty0 <= cy <= ty1 for (tx0,tx1,ty0,ty1) in targets):
            return path  # found a point inside a target region

        # Explore neighbors (4 cardinal)
        for nx, ny in [(cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)]:
            if 0 <= nx < max_x and 0 <= ny < max_y and grid[ny][nx] == 0:
                q.append(((nx,ny), path + [(nx,ny)]))
    return None  # no path found

# -------------------------------------------------------------
# 4️⃣ Move along path
# -------------------------------------------------------------
def move_along_path(path):
    grid, x, y, theta = get_pathfind_vars()
    if len(path) < 2:
        return 0.0, 0.0, 0.0  # already at target

    next_x, next_y = path[1]
    dx_cmd = np.clip(next_x - x, -PATHFIND_MAX_DX, PATHFIND_MAX_DX)
    dy_cmd = np.clip(next_y - y, -PATHFIND_MAX_DY, PATHFIND_MAX_DY)

    # Rotate toward movement direction
    if dx_cmd != 0 or dy_cmd != 0:
        desired_theta = math.atan2(dy_cmd, dx_cmd)
        dtheta = desired_theta - theta
        dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi
        dtheta_cmd = np.clip(dtheta, -math.radians(PATHFIND_MAX_DTHETA), math.radians(PATHFIND_MAX_DTHETA))
    else:
        dtheta_cmd = 0.0

    return dx_cmd, dy_cmd, math.degrees(dtheta_cmd)

# -------------------------------------------------------------
# 5️⃣ Main pathfinding function
# -------------------------------------------------------------
def pathfind_step_region(target_region):
    grid, x, y, theta = get_pathfind_vars()

    # 1. Already in target?
    if target_region[0] <= x <= target_region[1] and target_region[2] <= y <= target_region[3]:
        return 0.0, 0.0, 0.0

    # 2. Align in maze first
    if not is_aligned_in_maze():
        print ("not aligned!")
        return align_in_maze()

    # 3. Compute BFS path
    start = (int(round(x)), int(round(y)))
    targets = [(target_region[0], target_region[1], target_region[2], target_region[3])]
    path = bfs_shortest_path(grid, start, targets)
    if not path or len(path) < 2:
        # Fallback: just move toward center
        target_x = (target_region[0]+target_region[1])/2
        target_y = (target_region[2]+target_region[3])/2
        dx_cmd = np.clip(target_x - x, -PATHFIND_MAX_DX, PATHFIND_MAX_DX)
        dy_cmd = np.clip(target_y - y, -PATHFIND_MAX_DY, PATHFIND_MAX_DY)
        return dx_cmd, dy_cmd, 0.0

    # 4. Move along path
    print ("pathfinding!")
    return move_along_path(path)
