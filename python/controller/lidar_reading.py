# lidar_reading.py
from bisect import bisect_left
from typing import List
import matplotlib.pyplot as plt
import math
from ultrasonic_reading import UltrasonicReading, ULTRASONIC_MAX_X, ULTRASONIC_MAX_Y
import numpy as np

# --- Tunables / constants for LIDAR file ---
DOWNSAMPLE_POINTS = 30     # default number of buckets for downsampling
EPSILON = 1e-12                # tiny value to avoid division by zero in normalization
# Force real Lidar data to conform to simulated measurements. 0 is forward.
LIDAR_ZERO_ANGLE_OFFSET_REAL = -48.0  # degrees
LIDAR_ZERO_ANGLE_OFFSET_SIMULATED = 0.0  # degrees
L = 3.73771654 # inches, 94.938 mm - DISTANCE_FROM_OBJECT_CENTER_TO_WHEEL_MIDPOINT
GRIPPER_WIDTH = 2.5
GRIPPER_LENGTH = 6

class LidarPointReading:
    """
    Represents one LIDAR beam measurement.
    - angle: degrees in range [0, 360). Stored in degrees for clarity.
    - distance: stored in **inches** (float).
    """
    __slots__ = ("angle", "distance")

    def __init__(self, angle_deg: float, distance: float):
        # keep angle normalized to [0,360)
        self.angle = float(angle_deg) % 360.0
        self.distance = float(distance) # already inches

    def __lt__(self, other):
        # For bisect sorting by angle
        return self.angle < other.angle

    def __repr__(self):
        return f"LidarPointReading(angle={self.angle:.2f}°, distance={self.distance:.2f} in)"


class LidarReading:
    """
    Collection of LidarPointReading sorted by angle.
    Units:
      - angles in degrees
      - distances in inches
    """
    def __init__(self, max_points: int = 360):
        self.max_points = max_points
        self.points: List[LidarPointReading] = []

    # ---- Construction helpers ----
    @classmethod
    def from_points(cls, points: List[LidarPointReading]):
        """Create a LidarReading from an existing list (makes a sorted copy)."""
        obj = cls(len(points))
        obj.points = sorted(points, key=lambda p: p.angle)
        return obj

    def clear(self):
        self.points.clear()

    # ---- add / get points ----
    def add_point(self, point: LidarPointReading, is_real_lidar_data: bool):
        """
        Insert `point` into sorted `self.points` by angle.
        If real_lidar_data == True, the calibraton offset LIDAR_ZERO_ANGLE_OFFSET_REAL (deg)
        is applied so that "front" of robot maps to 0°.
        NOTE: This mutates the `point.angle` value if real_lidar_data == True.
        """
        if len(self.points) >= self.max_points:
            return

        if is_real_lidar_data:
            point.angle = (point.angle + LIDAR_ZERO_ANGLE_OFFSET_REAL) % 360.0
        else:
            point.angle = (point.angle + LIDAR_ZERO_ANGLE_OFFSET_SIMULATED) % 360.0

        idx = bisect_left(self.points, point)
        self.points.insert(idx, point)

    def get_points(self) -> List[LidarPointReading]:
        """Return underlying sorted list (do not mutate)."""
        return self.points

    # ---- normalization ----
    def get_normalized(self):
        """
        Return a NEW LidarReading with distances scaled to (0, 1].
        Scaling: distance_inches / max_distance_inches
        If there are no points or max_distance == 0, returns an empty LidarReading.
        """
        if not self.points:
            return LidarReading(self.max_points)

        max_dist = max((p.distance for p in self.points), default=0.0)

        if max_dist <= 0.0 + EPSILON:
            # No valid distances
            return LidarReading(self.max_points)

        normalized = [
            LidarPointReading(
                p.angle, 
                p.distance / max_dist # unitless 0..1 (distance stored as proportion)
            )
            for p in self.points
        ]
        return LidarReading.from_points(normalized)

    # ---- downsampling ----
    def get_downsampled(self, num_points: int = DOWNSAMPLE_POINTS, align_to: float = None):
        """
        Downsample current reading into `num_points` evenly spaced angular buckets.

        Bucket centers: c_i = i * (360 / num_points), i = 0..num_points-1
        Each bucket covers: (c_i - bin_half, c_i + bin_half] where bin_half = 180/num_points degrees.
        Wrap-around is handled (angles near 0 and 360).

        For each bucket:
          - Gather all points p whose p.angle falls into the bucket range (with wrap handling)
          - If bucket contains >=1 points, compute avg distance and produce one LidarPointReading
            located at the bucket center angle with that averaged distance.
          - If bucket empty: skip it (do NOT produce a 0-distance placeholder).
        Returns:
          - a NEW LidarReading containing the downsampled points (angles in degrees, distances in inches)
        """
        if not self.points:
            return LidarReading(num_points)

        # If num_points >= number of measured angles, prefer to return the reading binned by unique angles?
        # But per your request: if num_points >= len(points) -> do nothing (return copy)
        if num_points >= len(self.points):
            return LidarReading.from_points(self.points)

        bin_half = 180.0 / float(num_points)  # half-width of each bucket in degrees
        downsampled = []

        # iterate bucket centers starting at 0 degrees as requested
        for i in range(num_points):
            center = (i * 360.0 / num_points) % 360.0
            start = (center - bin_half) % 360.0
            end = (center + bin_half) % 360.0

            # collect points in bucket (handle wrap)
            if start < end:
                bucket = [p for p in self.points if (start <= p.angle < end)]
            else:
                # wrap-around example: start=316, end=45
                bucket = [p for p in self.points if (p.angle >= start or p.angle < end)]

            if not bucket:
                continue  # skip empty buckets per your requirement

            avg_distance = sum(p.distance for p in bucket) / len(bucket)
            downsampled.append(
                LidarPointReading(
                    center, 
                    avg_distance
                )
            )

        return LidarReading.from_points(downsampled)

    def __repr__(self):
        return f"LidarReading({len(self.points)} points: {self.points})"


def init_lidar_plot(_lidar_fig, _lidar_ax, _lidar_scatter):
    """Initialize or recreate the LIDAR scatter plot window. Returns updated handles."""
    # If old fig exists but user closed the window → reset
    if _lidar_fig is not None and not plt.fignum_exists(_lidar_fig.number):
        _lidar_fig = None
        _lidar_ax = None
        _lidar_scatter = None

    # Create new figure if needed
    if _lidar_fig is None:
        plt.ion()
        _lidar_fig, _lidar_ax = plt.subplots()
        _lidar_fig.canvas.manager.set_window_title("Live LIDAR (x,y) Plot")

        _lidar_ax.set_title("LIDAR Scan")
        _lidar_ax.set_xlabel("x")
        _lidar_ax.set_ylabel("y")
        _lidar_ax.set_aspect("equal", "box")
        _lidar_ax.grid(True)

        # initial placeholder scatter (will be replaced on update)
        _lidar_scatter = _lidar_ax.scatter([], [])

        _lidar_fig.show()

    return _lidar_fig, _lidar_ax, _lidar_scatter


def update_lidar_plot(_lidar_fig, _lidar_ax, _lidar_scatter, lidar_reading, ultrasonic_reading: UltrasonicReading = None):
    """
    Clear the plot completely and draw:
      - a blue dot at (0,0)
      - a short blue line pointing straight up from (0,0)
    Returns updated (fig, ax, scatter) handles. If the plot window was closed,
    this will recreate it.
    """
    # ensure figure/axes exist (and reassign returned handles)
    _lidar_fig, _lidar_ax, _lidar_scatter = init_lidar_plot(_lidar_fig, _lidar_ax, _lidar_scatter)

    # Clear entire axes
    _lidar_ax.cla()

    # Redraw axes decorations after clearing
    _lidar_ax.set_title("LIDAR Scan")
    _lidar_ax.set_xlabel("x")
    _lidar_ax.set_ylabel("y")
    _lidar_ax.set_aspect("equal", "box")
    _lidar_ax.grid(True)

    # Convert lidar points → x,y
    xs = []
    ys = []
    normalized = False

    for p in lidar_reading.get_points():
        theta_rad = math.radians(-p.angle + 90)
        r = p.distance  # normalized 0..1
        x = r * math.cos(theta_rad)
        y = r * math.sin(theta_rad)
        xs.append(x)
        ys.append(y)
    
    if max(xs) < 1:
        normalized = True

    # Plot lidar points as red dots
    if xs:
        _lidar_ax.scatter(xs, ys, c='red', s=20, zorder=4)
        
    # Plot ultrasonic
    uxs, uys = [], []
    if ultrasonic_reading is not None:
        for p in ultrasonic_reading.get_points():
            x, y = p.get_relative_coords_of_reading(ultrasonic_reading.get_final_encoder())
            if (abs(x) > ULTRASONIC_MAX_X or abs(y) > ULTRASONIC_MAX_Y):
                continue
            uxs.append(x)
            uys.append(y)
        
        if uys:
            _lidar_ax.scatter(uxs, uys, c='green', s=20, zorder=4)

    # Compute auto-limits centered at 0,0
    if xs:
        max_x = max(abs(min(xs)), abs(max(xs)))
        max_y = max(abs(min(ys)), abs(max(ys)))
    else:
        max_x = max_y = 1.0  # fall back if no data

    max_range = 1.1 * max(max_x, max_y)

    _lidar_ax.set_xlim(-max_range, max_range)
    _lidar_ax.set_ylim(-max_range, max_range)
    
    
    # Draw robot
    _lidar_ax.scatter([0.0], [0.0], c='blue', s=40, zorder=3)
    # Draw a short line facing straight up from (0,0)
    dir_len = 0.05 * max_range  # length of the direction indicator (adjust as desired)
    _lidar_ax.plot([0.0, 0.0], [0.0, dir_len], color='blue', linewidth=2, zorder=2)
    
    # --- Draw gripper rectangle ---
    rect = plt.Rectangle(
        (-GRIPPER_WIDTH/2, 0),         # (x_min, y_min)
        GRIPPER_WIDTH,                 # width  (spans in x)
        GRIPPER_LENGTH,                 # height (spans in y)
        fill=False,
        edgecolor='green',
        linewidth=2,
        zorder=2
    )
    _lidar_ax.add_patch(rect)
    
    if not normalized:
        # Plot circle of radius L
        circle = plt.Circle((0, 0), L, fill=False, edgecolor='green', linewidth=1.5, zorder=1)
        _lidar_ax.add_patch(circle)

    # Redraw and flush events
    _lidar_fig.canvas.draw()
    _lidar_fig.canvas.flush_events()

    return _lidar_fig, _lidar_ax, _lidar_scatter