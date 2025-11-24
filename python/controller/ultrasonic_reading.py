# ultrasonic_reading.py
from bisect import bisect_left
from typing import List
import matplotlib.pyplot as plt
import math
import numpy as np
from encoder_reading import EncoderReading
from inverse_kinematics import inverse_kinematics, DEG_TO_RAD, L

ULTRASONIC_1_OFFSET_TO_FORWARD = 0 * DEG_TO_RAD
ULTRASONIC_2_OFFSET_TO_FORWARD = 120 * DEG_TO_RAD
ULTRASONIC_RADIUS = 3.0 # inches
ULTRASONIC_MAX_X = 100
ULTRASONIC_MAX_Y = 100

class UltrasonicPointReading:
    """
    Represents one Ultrasonic beam measurement from ultrasonic.
    - which ultrasonic: either 1 or 2
    - encoder_reading: Encoder values
    - distance: stored in **inches** (float).
    """
    __slots__ = ("which_ultrasonic", "encoder_reading", "distance")

    def __init__(self, which_ultrasonic: int, encoder_reading: EncoderReading, distance: float):
        self.which_ultrasonic = 2 if which_ultrasonic is 2 else 1
        self.encoder_reading = encoder_reading
        self.distance = float(distance)
        
    def get_relative_coords_of_reading(self, final_encoder_reading: EncoderReading):
        """
        Returns (dX, dY, dTheta) of the ultrasonic hit *in the current robot frame*.
        """
        # 1. Sensor direction
        if self.which_ultrasonic == 1:
            offset = ULTRASONIC_1_OFFSET_TO_FORWARD
            return 0,0
        else:
            offset = ULTRASONIC_2_OFFSET_TO_FORWARD

        cos_o = math.cos(offset)
        sin_o = math.sin(offset)

        # 2. Sensor position relative to robot center
        sensor_x = ULTRASONIC_RADIUS * cos_o
        sensor_y = ULTRASONIC_RADIUS * sin_o

        # 3. Hit point in robot frame at time of reading
        hit_x_read = sensor_x + self.distance * cos_o
        hit_y_read = sensor_y + self.distance * sin_o
        
        # 4. Compute robot motion since the reading
        dX_rel, dY_rel, dTheta_rel = inverse_kinematics(
            self.encoder_reading,
            final_encoder_reading
        )

        # 5. Transform point into current robot frame
        # First shift by robot motion
        x_shift = hit_x_read - dX_rel
        y_shift = hit_y_read - dY_rel

        # Then rotate by -dTheta_rel to account for robot rotation
        theta = dTheta_rel
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        hit_x_now = x_shift * cos_t - y_shift * sin_t
        hit_y_now = x_shift * sin_t + y_shift * cos_t

        return hit_x_now, hit_y_now

    def __repr__(self):
        return f"UltrasonicPointReading(which={self.which_ultrasonic}, d1={self.encoder_reading.encoder1}, d2={self.encoder_reading.encoder2}, d3={self.encoder_reading.encoder3}, distance={self.distance:.2f} in)"


class UltrasonicReading:
    """
    Collection of UltrasonicReading from several ultrasonics
    Units:
      - encoder_reading
      - distances in inches
    """
    def __init__(self, which_ultrasonic: int = 1):
        self.points: List[UltrasonicReading] = []
        self.final_encoder_reading = None
        
    def set_final_encoder(self, final_encoder_reading: EncoderReading):
        self.final_encoder_reading = final_encoder_reading
    
    def get_final_encoder(self):
        return self.final_encoder_reading

    # ---- Construction helpers ----
    @classmethod
    def from_points(cls, points: List[UltrasonicPointReading]):
        """Create an UltrasonicReading from an existing list (makes a sorted copy)."""
        obj = cls(len(points))
        obj.points = points
        return obj

    def clear(self):
        self.points.clear()

    # ---- add / get points ----
    def add_point(self, point: UltrasonicPointReading):
        """
        Insert `point` into sorted `self.points` by angle.
        If real_lidar_data == True, the calibraton offset LIDAR_ZERO_ANGLE_OFFSET_REAL (deg)
        is applied so that "front" of robot maps to 0°.
        NOTE: This mutates the `point.angle` value if real_lidar_data == True.
        """
        self.points.append(point)

    def get_points(self) -> List[UltrasonicPointReading]:
        """Return underlying sorted list (do not mutate)."""
        return self.points

    def __repr__(self):
        return f"UltrasonicReading({len(self.points)} points: {self.points})"


# def init_ultrasonic_plot(_fig, _ax, _scatter):
#     """Initialize or recreate the ultrasonic scatter plot window. Returns updated handles."""
#     # If old fig exists but user closed it → reset
#     if _fig is not None and not plt.fignum_exists(_fig.number):
#         _fig = None
#         _ax = None
#         _scatter = None

#     # Create new figure if needed
#     if _fig is None:
#         plt.ion()
#         _fig, _ax = plt.subplots()
#         _fig.canvas.manager.set_window_title("Live Ultrasonic (x,y) Plot")

#         _ax.set_title("Ultrasonic Scan")
#         _ax.set_xlabel("x (inches)")
#         _ax.set_ylabel("y (inches)")
#         _ax.set_aspect("equal", "box")
#         _ax.grid(True)

#         # placeholder scatter
#         _scatter = _ax.scatter([], [], c='green')

#         _fig.show()

#     return _fig, _ax, _scatter


# def update_ultrasonic_plot(_fig, _ax, _scatter, ultrasonic_reading, final_encoder_reading):
#     """
#     Plot ultrasonic ping positions.
#     Each UltrasonicPointReading computes (x,y) from get_relative_coords_of_reading().
#     All plotted as green dots.
#     """
#     # Ensure figure/axes exist
#     _fig, _ax, _scatter = init_ultrasonic_plot(_fig, _ax, _scatter)

#     # Clear axes
#     _ax.cla()

#     # Restore decorations
#     _ax.set_title("Ultrasonic Scan")
#     _ax.set_xlabel("x (inches)")
#     _ax.set_ylabel("y (inches)")
#     _ax.set_aspect("equal", "box")
#     _ax.grid(True)

#     xs = []
#     ys = []

#     for p in ultrasonic_reading.get_points():
#         x, y = p.get_relative_coords_of_reading(final_encoder_reading)
#         if (abs(x) > ULTRASONIC_MAX_X or abs(y) > ULTRASONIC_MAX_Y):
#             continue
#         xs.append(x)
#         ys.append(y)
        
#     # Plot ultrasonic points in green
#     if xs:
#         _ax.scatter(xs, ys, c='green', s=20, zorder=4)

#     # Auto-limits centered at 0,0
#     if xs:
#         max_x = max(abs(min(xs)), abs(max(xs)))
#         max_y = max(abs(min(ys)), abs(max(ys)))
#         max_range = 1.1 * max(max_x, max_y)
#     else:
#         max_range = 10  # fallback

#     _ax.set_xlim(-max_range, max_range)
#     _ax.set_ylim(-max_range, max_range)

#     # Draw robot at origin
#     _ax.scatter([0.0], [0.0], c='blue', s=40, zorder=3)
#     # Draw direction indicator (straight up)
#     dir_len = 0.05 * max_range
#     _ax.plot([0.0, 0.0], [0.0, dir_len], color='blue', linewidth=2, zorder=2)

#     # Redraw
#     _fig.canvas.draw()
#     _fig.canvas.flush_events()

#     return _fig, _ax, _scatter
