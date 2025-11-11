# lidar_reading.py
from bisect import bisect_left
from typing import List

# --- Tunables / constants for LIDAR file ---
MM_TO_INCH = 0.0393701     # millimeters -> inches (kept for backward compatibility if needed)
DOWNSAMPLE_POINTS = 15     # default number of buckets for downsampling
EPSILON = 1e-12                # tiny value to avoid division by zero in normalization
# Force real Lidar data to conform to simulated measurements. 0 is forward.
LIDAR_ZERO_ANGLE_OFFSET_REAL = -48.0  # degrees
LIDAR_ZERO_ANGLE_OFFSET_SIMULATED = 0.0  # degrees
class LidarPointReading:
    """
    Represents one LIDAR beam measurement.
    - angle: degrees in range [0, 360). Stored in degrees for clarity.
    - distance: stored in **inches** (float).
    """
    __slots__ = ("angle", "distance")

    def __init__(self, angle_deg: float, distance: float, needs_mm_to_inch_conversion: bool):
        # keep angle normalized to [0,360)
        self.angle = float(angle_deg) % 360.0
        if needs_mm_to_inch_conversion:
            self.distance = float(distance * MM_TO_INCH) # now inches
        else:
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
                p.distance / max_dist, # unitless 0..1 (distance stored as proportion)
                needs_mm_to_inch_conversion=False # no conversion to keep in 0..1
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
                    avg_distance, 
                    needs_mm_to_inch_conversion=False # downsampling pre-existing points that must be in inches
                )
            )

        return LidarReading.from_points(downsampled)

    def __repr__(self):
        return f"LidarReading({len(self.points)} points: {self.points})"
