from bisect import bisect_left

# Conversion constants
MM_TO_INCH = 0.0393701

MAX_LIDAR_POINTS = 360

class LidarPointReading:
    """Single LIDAR point: angle in degrees, distance in inches."""
    __slots__ = ("angle", "distance")

    def __init__(self, angle: float, distance_mm: float):
        self.angle = angle  # degrees
        self.distance = distance_mm * MM_TO_INCH  # convert mm → inches

    def __lt__(self, other):
        return self.angle < other.angle

    def __repr__(self):
        return f"LidarPointReading(angle={self.angle:.2f}°, distance={self.distance:.2f} in)"

class LidarReading:
    """Collection of LidarPointReading sorted by angle."""
    def __init__(self, max_points: int = MAX_LIDAR_POINTS):
        self.max_points = max_points
        self.points = []

    def clear(self):
        """Clear all points (used when sending new LIDAR request)."""
        self.points.clear()

    def add_point(self, point: LidarPointReading):
        """Add a point and keep sorted."""
        if len(self.points) >= self.max_points:
            return
        idx = bisect_left(self.points, point)
        self.points.insert(idx, point)

    def __repr__(self):
        return f"LidarReading({len(self.points)} points: {self.points})"
