from bisect import bisect_left

# Conversion constants
MM_TO_INCH = 0.0393701

MAX_LIDAR_POINTS = 360
DOWNSAMPLE_POINTS = 20

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
    
    def get_points(self):
        return self.points
    
    def downsample(self, num_points: int = DOWNSAMPLE_POINTS):
        """
        Downsample the full 0–360° LIDAR sweep into `num_points` evenly spaced bins.
        Each bin covers 360 / num_points degrees and stores the average distance
        of all points in that range. If a bin has no data, it is skipped.
        """
        if not self.points:
            return LidarReading(num_points)

        bin_size = 360.0 / num_points
        downsampled = LidarReading(num_points)

        for i in range(num_points):
            start_angle = i * bin_size
            end_angle = (i + 1) * bin_size
            mid_angle = start_angle + bin_size / 2.0

            # Collect all distances in this angular range
            distances = [
                p.distance for p in self.points
                if start_angle <= p.angle < end_angle
            ]

            if distances:
                avg_distance = sum(distances) / len(distances)
                point = LidarPointReading(mid_angle, avg_distance / MM_TO_INCH)  # convert back to mm → inches
                downsampled.add_point(point)

        return downsampled

    def __repr__(self):
        return f"LidarReading({len(self.points)} points: {self.points})"
