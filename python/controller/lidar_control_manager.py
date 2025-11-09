from message import Message, MessageType
from sender import send
from lidar_reading import LidarReading  # We'll define this separately

def send_lidar_request(ser, reading: LidarReading):
    """Send a LIDAR request over serial and clear the reading."""
    reading.clear()
    msg = Message(MessageType.LidarRequest, b"")
    send(ser, msg)
