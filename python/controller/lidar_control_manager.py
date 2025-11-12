from message import Message, MessageType
from sender import send
from lidar_reading import LidarReading
from encoder_control_manager import send_encoder_request

def send_lidar_request(ser, reading: LidarReading, key: str = 'l'):
    """Send a LIDAR request over serial and clear the reading."""
    reading.clear()
    msg = Message(
        MessageType.LidarState, 
        key.encode()
    )
    send(ser, msg)

	# Also request encoder
    send_encoder_request(ser)