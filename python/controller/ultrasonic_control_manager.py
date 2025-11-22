from message import Message, MessageType
from sender import send
from ultrasonic_reading import UltrasonicReading
from encoder_control_manager import send_encoder_request

def send_ultrasonic_request(ser, reading: UltrasonicReading, key: str = 'p'):
    """Send an Ultrasonic request over serial and clear the reading."""
    reading.clear()
    msg = Message(
        MessageType.UltrasonicState, 
        key.encode()
    )
    send(ser, msg)

	# Also request encoder
    send_encoder_request(ser)