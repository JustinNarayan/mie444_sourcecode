from message import Message, MessageType
from sender import send

def send_encoder_request(ser, key: str = 'e'):
    """Send an Encoder request over serial"""
    msg = Message(
        MessageType.DrivetrainEncoderState, 
        key.encode()
    )
    send(ser, msg)
