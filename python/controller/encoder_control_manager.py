from message import Message, MessageType
from sender import send

def send_encoder_request(ser):
    """Send an Encoder request over serial"""
    msg = Message(MessageType.DrivetrainEncoderRequest, b"")
    send(ser, msg)
