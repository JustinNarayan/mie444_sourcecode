from message import Message, MessageType
from sender import send

def send_drive_command(ser, key: str):
    """Send a drivetrain command over serial."""
    msg = Message(MessageType.DrivetrainManualCommand, key.encode())
    send(ser, msg)
