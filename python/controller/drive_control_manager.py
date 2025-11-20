from message import Message, MessageType
from sender import send
import struct


def send_drive_command(ser, key: str):
    """Send a drivetrain command over serial."""
    msg = Message(MessageType.DrivetrainManualCommand, key.encode())
    send(ser, msg)


def send_drive_automated_command(
    ser, delta_x: int = 0, delta_y: int = 0, delta_theta: int = 0
):
    """Send a drivetrain automated command over serial."""
    msg = Message(
        MessageType.DrivetrainAutomatedCommand,
        struct.pack("<hhh", int(delta_x), int(delta_y), int(delta_theta)),
    )
    send(ser, msg)
