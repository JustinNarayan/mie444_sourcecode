from message import Message, MessageType
from sender import send
import struct


def send_drive_command(ser, key: str):
    """Send a drivetrain command over serial."""
    msg = Message(MessageType.DrivetrainManualCommand, key.encode())
    send(ser, msg)


def send_drive_automated_command(
    ser, delta_x: float = 0, delta_y: float = 0, delta_theta: float = 0
):
    """Send a drivetrain automated command over serial."""
    msg = Message(
        MessageType.DrivetrainAutomatedCommand,
        struct.pack("<fff", delta_x, delta_y, delta_theta),
    )
    send(ser, msg)
