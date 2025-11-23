from message import Message, MessageType
from sender import send

def send_gripper_request(ser, content: str):
    """Send a Gripper request over serial"""
    msg = Message(
        MessageType.GripperCommand, 
        content.encode()
    )
    send(ser, msg)
