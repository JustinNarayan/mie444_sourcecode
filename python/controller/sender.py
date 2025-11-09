import sys
from message import Message

def print_sent_line(msg: Message):
    """Print sent message on its own line."""
    sys.stdout.write('\r\033[K')  # Clear line
    sys.stdout.write(f"Send > {repr(msg)}\n")
    sys.stdout.flush()

def send(ser, msg: Message):
    """Send a Message over serial and log it."""
    ser.write(msg.raw)
    ser.flush()
    print_sent_line(msg)