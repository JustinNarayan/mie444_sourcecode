# bluetooth_manager.py
import serial

def connect(port: str, baud: int):
    """Connect to the serial port and return the Serial object."""
    try:
        ser = serial.Serial(port, baud, timeout=0)
        print(f"Connected to {port} at {baud} baud.\n")
        return ser
    except Exception as e:
        print("Failed to open serial port:", e)
        return None

def disconnect(ser):
    if ser and ser.is_open:
        ser.close()
        print("Serial connection closed.")
