import serial
import threading
import time
import sys
import keyboard  # pip install keyboard
from enum import Enum, auto

class MessageType(Enum):
	def _generate_next_value_(name, start, count, last_values):
		return count  # start counting at 0
    
	Generic = auto()
	Error = auto()
	DrivetrainManualCommand = auto()
	DrivetrainManualResponse = auto()
	DrivetrainEncoder = auto()
	ReadingLidar = auto()
	Count = auto()
 
def encode_message(type: MessageType, content: str):
    content_bytes = content.encode()
    content_size = len(content_bytes)
    return (
        type.value.to_bytes(1, "little") +
        content_size.to_bytes(1, "little") +
        content_bytes +
        b"$"
    )


# ======= USER SETTINGS =======
PORT = "COM6"       # Replace with your COM port
BAUD = 57600        # Match Arduino baud rate
SEND_INTERVAL = 0.01  # seconds (10 ms)
# =============================

VALID_KEYS = ['q', 'a', 'd', 'w', 's']

def print_rcvd_line(message):
    """Print Rcvd > message on its own line."""
    sys.stdout.write('\r\033[K')  # Clear line
    sys.stdout.write(f"Rcvd > {message.encode('utf-8')}\n")
    sys.stdout.flush()

def reader(ser, stop_event):
    """Continuously read and print incoming serial data."""
    buffer = ""
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                msg = data.decode('latin1', errors='ignore')
                print_rcvd_line(msg)
            time.sleep(0.01)
        except serial.SerialException:
            print("\n[Serial connection lost]")
            stop_event.set()
            break
        except Exception as e:
            print("\n[Reader error:]", e)
            stop_event.set()
            break

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)
        print(f"Connected to {PORT} at {BAUD} baud.\n")
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    stop_event = threading.Event()
    threading.Thread(target=reader, args=(ser, stop_event), daemon=True).start()

    try:
        while not stop_event.is_set():
            key_pressed = None
            for k in VALID_KEYS:
                if keyboard.is_pressed(k):
                    key_pressed = k
                    break  # Only take the first pressed key

            if key_pressed:
                msg = encode_message(MessageType.DrivetrainManualCommand, key_pressed)
                ser.write(msg)
                ser.flush()

            time.sleep(SEND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_event.set()
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
