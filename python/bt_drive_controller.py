import serial
import threading
import time
import sys
import keyboard  # pip install keyboard

# ======= USER SETTINGS =======
PORT = "COM6"       # Replace with your COM port
BAUD = 57600        # Match Arduino baud rate
SEND_INTERVAL = 0.01  # seconds (10 ms)
# =============================

VALID_KEYS = ['h', 'w', 'a', 's', 'd']

def print_rcvd_line(message):
    """Print Rcvd > message on its own line."""
    sys.stdout.write('\r\033[K')  # Clear line
    sys.stdout.write(f"Rcvd > {message}\n")
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
                msg = f"{key_pressed}$"
                ser.write(msg.encode('latin1'))
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
