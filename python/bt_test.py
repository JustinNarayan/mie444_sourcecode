import serial
import threading
import time
import sys
import queue

# Set your COM port and baud rate here
PORT = "COM5"   # Replace with your COM port
BAUD = 57600    # Match Arduino baud rate

def print_send_line(message):
    """Overwrite current line with Send > message"""
    sys.stdout.write('\r')                  # Return to start of line
    sys.stdout.write('\033[K')              # Clear line
    sys.stdout.write(f"Send > {message}\n") # Print send line
    sys.stdout.flush()

def print_rcvd_line(message):
    """Print Rcvd > message on a new line"""
    sys.stdout.write('\r')                  # Return to start of line
    sys.stdout.write('\033[K')              # Clear line
    sys.stdout.write(f"Rcvd > {message}\n")
    sys.stdout.write('Send > ')             # Restore input prompt
    sys.stdout.flush()

def reader(ser, input_queue):
    """Continuously read raw bytes and print if data is available."""
    while True:
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                msg = data.decode('latin1')
                print_rcvd_line(msg)
            time.sleep(0.01)
        except serial.SerialException:
            print("\n[Serial connection lost]")
            break
        except Exception as e:
            print("\n[Reader error:]", e)
            break

def input_thread(input_queue):
    """Thread that reads user input non-blocking and puts it into a queue."""
    while True:
        try:
            line = input('Send > ')
            input_queue.put(line)
        except EOFError:
            break

def main():
    if not PORT or not BAUD:
        print("Please set PORT and BAUD at the top of the script.")
        return

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)  # Non-blocking
        print(f"Connected to {PORT} at {BAUD} baud.\n")
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    # Thread-safe queue for user input
    input_queue = queue.Queue()

    # Start threads
    threading.Thread(target=reader, args=(ser, input_queue), daemon=True).start()
    threading.Thread(target=input_thread, args=(input_queue,), daemon=True).start()

    try:
        while True:
            # Check if user typed anything
            if not input_queue.empty():
                user_input = input_queue.get()
                # Replace literal '\n' with actual newline character
                to_send = user_input.replace("\\n", "\n").encode('latin1')
                ser.write(to_send)
                ser.flush()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        print("\nSerial connection closed.")

if __name__ == "__main__":
    main()
