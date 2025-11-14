import threading
import keyboard
from drive_control_manager import send_drive_command
from lidar_control_manager import send_lidar_request
from encoder_control_manager import send_encoder_request

DRIVETRAIN_KEYS = ['q', 'a', 'd', 'w', 's']
LIDAR_KEYS = ['l']
ENCODER_KEYS = ['e']

def start_keyboard_listener(ser, stop_event, lidar_reading):
    """Start a background thread to listen for keyboard input and send messages."""
    
    def keyboard_thread():
        lidar_pressed = {k: False for k in LIDAR_KEYS}  # track edge for LIDAR keys
        encoder_pressed = {k: False for k in ENCODER_KEYS}  # track edge for ENCOER keys

        while not stop_event.is_set():
            # ----- Drivetrain keys (continuous) -----
            for k in DRIVETRAIN_KEYS:
                if keyboard.is_pressed(k):
                    send_drive_command(ser, k)
                    break  # Only send one drivetrain command per cycle

            # ----- Lidar keys (edge-triggered) -----
            for k in LIDAR_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not lidar_pressed[k]:
                    send_lidar_request(ser, lidar_reading, k)
                    lidar_pressed[k] = True  # mark as pressed
                elif not is_pressed and lidar_pressed[k]:
                    # reset state when key released
                    lidar_pressed[k] = False
            
            # ----- Encoder keys (edge-triggered) -----
            for k in ENCODER_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not encoder_pressed[k]:
                    send_encoder_request(ser, k)
                    encoder_pressed[k] = True  # mark as pressed
                elif not is_pressed and encoder_pressed[k]:
                    # reset state when key released
                    encoder_pressed[k] = False

            # Small sleep to reduce CPU usage
            threading.Event().wait(0.01)

    threading.Thread(target=keyboard_thread, daemon=True).start()
