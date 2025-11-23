import threading
import keyboard
from drive_control_manager import send_drive_command, send_drive_automated_command
from lidar_control_manager import send_lidar_request
from automated_command import AutomatedCommand
from ultrasonic_control_manager import send_ultrasonic_request
from encoder_control_manager import send_encoder_request
from gripper_control_manager import send_gripper_request
from mcl2.mcl_main import reset_localization

DRIVETRAIN_KEYS = ["q", "z", "a", "d", "w", "s"]
DRIVETRAIN_AUTOMATED_KEYS = ["y", "h", "g", "j", "t", "u", "b"]
CURRENT_AUTOMATED_COMMAND_KEY = "b"
LIDAR_KEYS = ["l"]
ULTRASONIC_KEYS = ["p"]
LOCALIZATION_RESET_KEYS = ['`']
ENCODER_KEYS = ["e"]
GRIPPER_KEYS = ['1', '2', '3', '4', '5', '0']

GRIPPER_KEYS_MAPPING = {
    '1': "home",
    '2': "extend",
    '3': "ready",
    '4': "open",
    '5': "close",
    '0': 'ping'
}

DELTA_POS, DELTA_ANG = 6, 10
DRIVETRAIN_AUTOMATED_KEYS_MAPPING = {
    # key: [delta_x (inches), delta_y (inches), delta_theta (radians)]
    "y": AutomatedCommand(DELTA_POS, 0.0, 0.0),  # forward
    "h": AutomatedCommand(-DELTA_POS, 0.0, 0.0),  # backward
    "g": AutomatedCommand(0.0, 0.0, DELTA_ANG),  # rotate left
    "j": AutomatedCommand(0.0, 0.0, -DELTA_ANG),  # rotate right
    "t": AutomatedCommand(0.0, DELTA_POS, 0.0),  # strate left
    "u": AutomatedCommand(0.0, -DELTA_POS, 0.0),  # strage right
    "b": AutomatedCommand(0,0,0)
}



def start_keyboard_listener(ser, stop_event, lidar_reading, ultrasonic_reading, current_automated_command: AutomatedCommand):
    """Start a background thread to listen for keyboard input and send messages."""

    def keyboard_thread():
        drivetrain_automated_pressed = {
            k: False for k in DRIVETRAIN_AUTOMATED_KEYS
        }  # track edge for drivetrain automated keys
        lidar_pressed = {k: False for k in LIDAR_KEYS}  # track edge for LIDAR keys
        ultrasonic_pressed = {k: False for k in ULTRASONIC_KEYS}  # track edge for ULTRASONIC keys
        localization_reset_pressed = {k: False for k in LOCALIZATION_RESET_KEYS}  # track edge for localization reset keys
        encoder_pressed = {
            k: False for k in ENCODER_KEYS
        }  # track edge for ENCODER keys
        gripper_pressed = {
            k: False for k in GRIPPER_KEYS
        }  # track edge for ENCODER keys

        while not stop_event.is_set():
            # ----- Drivetrain keys (continuous) -----
            for k in DRIVETRAIN_KEYS:
                if keyboard.is_pressed(k):
                    send_drive_command(ser, k)
                    break  # Only send one drivetrain command per cycle

            # ----- Drivetrain automated keys (edge-triggered) -----
            for k in DRIVETRAIN_AUTOMATED_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not drivetrain_automated_pressed[k]:
                    # Get mapping of keys
                    [delta_x, delta_y, delta_theta] = DRIVETRAIN_AUTOMATED_KEYS_MAPPING[k].get()
                    if k is CURRENT_AUTOMATED_COMMAND_KEY:
                        [delta_x, delta_y, delta_theta] = current_automated_command.get()
                    send_drive_automated_command(ser, delta_x, delta_y, delta_theta)
                    drivetrain_automated_pressed[k] = True  # mark as pressed
                elif not is_pressed and drivetrain_automated_pressed[k]:
                    # reset state when key released
                    drivetrain_automated_pressed[k] = False

            # ----- Lidar keys (edge-triggered) -----
            for k in LIDAR_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not lidar_pressed[k]:
                    send_lidar_request(ser, lidar_reading, k)
                    lidar_pressed[k] = True  # mark as pressed
                elif not is_pressed and lidar_pressed[k]:
                    # reset state when key released
                    lidar_pressed[k] = False

            # ----- Ultrasonic keys (edge-triggered) -----
            for k in ULTRASONIC_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not ultrasonic_pressed[k]:
                    send_ultrasonic_request(ser, ultrasonic_reading, k)
                    ultrasonic_pressed[k] = True  # mark as pressed
                elif not is_pressed and ultrasonic_pressed[k]:
                    # reset state when key released
                    ultrasonic_pressed[k] = False
            
            # ----- Localization reset keys (edge-triggered) -----
            for k in LOCALIZATION_RESET_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not localization_reset_pressed[k]:
                    reset_localization()
                    localization_reset_pressed[k] = True  # mark as pressed
                elif not is_pressed and localization_reset_pressed[k]:
                    # reset state when key released
                    localization_reset_pressed[k] = False

            # ----- Encoder keys (edge-triggered) -----
            for k in ENCODER_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not encoder_pressed[k]:
                    send_encoder_request(ser, k)
                    encoder_pressed[k] = True  # mark as pressed
                elif not is_pressed and encoder_pressed[k]:
                    # reset state when key released
                    encoder_pressed[k] = False

            # ----- Gripper keys (edge-triggered) -----
            for k in GRIPPER_KEYS:
                is_pressed = keyboard.is_pressed(k)
                if is_pressed and not gripper_pressed[k]:
                    send_gripper_request(ser, GRIPPER_KEYS_MAPPING[k])
                    gripper_pressed[k] = True  # mark as pressed
                elif not is_pressed and gripper_pressed[k]:
                    # reset state when key released
                    gripper_pressed[k] = False

            # Small sleep to reduce CPU usage
            threading.Event().wait(0.01)

    threading.Thread(target=keyboard_thread, daemon=True).start()
