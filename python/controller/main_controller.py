import threading
import time
import pygame
from bluetooth_manager import connect, disconnect
from receiver import start_receiver, LOCALIZATION
from control_manager import start_keyboard_listener
from lidar_reading import LidarReading
from ultrasonic_reading import UltrasonicReading
from automated_command import AutomatedCommand
from mcl2.mcl_main import begin_localization

# ======= USER SETTINGS =======
PORT = "COM6"
BAUD = 57600
SEND_INTERVAL = 0.01  # seconds (10 ms)
WAIT_INTERVAL = 0.01
# =============================


def main():
    ser = connect(PORT, BAUD)
    if not ser:
        return

    stop_event = threading.Event()

    # Single LIDAR reading instance
    lidar_reading = LidarReading()
    ultrasonic_reading = UltrasonicReading()
    current_automated_command = AutomatedCommand(0,0,0)

    start_receiver(ser, stop_event, lidar_reading, ultrasonic_reading, current_automated_command)
    start_keyboard_listener(
        ser, stop_event, lidar_reading, ultrasonic_reading, current_automated_command
    )

    # Localization begin
    if LOCALIZATION:
        begin_localization()

    try:
        while not stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_event.set()
            time.sleep(WAIT_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_event.set()
        disconnect(ser)
        pygame.quit()


if __name__ == "__main__":
    main()
