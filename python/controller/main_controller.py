import threading
import time
import pygame
from bluetooth_manager import connect, disconnect
from receiver import start_receiver
from control_manager import start_keyboard_listener
from lidar_reading import LidarReading
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

    start_receiver(ser, stop_event, lidar_reading)
    start_keyboard_listener(ser, stop_event, lidar_reading)  # you may pass lidar_reading if needed
    
    # Localization begin
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
