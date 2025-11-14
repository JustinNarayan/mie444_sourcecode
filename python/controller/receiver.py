import threading
import time
import sys
import serial
from message import Message, MESSAGE_END_CHAR, parse_message_from_buffer, MessageType, SHOULD_NOT_PRINT_TO_SCREEN
from lidar_reading import LidarPointReading, LidarReading
from encoder_reading import EncoderReading
from localization import current_encoder_reading, last_sent_encoder_reading, post_lidar_encoder_reading, prepare_info_for_localization_step
from encoder_control_manager import send_encoder_request
from lidar_reading import init_lidar_plot, update_lidar_plot

# Lidar visualization
VISUALIZE_LIDAR = True

def print_rcvd_message(msg: Message):
    sys.stdout.write('\r\033[K')
    sys.stdout.write(f"Rcvd > {repr(msg)}\n")
    sys.stdout.flush()

def start_receiver(ser, stop_event, lidar_reading: LidarReading):
    """Background thread to continuously read from serial and update readings."""
    def reader():
        buffer = bytearray()
        synced = False
        lidar_reading_ready_for_localization = False
        
        # Lidar vis
        _lidar_fig = None
        _lidar_ax = None
        _lidar_scatter = None

        while not stop_event.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    time.sleep(0.01)
                    continue

                buffer.extend(data)

                # Step 1: synchronize to first '$'
                if not synced:
                    if MESSAGE_END_CHAR in buffer:
                        idx = buffer.index(MESSAGE_END_CHAR)
                        buffer = buffer[idx + 1:]
                        synced = True
                        print("[Receiver synchronized to stream]")
                        
                        # Ping encoder for first reading
                        send_encoder_request(ser)
                        
                        # Initialize lidar vis
                        if VISUALIZE_LIDAR:
                            _lidar_fig, _lidar_ax, _lidar_scatter = init_lidar_plot(_lidar_fig, _lidar_ax, _lidar_scatter)
                    else:
                        continue

                # Step 2: parse messages
                while True:
                    try:
                        msg, consumed = parse_message_from_buffer(buffer)
                        if msg:
                            # --- LIDAR integration ---
                            if msg.type == MessageType.LidarPointReading:
                                angle_deg, distance_mm = msg.decode()
                                point = LidarPointReading(
                                    angle_deg, 
                                    distance_mm, 
                                    needs_mm_to_inch_conversion=True
                                )
                                lidar_reading.add_point(point, is_real_lidar_data=True)

                            elif msg.type == MessageType.LidarState:
                                if msg.get_content() == b'complete': # complete
									# Ping encoder after receiving a complete lidar scan
                                    send_encoder_request(ser)
                                    lidar_reading_ready_for_localization = True
                                    # Visualize
                                    if VISUALIZE_LIDAR:
                                        _lidar_fig, _lidar_ax, _lidar_scatter = update_lidar_plot(_lidar_fig, _lidar_ax, _lidar_scatter, lidar_reading)

                            # --- ENCODER integration ---
                            elif msg.type == MessageType.DrivetrainEncoderDistances:
                                current_encoder_reading.update_from_msg(msg)

                                # Initialize last_sent_encoder_reading if None
                                from localization import last_sent_encoder_reading
                                if last_sent_encoder_reading is None:
                                    last_sent_encoder_reading = current_encoder_reading.copy()
                                    
                                # Step localization if Lidar reading ready
                                if lidar_reading_ready_for_localization:
                                    prepare_info_for_localization_step(lidar_reading)
                                    lidar_reading_ready_for_localization = False
                                    
                            if (msg.get_type() not in SHOULD_NOT_PRINT_TO_SCREEN):
                                print_rcvd_message(msg)
                            buffer = buffer[consumed:]
                            continue
                        break  # not enough data yet
                    except ValueError as e:
                        hex_str = " ".join(f"{b:02X}" for b in buffer)
                        print(f"[Receiver decode error: {e}] Raw ({len(buffer)} bytes): {hex_str}")
                        # Skip until next '$'
                        if MESSAGE_END_CHAR in buffer:
                            idx = buffer.index(MESSAGE_END_CHAR)
                            buffer = buffer[idx + 1:]
                        else:
                            buffer.clear()
                        synced = False
                        break

            except serial.SerialException as e:
                print("\n[Serial connection lost : {e}]")
                stop_event.set()
                break
            except Exception as e:
                print(f"\n[Receiver fatal error: {e}]")
                stop_event.set()
                break

    threading.Thread(target=reader, daemon=True).start()
