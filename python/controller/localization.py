# Placeholder for localization logic
from encoder_reading import EncoderReading

# Global variables
current_encoder_reading: EncoderReading = EncoderReading()
last_sent_encoder_reading: EncoderReading | None = None
post_lidar_encoder_reading: EncoderReading | None = None

def prepare_info_for_localization_step():
    """
    Called whenever a LidarComplete message is received.
    Updates post_lidar_encoder_reading and calls get_delta_position_orientation.
    """
    global post_lidar_encoder_reading, last_sent_encoder_reading, current_encoder_reading

    # Set post-lidar encoder reading
    post_lidar_encoder_reading = current_encoder_reading.copy()

    # Compute delta using last sent encoder
    if last_sent_encoder_reading is not None:
        delta_x, delta_y, delta_theta = get_delta_position_orientation(
            post_lidar_encoder_reading,
            last_sent_encoder_reading
        )
        # Do something with delta if needed
        print(delta_x, delta_y, delta_theta)

    # Update last sent reading
    last_sent_encoder_reading = post_lidar_encoder_reading.copy()

    # Clear post-lidar reading
    post_lidar_encoder_reading = None

def get_delta_position_orientation(post_lidar: EncoderReading, last_sent: EncoderReading):
    """
    Placeholder function: compute delta_x, delta_y, delta_theta
    between two encoder readings.
    """
    # TODO: Implement actual localization calculation
    return 0.0, 0.0, 0.0
