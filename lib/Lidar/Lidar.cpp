#if defined(BOARD_CONTROLLER)
#include "Lidar.h"
#include "MemoryUtilities.h"

/**
 * @brief Initialize control pin and Serial interface
 * 
 * @param controlPin 
 * @param port RX/TX Serial connection
 * 
 */
void Lidar::init(uint8_t controlPin, HardwareSerial* port)
{
	this->controlPin = controlPin;
	this->port = port;

	pinMode(this->controlPin, OUTPUT);
	analogWrite(this->controlPin, LIDAR_MOTOR_SPIN_SPEED);

	this->rpLidar.begin(*(this->port)); // May not be required
}

/**
 * @brief Request a complete reading from the Lidar module.
 * 
 * @param reading Now populated with Lidar data
 * @return  LIDAR_READING_FAILURE if failed
 * 			LIDAR_READING_SUCCESS otherwise
 */
LidarState Lidar::requestReading(LidarReading* reading)
{
	// Verify Lidar is active
	if (!this->rpLidar.isOpen()) return LidarState::NotOpen;

	// Clear bitmask
	memorySet(&(reading->bitmask), 0, sizeof(reading->bitmask));

	// Start scan
	if (IS_FAIL(this->rpLidar.startScan(true))) return LidarState::CannotScan;
	unsigned long scanStartTime = millis();

	// Populate reading
	reading->numCollected = 0;
	while( 
		// Lidar not timed out
		((millis() - scanStartTime) < LIDAR_SWEEP_TIMEOUT_MS) &&
		// Points left to be collected
		(reading->numCollected < LIDAR_GRANULARITY_NUM_POINTS)
	)
	{
		// Verify point is available
		if (!IS_OK(this->rpLidar.waitPoint())) continue;

		// Read current point
		const RPLidarMeasurement currentPoint = this->rpLidar.getCurrentPoint();
		float32_t angle = currentPoint.angle;
		float32_t distance = currentPoint.distance;
		uint8_t quality = currentPoint.quality;

		// Only proceed if quality is sufficient
		if (
			(quality > LIDAR_MINIMUM_QUALITY_TO_SEND) &&
			(distance != 0)
		) {
			// Determine index
			lidarPointIndex pointIdx = ((lidarPointIndex)(
				(angle / 360.0f) * LIDAR_GRANULARITY_NUM_POINTS
			)) % LIDAR_GRANULARITY_NUM_POINTS;

			// Skip if this point has been populated
			if (false == BITMASK_IS_SET(reading->bitmask, pointIdx))
			{
				BITMASK_SET(reading->bitmask, pointIdx);
				reading->point[pointIdx].angle = (lidarAngle_deg)angle;
				reading->point[pointIdx].distance = (lidarDistance_mm)distance;
				reading->numCollected++;
			}
		}
	}

	// Stop scanning
	this->rpLidar.stop();

	return LidarState::Success;
}

#endif