#include "LidarController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new LidarController
 * 
 * @param lidar
 * @param driveController To request a full halt before pinging Lidar
 */
LidarController::LidarController(Lidar* lidar, DriveController* driveController) :
	lidar(lidar),
	driveController(driveController),
	reading({0}),
	hasUnaddressedRequest(false),
	lastCompleteSentTimestampMillis(0) {}

/**
 * @brief Read input messages for LidarRequest type
 * 
 */
void LidarController::checkLidarRequest(void)
{
	// Dequeue LidarRequest
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::LidarRequest, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		if (this->shouldAcceptNewRequests())
		{
			this->hasUnaddressedRequest = true;
			this->reading = {0};
		}

		// Eliminate stale messages
		this->purge(MessageType::LidarRequest);
	}
}

/**
 * @brief Ping lidar for a complete reading
 * 
 */
int LidarController::refreshLidarReading(void)
{
	this->reading = {0}; // Shouldn't be required since reset on new request

	// Bring drivetrain to a halt
	this->driveController->demandHalt();

	// Ping lidar
	int lidarResult = this->lidar->requestReading(&reading);
	
	// Clear unaddressed request on successful ping
	if (lidarResult == LIDAR_READING_SUCCESS)
		this->hasUnaddressedRequest = false;

	return lidarResult;
}

/**
 * @brief Determine if Lidar should accept new readings requests.
 * 
 */
bool LidarController::shouldAcceptNewRequests(void)
{
	unsigned long timeSinceLastCompleteSentMillis = millis() - this->lastCompleteSentTimestampMillis;
	return (
		(
			// Not currently procesing a request
			(false == this->hasUnaddressedRequest) &&

			// Enough time has elapsed since last request
			(timeSinceLastCompleteSentMillis > LIDAR_MIN_TIME_SINCE_HALT_RECEIVED_COMMAND)
		) ||
		(
			// Last reading is lost
			(timeSinceLastCompleteSentMillis > LIDAR_MAX_TIME_TO_SEND_READING)
		)
	);
}

/**
 * @brief Determine if Lidar should be pinged
 * 
 * @return true 
 * @return false 
 */
bool LidarController::shouldRefreshLidarReading(void)
{
	return (
		// Addressing a request
		this->hasUnaddressedRequest &&

		// Reading has been cleared
		isLidarReadingEmpty(&(this->reading))
	);
}

/**
 * @brief Determine if Lidar data should be sent
 * 
 * @return true 
 * @return false 
 */
bool LidarController::shouldSendLidarReading(void)
{
	return (
		// Reading points yet to be sent
		(false == isLidarReadingFullyProcessed(&(this->reading)))
	);
}

/**
 * @brief Send Lidar data points
 * 
 */
void LidarController::sendLidarData(void)
{
	Message message;
	bool messageQueueMayHaveSpace = true;
	while (
		messageQueueMayHaveSpace &&
		(false == isLidarReadingFullyProcessed(&(this->reading)))
	)
	{
		// Get next Lidar data to send
		stepLidarReadingToNextValidPoint(&(this->reading));
		LidarPointReading* currentPoint = getCurrentLidarPointReading(&(this->reading));

		// Construct message
		LidarPointReadingTranslation.asMessage(currentPoint, &message);

		// Post message
		ControllerMessageQueueOutput result = this->post(&message);

		// Mark as complete if successfully enqueued
		if (result == ControllerMessageQueueOutput::EnqueueSuccess)
		{
			markLidarReadingIndexProcessed(&(this->reading));
		}
		else
		{
			messageQueueMayHaveSpace = false;
		}
	}

	// If all data sent, mark as complete
	if (isLidarReadingFullyProcessed(&(this->reading)))
		lastCompleteSentTimestampMillis = millis();
}

/**
 * @brief Send encoder information upon request or periodically
 * 
 */
void LidarController::process(void)
{
	// Monitor incoming messages
	this->checkLidarRequest();

	// Ping lidar if required
	if (this->shouldRefreshLidarReading())
	{
		this->refreshLidarReading();
	}

	// Send as much Lidar infromation as possible
	if (this->shouldSendLidarReading())
	{
		this->sendLidarData();
	}
}