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
 * @brief Try to send a single Lidar point reading.
 * 
 * @return Whether post was successful. If false, messages out queue was likely full
 */
bool LidarController::trySendNextLidarReadingPoint(void)
{
	// Find next point to send
	stepLidarReadingToNextValidPoint(&(this->reading));
	LidarPointReading* currentPoint = getCurrentLidarPointReading(&(this->reading));
	
	// Construct message
	Message message;
	LidarPointReadingTranslation.asMessage(currentPoint, &message);
	
	// Post message
	ControllerMessageQueueOutput result = this->post(&message);

	// Notify if post was unsuccessful
	if (result != ControllerMessageQueueOutput::EnqueueSuccess)
		return false;

	// Mark point as processed
	markLidarReadingIndexProcessed(&(this->reading));
	return true;
}

/**
 * @brief Notify that all Lidar information has been sent
 * 
 */
void LidarController::sendLidarComplete(void)
{
	Message message;
	message.init(MessageType::LidarComplete, MESSAGE_CONTENT_SIZE_AUTOMATIC, "");
	this->post(&message);
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
		messageQueueMayHaveSpace = this->trySendNextLidarReadingPoint();
	}

	// If all data sent, mark as complete
	if (isLidarReadingFullyProcessed(&(this->reading)))
	{
		lastCompleteSentTimestampMillis = millis();
		this->sendLidarComplete();
	}
}

/**
 * @brief Determine if Lidar should request prioiritzed sender. Do so if a request has been 
 * received but the data is not stale.
 * 
 * @return true 
 * @return false 
 */
bool LidarController::shouldRequestPrioritizedSender(void)
{
	return (
		(false == shouldAcceptNewRequests())
	);
}

/**
 * @brief Send lidar information upon request
 * 
 */
void LidarController::process(void)
{
	// Monitor incoming messages
	this->checkLidarRequest();

	// Check if should request prioritized sender
	if (this->shouldRequestPrioritizedSender())
	{
		this->setRequestPrioritizedSender();
	}
	else
	{
		this->clearRequestPrioritizedSender();
	}

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