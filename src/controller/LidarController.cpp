#include "LidarController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new LidarController
 * 
 * @param lidar
 * @param driveController To request a full halt before pinging Lidar
 */
LidarController::LidarController(Lidar* lidar, PeripheralEnvoy* envoy) :
	lidar(lidar),
	envoy(envoy),
	reading({0}),
	hasUnaddressedRequest(false),
	lastCompleteSentTimestampMillis(0),
	hasNotSentComplete(false) {}

/**
 * @brief Read input messages for LidarRequest type
 * 
 */
void LidarController::checkLidarState(void)
{
	// Dequeue LidarState
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::LidarState, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		// Confirm message is a request, so it's valid
		if (LidarStateTranslation.asEnum(&message) != LidarState::Request)
			return;

		// Determine if request is valid
		if (this->shouldAcceptNewRequests())
		{
			// Accept
			this->hasUnaddressedRequest = true;
			this->reading = {0};
		}
		else
		{
			// Reject
			this->sendLidarState(LidarState::Rejected);
		}

		// Eliminate stale requests
		this->purge(MessageType::LidarState);
	}
}

/**
 * @brief Ping lidar for a complete reading
 * 
 */
void LidarController::refreshLidarReading(void)
{
	this->reading = {0}; // Shouldn't be required since reset on new request

	// Bring drivetrain to a halt
	this->envoy->envoyDrivetrainManualCommand(DrivetrainManualCommand::Halt);

	// Ping lidar
	LidarState result = this->lidar->requestReading(&reading);
	
	// Clear unaddressed request on successful ping
	if (result == LidarState::Success)
		this->hasUnaddressedRequest = false;

	// Send state
	this->sendLidarState(result);
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
 * @brief Notify current Lidar state
 * 
 */
void LidarController::sendLidarState(LidarState state)
{
	Message message;
	LidarStateTranslation.asMessage(state, &message);
	ControllerMessageQueueOutput result = this->post(&message);
	
	if (state == LidarState::Complete)
		if (result == ControllerMessageQueueOutput::EnqueueSuccess)
			this->hasNotSentComplete = false;
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
		this->lastCompleteSentTimestampMillis = millis();
		this->hasNotSentComplete = true;
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
	this->checkLidarState();

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

	// Notify complete if required
	if (this->hasNotSentComplete == true)
	{
		this->sendLidarState(LidarState::Complete);
	}
}