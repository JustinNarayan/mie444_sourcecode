#include "DriveEncoderController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new DriveEncoderController
 * 
 * @param drivetrainEncoders
 */
DriveEncoderController::DriveEncoderController(DrivetrainEncoders* drivetrainEncoders) :
	drivetrainEncoders(drivetrainEncoders),
	hasUnaddressedRequestFromLocalization(false),
	lastSentToLocalizationTimestampMillis(0),
	hasUnaddressedRequestFromController(false),
	lastSentToControllerTimestampMillis(0) {}

/**
 * @brief Read input messages for DrivetrainEncoderState type
 * 
 */
void DriveEncoderController::checkDrivetrainEncoderState(void)
{
	// Dequeue DrivetrainEncoderState
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderState, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		// Decode which party requested message
		DrivetrainEncoderState state = DrivetrainEncoderStateTranslation.asEnum(&message);
		if (state == DrivetrainEncoderState::RequestFromLocalization)
			this->hasUnaddressedRequestFromLocalization = true;
		else if (state == DrivetrainEncoderState::RequestFromController)
			this->hasUnaddressedRequestFromController = true;
	}
}

/**
 * @brief Ping drivetrain for present encoder values and convert to appropriate distances
 * 
 * @param distances 
 */
void DriveEncoderController::getDrivetrainEncoderDistances(DrivetrainEncoderDistances* distances, bool isForLocalization)
{
	long distance1, distance2, distance3;
	this->drivetrainEncoders->getCurrentDistances(&distance1, &distance2, &distance3);

	distances->isForLocalization = isForLocalization;
	distances->encoder1Dist_in = (float32_t)distance1 * ENCODER_1_TO_IN;
	distances->encoder2Dist_in = (float32_t)distance2 * ENCODER_2_TO_IN;
	distances->encoder3Dist_in = (float32_t)distance3 * ENCODER_3_TO_IN;
}

/**
 * @brief Send current drivetrain encoder distances
 * 
 */
void DriveEncoderController::sendDrivetrainEncoderDistances(bool isForLocalization)
{
	DrivetrainEncoderDistances distances;
	this->getDrivetrainEncoderDistances(&distances, isForLocalization);

	Message message;
	DrivetrainEncoderDistancesTranslation.asMessage(&distances, &message);
	ControllerMessageQueueOutput result = this->post(&message);

	if (result == ControllerMessageQueueOutput::EnqueueSuccess)
	{
		// Indicate message was sent
		if (isForLocalization)
		{
			this->hasUnaddressedRequestFromLocalization = false;
			this->lastSentToLocalizationTimestampMillis = millis();
		}
		else
		{
			this->hasUnaddressedRequestFromController = false;
			this->lastSentToControllerTimestampMillis = millis();
		}
	}
}

/**
 * @brief Determine if drivetrain should send. If no reading has been sent recently, send.
 * 
 */
bool DriveEncoderController::shouldSend(bool isForLocalization)
{
	bool hasUnaddressedRequest = isForLocalization ? 
		this->hasUnaddressedRequestFromLocalization : 
		this->hasUnaddressedRequestFromController;
	unsigned long lastSentTimestampMillis = isForLocalization ?
		this->hasUnaddressedRequestFromLocalization :
		this->hasUnaddressedRequestFromController;

	return (
		// Controller board has requested
		hasUnaddressedRequest ||

		// No reading has been recently sent and encoder volunteers readings
		(
			ENCODER_WILL_VOLUNTEER_READINGS &&
			((millis() - lastSentTimestampMillis) > ENCODER_TIME_TO_SEND_AFTER_LAST_SENT_DISTANCES)
		)
	);
}

/**
 * @brief Send encoder information upon request or periodically
 * 
 */
void DriveEncoderController::process(void)
{
	this->checkDrivetrainEncoderState();

	// Sending for localization
	bool isForLocalization = true;
	if (this->shouldSend(isForLocalization))
		this->sendDrivetrainEncoderDistances(isForLocalization);

	// Sending for controller
	isForLocalization = false;
	if (this->shouldSend(isForLocalization))
		this->sendDrivetrainEncoderDistances(isForLocalization);
}