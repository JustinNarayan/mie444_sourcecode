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
	hasUnaddressedRequest(false),
	lastSentTimestampMillis(0) {}

/**
 * @brief Read input messages for DrivetrainEncoderRequest type
 * 
 */
void DriveEncoderController::checkDrivetrainEncoderRequest(void)
{
	// Dequeue DrivetrainEncoderRequest
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderRequest, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		this->hasUnaddressedRequest = true;

		// Eliminate stale duplicate messages
		this->purge(MessageType::DrivetrainEncoderRequest);
	}
}

/**
 * @brief Ping drivetrain for present encoder values and convert to appropriate distances
 * 
 * @param distances 
 */
void DriveEncoderController::getDrivetrainEncoderDistances(DrivetrainEncoderDistances* distances)
{
	long distance1, distance2, distance3;
	this->drivetrainEncoders->getCurrentDistances(&distance1, &distance2, &distance3);

	distances->encoder1Dist_in = (float32_t)distance1 * ENCODER_1_TO_IN;
	distances->encoder2Dist_in = (float32_t)distance2 * ENCODER_2_TO_IN;
	distances->encoder3Dist_in = (float32_t)distance3 * ENCODER_3_TO_IN;
}

/**
 * @brief Send current drivetrain encoder distances
 * 
 */
void DriveEncoderController::sendDrivetrainEncoderDistances(void)
{
	DrivetrainEncoderDistances distances;
	this->getDrivetrainEncoderDistances(&distances);

	Message message;
	DrivetrainEncoderDistancesTranslation.asMessage(&distances, &message);
	this->post(&message);

	// Indicate message was sent
	this->hasUnaddressedRequest = false;
	this->lastSentTimestampMillis = millis();
}

/**
 * @brief Determine if drivetrain should send. If no reading has been sent recently, send.
 * 
 */
bool DriveEncoderController::shouldSend(void)
{
	return (
		// Controller board has requested
		this->hasUnaddressedRequest ||

		// No reading has been recently sent and encoder volunteers readings
		(
			ENCODER_WILL_VOLUNTEER_READINGS &&
			((millis() - this->lastSentTimestampMillis) > ENCODER_TIME_TO_SEND_AFTER_LAST_SENT_DISTANCES)
		)
	);
}

/**
 * @brief Send encoder information upon request or periodically
 * 
 */
void DriveEncoderController::process(void)
{
	this->checkDrivetrainEncoderRequest();
	if (this->shouldSend())
		this->sendDrivetrainEncoderDistances();
}