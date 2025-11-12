#include "EncoderRequestController.h"
#include <Translate.h>

/**
 * @brief Construct a new EncoderRequestController
 * 
 * @param envoy To send encoder requests to peripheral board
 */
EncoderRequestController::EncoderRequestController(CommsEnvoy* envoy) :
	envoy(envoy) {}

/**
 * @brief Read input messages for DrivetrainEncoderState type
 * 
 */
void EncoderRequestController::checkEncoderState(void)
{
	// Dequeue DrivetrainEncoderState
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderState, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		// This controller only handles only requests for localization
		if (
			DrivetrainEncoderStateTranslation.asEnum(&message) != DrivetrainEncoderState::RequestFromLocalization
		)
			return;

		this->hasUnaddressedRequest = true;

		// Eliminate stale messages
		this->purge(MessageType::DrivetrainEncoderState);
	}
}

/**
 * @brief Ping encoder for a reading
 * 
 */
void EncoderRequestController::envoyRequest(void)
{
	Message message;
	DrivetrainEncoderStateTranslation.asMessage(
		DrivetrainEncoderState::RequestFromLocalization, 
		&message
	);
	this->envoy->envoy(&message);
	this->hasUnaddressedRequest = false;
}

/**
 * @brief Indicate encoder has been pinged
 * 
 */
void EncoderRequestController::sendEncoderPinged(void)
{
	Message message;
	DrivetrainEncoderStateTranslation.asMessage(DrivetrainEncoderState::Pinged, &message);
	this->post(&message);
}

/**
 * @brief Determine if encoder should be pinged
 * 
 * @return true 
 * @return false 
 */
bool EncoderRequestController::shouldEnvoyRequest(void)
{
	return (
		// Addressing a request
		this->hasUnaddressedRequest
	);
}

/**
 * @brief Send encoder information upon request or periodically
 * 
 */
void EncoderRequestController::process(void)
{
	// Monitor incoming messages
	this->checkEncoderState();

	// Check if should ping encoders
	if (this->shouldEnvoyRequest())
	{
		this->envoyRequest();
		this->sendEncoderPinged();
	}
}