#include "EncoderRequestController.h"

/**
 * @brief Construct a new EncoderRequestController
 * 
 * @param envoy To send encoder requests to peripheral board
 */
EncoderRequestController::EncoderRequestController(CommsEnvoy* envoy) :
	envoy(envoy) {}

/**
 * @brief Read input messages for DrivetrainEncoderRequest type
 * 
 */
void EncoderRequestController::checkEncoderRequest(void)
{
	// Dequeue DrivetrainEncoderRequest
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderRequest, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		this->hasUnaddressedRequest = true;

		// Eliminate stale messages
		this->purge(MessageType::DrivetrainEncoderRequest);
	}
}

/**
 * @brief Ping encoder for a reading
 * 
 */
void EncoderRequestController::envoyRequest(void)
{
	Message message;
	message.init(MessageType::DrivetrainEncoderRequest, MESSAGE_CONTENT_SIZE_AUTOMATIC, "");
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
	message.init(MessageType::DrivetrainEncoderPinged, MESSAGE_CONTENT_SIZE_AUTOMATIC, "");
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
	this->checkEncoderRequest();

	// Check if should ping encoders
	if (this->shouldEnvoyRequest())
	{
		this->envoyRequest();
		this->sendEncoderPinged();
	}
}