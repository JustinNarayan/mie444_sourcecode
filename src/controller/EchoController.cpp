#include "EchoController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new Echo Controller
 * 
 */
EchoController::EchoController(void) {}

/**
 * @brief Read input messages for a MessageType and echo it back out
 * 
 */
void EchoController::echoMessage(MessageType type)
{
	// Dequeue Message
	Message message;
	ControllerMessageQueueOutput ret = this->read(type, &message);

	// Echo if received
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
		this->post(&message);
}

/**
 * @brief Echo all received Message objects
 * 
 */
void EchoController::process(void)
{
	echoMessage(MessageType::DrivetrainEncoderRequest);
	echoMessage(MessageType::DrivetrainEncoderDistances);
	echoMessage(MessageType::Generic);
}