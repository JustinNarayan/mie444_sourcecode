#pragma once
#include "MemoryUtilities.h"
#include <CommsInterface.h>
#include <Translate.h>

class PeripheralEnvoy
{
private:
	CommsInterface* toSend;

	/**
	 * @brief Envoy message on comms interface
	 * 
	 */
	void envoy(Message *message)
	{
		this->toSend->sendMessage(message);
	}

public:
	/**
	 * @brief Construct a new CommsEnvoy
	 * 
	 * @param toReceive Channel to send messages on
	 */
	PeripheralEnvoy(CommsInterface* toSend) : toSend(toSend) {};

	/**
	 * @brief Envoy encoder request on comms interface
	 *
	 */
	void envoyEncoderRequest(void)
	{
		Message message;
		DrivetrainEncoderStateTranslation.asMessage(
			DrivetrainEncoderState::Request, 
			&message
		);
		this->envoy(&message);
	}

	/**
	 * @brief Envoy drivetrain manual command on comms interface
	 *
	 */
	void envoyDrivetrainManualCommand(DrivetrainManualCommand command)
	{
		Message message;
		DrivetrainManualCommandTranslation.asMessage(
			command, 
			&message
		);
		this->envoy(&message);
	}
};