#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInEcho = MessageTypes<
	MessageType::DrivetrainEncoderRequest, // Encoder requests
	MessageType::DrivetrainEncoderDistances, // Encoder outputs
    MessageType::Generic // Debug messages
>;
using MessageTypesOutEcho = MessageTypes<
	MessageType::DrivetrainEncoderRequest, // Encoder requests
	MessageType::DrivetrainEncoderDistances, // Encoder outputs
    MessageType::Generic // Debug messages
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class EchoController : public Controller<
	MessageTypesInEcho, 
	MessageTypesOutEcho
>
{
private:
	/**
	 * @brief Communication utilities
	 */
	void echoMessage(MessageType type);
public:
	EchoController(void);
	void process(void);
};