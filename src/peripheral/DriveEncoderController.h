#pragma once
#include <CommsInterface.h>
#include <Controller.h>
#include <DrivetrainEncoders.h>
#include "DrivetrainDefs.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesIn = MessageTypes<
    MessageType::DrivetrainEncoderRequest // Request an encoder reading
>;
using MessageTypesOut = MessageTypes<
    MessageType::DrivetrainEncoderDistances, // Encoder reading
    MessageType::Error // Errors
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class DriveEncoderController : public Controller<MessageTypesIn, MessageTypesOut>
{
private:
	/**
	 * Reference to Drivetrain Encoders
	 */
	DrivetrainEncoders* drivetrainEncoders;

	/**
	 * Whether a request was received and not yet addressed
	 */
	bool hasUnaddressedRequest;
	unsigned long lastSentTimestampMillis;

	/**
	 * @brief Low-level utilities
	 * 
	 */
	void getDrivetrainEncoderDistances(DrivetrainEncoderDistances* distances);
	
	/**
	 * @brief Communication utilities
	 */
	void checkDrivetrainEncoderRequest(void);
	void sendDrivetrainEncoderDistances(void);

	/**
	 * @brief Proces utilities
	 * 
	 */
	bool shouldSend(void);
public:
	DriveEncoderController(DrivetrainEncoders* drivetrainEncoders);
	void process(void);
};