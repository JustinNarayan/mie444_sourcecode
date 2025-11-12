#pragma once
#include <CommsInterface.h>
#include <Controller.h>
#include <DrivetrainEncoders.h>
#include "DrivetrainDefs.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInDriveEncoder = MessageTypes<
    MessageType::DrivetrainEncoderState // Request an encoder reading
>;
using MessageTypesOutDriveEncoder = MessageTypes<
    MessageType::DrivetrainEncoderDistances // Encoder reading
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class DriveEncoderController : public Controller<
	MessageTypesInDriveEncoder, 
	MessageTypesOutDriveEncoder
>
{
private:
	/**
	 * Reference to Drivetrain Encoders
	 */
	DrivetrainEncoders* drivetrainEncoders;

	/**
	 * Whether a request was received and not yet addressed
	 */
	bool hasUnaddressedRequestFromLocalization;
	unsigned long lastSentToLocalizationTimestampMillis;
	bool hasUnaddressedRequestFromController;
	unsigned long lastSentToControllerTimestampMillis;

	/**
	 * @brief Low-level utilities
	 * 
	 */
	void getDrivetrainEncoderDistances(DrivetrainEncoderDistances* distances, bool isForLocalization);
	
	/**
	 * @brief Communication utilities
	 */
	void checkDrivetrainEncoderState(void);
	void sendDrivetrainEncoderDistances(bool isForLocalization);

	/**
	 * @brief Proces utilities
	 * 
	 */
	bool shouldSend(bool isForLocalization);
public:
	DriveEncoderController(DrivetrainEncoders* drivetrainEncoders);
	void process(void);
};