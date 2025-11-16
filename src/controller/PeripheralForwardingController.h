#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>
#include "PeripheralEnvoy.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInForwarding = MessageTypes<
    MessageType::DrivetrainEncoderState, // Request for Encoder readings,
	MessageType::DrivetrainManualCommand // Commands for drivetrain
>;
using MessageTypesOutForwarding = MessageTypes<>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class PeripheralForwardingController : public Controller<
	MessageTypesInForwarding, 
	MessageTypesOutForwarding
>
{
private:
	/**
	 * Reference to Peripheral Comms Envoy
	 */
	PeripheralEnvoy* envoy;

	/**
	 * Whether an encoder request was received and not yet addressed
	 */
	bool hasUnaddressedEncoderRequest;

	/**
	 * @brief Current DrivetrainManualCommand to send
	 * 
	 */
	DrivetrainManualCommand drivetrainManualCommand;
	time_ms drivetrainManualCommandLastReceivedTime;

	/**
	 * @brief Encoder utilities
	 */
	void checkEncoderState(void);
	void envoyEncoderRequest(void);
	bool shouldEnvoyRequest(void);

	/**
	 * @brief Drivetrain manual command utilities
	 */
	void checkDrivetrainManualCommand(void);
	void envoyDrivetrainManualCommand(void);
	bool shouldEnvoyDrivetrainManualCommand(void);
	bool validateDrivetrainManualCommandFresh(void);
public:
	PeripheralForwardingController(PeripheralEnvoy* envoy);
	void process(void);
};