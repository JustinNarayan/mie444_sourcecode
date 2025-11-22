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
	MessageType::DrivetrainManualCommand, // Commands for drivetrain
    MessageType::DrivetrainAutomatedCommand // Automated commands for drivetrain
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
	 * @brief Current DrivetrainAutomatedCommand to send
	 * 
	 */
	DrivetrainAutomatedCommand drivetrainAutomatedCommand;
    bool hasUnforwardDrivetrainAutomatedCommand;
	time_ms drivetrainAutomatedCommandLastReceivedTime;

    /**
     * @brief Allow other controllers to override supplied manual and automatic commands,
     * such as during a circular ultrasonic ping
     * 
     */
    bool blockingExternalDrivetrainCommands;
    bool hasInternalDrivetrainCommand;
    DrivetrainAutomatedCommand internalDrivetrainCommand;

	/**
	 * @brief Encoder utilities
	 */
	void checkEncoderState(void);
	void envoyEncoderRequest(void);
	bool shouldEnvoyEncoderRequest(void);

	/**
	 * @brief Drivetrain manual command utilities
	 */
	void checkDrivetrainManualCommand(void);
	void envoyDrivetrainManualCommand(void);
	bool shouldEnvoyDrivetrainManualCommand(void);
	bool validateDrivetrainManualCommandFresh(void);
    
	/**
	 * @brief Drivetrain automated command utilities
	 */
	void checkDrivetrainAutomatedCommand(void);
	void envoyDrivetrainAutomatedCommand(void);
	bool shouldEnvoyDrivetrainAutomatedCommand(void);

    /**
     * @brief Internal drivetrain control interface
     * 
     */
    void envoyInternalDrivetrainCommand(void);
    bool shouldEnvoyInternalDrivetrainCommand(void);
public:
	PeripheralForwardingController(PeripheralEnvoy* envoy);
    void blockExternalDrivetrainCommands(bool shouldBlock);
    void provideInternalDrivetrainCommand(DrivetrainAutomatedCommand* command);
	void process(void);
};