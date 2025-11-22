#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>
#include <Drivetrain.h>
#include "DriveEncoderController.h"
#include "DrivetrainDefs.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInDrive = MessageTypes<
    MessageType::DrivetrainManualCommand, // Manual commands
    MessageType::DrivetrainAutomatedCommand // Automated commands
>;
using MessageTypesOutDrive = MessageTypes<
    MessageType::DrivetrainManualResponse, // Manual command responses
    MessageType::DrivetrainAutomatedResponse // Automated command responses
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class DriveController : public Controller<
	MessageTypesInDrive, 
	MessageTypesOutDrive
>
{
private:
	/**
	 * Reference to Drivetrain and DriveEncoderControllers
	 */
	Drivetrain* drivetrain;
    DriveEncoderController* encoders;

	/**
	 * Manual commands
	 */
	time_ms lastReceivedValidManualCommandTime;
	DrivetrainManualCommand lastIssuedCommand;

    /**
     * Automated commands
     * 
     */
    DrivetrainAutomatedResponse automatedCommandState;
	DrivetrainAutomatedCommand currentAutomatedCommand;
    bool hasUnaddressedAutomatedCommand;
	time_ms lastIssuedAutomatedCommandTime;
    DrivetrainEncoderDistances encodersStart;
    DrivetrainEncoderDistances encodersTarget;
    DrivetrainCommandDirection automatedCommandDirection;
    time_ms lastControlAdjustmentTime;
    bool targetReached;
    time_ms targetReachedTime;

	/**
	 * @brief Communication utilities
	 */
	DrivetrainManualCommand getDrivetrainManualCommand(void);
	void sendDrivetrainManualResponse(DrivetrainManualResponse response);
	void checkDrivetrainAutomatedCommand(void);
	void sendDrivetrainAutomatedResponse(DrivetrainAutomatedResponse response);
	void sendDrivetrainDisplacements(DrivetrainDisplacements *displacements);
	void sendDrivetrainMotorCommand(DrivetrainMotorCommand *command);

	/**
	 * @brief Process utilities
	 */
	void processManualCommand(DrivetrainManualCommand command);
	void applyManualCommand(DrivetrainManualCommand command);
    void initializeAutomatedCommand(void);
	void applyAutomatedCommand(bool isFirstApplicationOfCommand);
    bool isAutomatedCommandRunning(void);
    bool isAutomatedCommandSuccessful(void);
    void monitorAutomatedCommand(void);
    void clearAutomatedCommand(void);
    void arbitrateCommands(DrivetrainManualCommand currentCommand);
	bool shouldHalt(void);
	void voluntaryHalt(bool startWithBrake = false);
	void voluntaryBrake(void);
public:
	DriveController(Drivetrain* drivetrain, DriveEncoderController* encoders);
	void process(void);
};