#include "DriveController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new DriveController
 * 
 * @param drivetrain
 */
DriveController::DriveController(Drivetrain* drivetrain, DriveEncoderController* encoders) : 
	drivetrain(drivetrain), 
	encoders(encoders),
	lastReceivedValidManualCommandTime(0),
	lastIssuedCommand(DrivetrainManualCommand::Halt),
	automatedCommandState(DrivetrainAutomatedResponse::NoReceived),
	currentAutomatedCommand({0}),
	hasUnaddressedAutomatedCommand(false),
	lastIssuedAutomatedCommandTime(0),
	encodersStart({0}),
	encodersTarget({0}),
	automatedCommandDirection({0}) {}

/**
 * @brief Read input messages for DrivetrainManualCommand type
 * 
 * @return DrivetrainManualCommands received command, including NoReceivedCommand and InvalidCommand
 */
DrivetrainManualCommand DriveController::getDrivetrainManualCommand(void)
{
	// Dequeue DrivetrainManualCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainManualCommand, &message);

	// Translate DrivetrainManualCommand
	DrivetrainManualCommand received = DrivetrainManualCommand::NoReceived;
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		received = DrivetrainManualCommandTranslation.asEnum(&message);

		// Purge stale messages
		this->purge(MessageType::DrivetrainManualCommand);
	}
	return received;
}

/**
 * @brief Read input messages for DrivetrainAutomatedCommand type.
 *  If no automated command is in progress, store the new command and flag it as
 *  unaddressed. On any received command, purge the queue to eliminate stale messages.
 * 
 * @return DrivetrainAutomatedCommands received command
 */
void DriveController::checkDrivetrainAutomatedCommand(void)
{
	// Dequeue DrivetrainManualCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainAutomatedCommand, &message);

	// Discard new automated commands if current command is in progress
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		if (this->automatedCommandState != DrivetrainAutomatedResponse::InProgress)
		{
			DrivetrainAutomatedCommandTranslation.asStruct(
				&message, 
				&this->currentAutomatedCommand
			);
			this->hasUnaddressedAutomatedCommand = true;
		}

		// Purge stale commands
		this->purge(MessageType::DrivetrainAutomatedCommand);
	}
}

/**
 * @brief Write DrivetrainManualResponse
 * 
 * @param response To write
 */
void DriveController::sendDrivetrainManualResponse(DrivetrainManualResponse response)
{
	Message message;
	DrivetrainManualResponseTranslation.asMessage(response, &message);
	this->post(&message);
}

/**
 * @brief Write DrivetrainAutomatedResponse
 * 
 * @param response To write
 */
void DriveController::sendDrivetrainAutomatedResponse(DrivetrainAutomatedResponse response)
{
	Message message;
	DrivetrainAutomatedResponseTranslation.asMessage(response, &message);
	this->post(&message);
}

/**
 * @brief Write DrivetrainDisplacements
 * 
 * @param response To write
 */
void DriveController::sendDrivetrainDisplacements(DrivetrainDisplacements *displacements)
{
	Message message;
	DrivetrainDisplacementsTranslation.asMessage(displacements, &message);
	this->post(&message);
}

/**
 * @brief Write DrivetrainMotorCommand
 * 
 * @param response To write
 */
void DriveController::sendDrivetrainMotorCommand(DrivetrainMotorCommand *command)
{
	Message message;
	DrivetrainMotorCommandTranslation.asMessage(command, &message);
	this->post(&message);
}

/**
 * @brief Process manual command to drivetrain and record it.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::processManualCommand(DrivetrainManualCommand command)
{
	// Record command as valid received
	this->lastReceivedValidManualCommandTime = millis();

	// Apply command if not a continued command and record command as issued
	if (command != this->lastIssuedCommand)
	{
		this->applyManualCommand(command);
		this->lastIssuedCommand = command;
	}
}

/**
 * @brief Apply command to drivetrain.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::applyManualCommand(DrivetrainManualCommand command)
{
	switch (command)
	{
		case DrivetrainManualCommand::TranslateForward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, true);
			break;
		case DrivetrainManualCommand::TranslateBackward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, false);
			break;
		case DrivetrainManualCommand::RotateLeft:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, true);
			break;
		case DrivetrainManualCommand::RotateRight:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, false);
			break;
        case DrivetrainManualCommand::Brake:
            drivetrain->setBrake();
            break;
		case DrivetrainManualCommand::Halt:
		default:
			drivetrain->halt();
			break;
	}
}

/**
 * @brief Initialize a drivetrain automated command.
 * 
 */
void DriveController::initializeAutomatedCommand(void)
{
    // Convert command to displacements
    DrivetrainDisplacements displacements;
    displacementsFromDrivetrainCommand(&displacements, &(this->currentAutomatedCommand));

	// Process automated command state
	this->encoders->getDrivetrainEncoderDistances(&this->encodersStart);
	encoderReadingsFromDisplacement(
		&displacements,
		&(this->encodersStart),
		&(this->encodersTarget) // Now holds target values
	);
	commandDirectionFromEncoderReadings(
		&(this->encodersStart),
		&(this->encodersTarget),
		&(this->automatedCommandDirection) // Now holds direction
	);

	// Issue command to drivetrain
	this->applyAutomatedCommand(true); // is first application

	// Mark request as addressed
	this->automatedCommandState = DrivetrainAutomatedResponse::InProgress;
	this->hasUnaddressedAutomatedCommand = false;
	this->lastIssuedAutomatedCommandTime = this->lastControlAdjustmentTime;

	// Notify command is acknowledged
	this->sendDrivetrainAutomatedResponse(DrivetrainAutomatedResponse::Acknowledge);
}

/**
 * @brief Issue a drivetrain automated command.
 * 
 */
void DriveController::applyAutomatedCommand(bool isFirstApplicationOfCommand)
{	
#if DRIVETRAIN_WILL_USE_SIMPLE_AUTOMATED_INTERFACE
    {
        // Prioritize rotation
        if (fabsf(this->currentAutomatedCommand.dTheta_deg) > DRIVETRAIN_AUTOMATED_MIN_DELTA_THETA)
        {
            this->drivetrain->setRotate(
                DRIVETRAIN_ROTATE_SPEED_AUTOMATED, (this->currentAutomatedCommand.dTheta_deg < 0)
            );
        }
        // Then translation
        else if (fabsf(this->currentAutomatedCommand.dX_in) > DRIVETRAIN_AUTOMATED_MIN_DELTA_X)
        {
            this->drivetrain->setTranslate(
                DRIVETRAIN_TRANSLATE_SPEED_AUTOMATED, (this->currentAutomatedCommand.dX_in > 0)
            );
        }
        // Then strafing
        else if (fabsf(this->currentAutomatedCommand.dY_in) > DRIVETRAIN_AUTOMATED_MIN_DELTA_Y)
        {
            this->drivetrain->setStrafe(
                DRIVETRAIN_STRAFE_SPEED_AUTOMATED, (this->currentAutomatedCommand.dY_in > 0)
            );
        }
    }
#else
    {
        // Get current encoder readings
        DrivetrainEncoderDistances encodersNow;
        this->encoders->getDrivetrainEncoderDistances(&encodersNow);

        // Construct drivetrain command
        DrivetrainMotorCommand command;
        getDrivetrainMotorCommand(
            &encodersNow,
            &this->encodersTarget,
            &command,
            isFirstApplicationOfCommand
        );
        
        // Send command to drivetrain
        drivetrain->setMotors(&command);

# if DRIVETRAIN_WILL_VOLUNTEER_AUTOMATED_COMMANDS
        // Send motor commands
        this->sendDrivetrainMotorCommand(&command);
# endif
    }
#endif

	// Record the command as issued
	this->lastIssuedCommand = DrivetrainManualCommand::Automated;

    // Record control as updated
    this->lastControlAdjustmentTime = millis();
}

/**
 * @brief Monitor a drivetrain automated command to determine when it's complete.
 * 
 */
void DriveController::monitorAutomatedCommand(void)
{
	// Check if command has timed out
	if (
		(millis() - this->lastIssuedAutomatedCommandTime) > DRIVETRAIN_AUTOMATED_COMMAND_MAX_TIME
	)
	{
		this->voluntaryHalt();
		this->clearAutomatedCommand();
		return;
	}

	// Update encoder readings
	DrivetrainEncoderDistances encodersNow;
	this->encoders->getDrivetrainEncoderDistances(&encodersNow);

	// Check if at target
	this->automatedCommandState = areEncodersAtTarget(
		&(this->automatedCommandDirection),
		&encodersNow,
		&(this->encodersTarget)
	);
	
	// Exit command if successful or failure
	if (this->automatedCommandState != DrivetrainAutomatedResponse::InProgress)
	{
		this->voluntaryHalt();
		this->sendDrivetrainAutomatedResponse(this->automatedCommandState);
        this->clearAutomatedCommand();
	}
}


/**
 * @brief Abort the current automated command, but do not issue a drivetrain halt command.
 * 
 */
void DriveController::clearAutomatedCommand(void)
{
    // Send net displacements
#if (DRIVETRAIN_WILL_VOLUNTEER_AUTOMATED_DISPLACEMENTS)
    delay(DRIVETRAIN_AUTOMATED_POST_COMMAND_DISPLACEMENT_WAIT); // reduce decode errors on encoders
    DrivetrainEncoderDistances encodersNow;
    this->encoders->getDrivetrainEncoderDistances(&encodersNow);
    DrivetrainDisplacements displacements;
    displacementsFromEncoderReadings(
        &displacements,
        &this->encodersStart,
        &encodersNow
    );
    this->sendDrivetrainDisplacements(&displacements);
#endif

    // Abort the command and notify
	if (this->automatedCommandState == DrivetrainAutomatedResponse::InProgress)
	{
		this->automatedCommandState = DrivetrainAutomatedResponse::Aborted;
        this->sendDrivetrainAutomatedResponse(this->automatedCommandState);
    }

    // Clear the state and set the state back to none received
    this->currentAutomatedCommand = {0};
    this->hasUnaddressedAutomatedCommand = false;
    this->encodersStart = {0};
    this->encodersTarget = {0};
    this->automatedCommandDirection = {0};
    this->automatedCommandState = DrivetrainAutomatedResponse::NoReceived;
}

/**
 * @brief Receive commands and issue to Drivetrain. Send back an appropriate acknowledge.
 * 
 */
void DriveController::arbitrateCommands(DrivetrainManualCommand currentManualCommand)
{
	// Process a manual command if received
	if (
		(currentManualCommand != DrivetrainManualCommand::NoReceived) &&
		(currentManualCommand != DrivetrainManualCommand::Invalid)
	)
	{
        if (this->lastIssuedCommand == DrivetrainManualCommand::Automated)
        {
		    this->clearAutomatedCommand();
        }
		this->processManualCommand(currentManualCommand);
		this->sendDrivetrainManualResponse(DrivetrainManualResponse::Acknowledge);
		return;
	}

	// Initialize automated command if received
	if (this->hasUnaddressedAutomatedCommand)
	{
        // Initialize command
		this->initializeAutomatedCommand();
		return;
	}

	// Monitor progress of automated command
	if (this->automatedCommandState == DrivetrainAutomatedResponse::InProgress)
	{
        // Update command with control algorithm
        if (
            (millis() - this->lastControlAdjustmentTime) > DRIVETRAIN_MINIMUM_TIME_BETWEEN_CONTROL_ADJUSTEMENTS
        )
        {
            this->applyAutomatedCommand(false); // is not first application
        }

        // Track progress
		this->monitorAutomatedCommand();
		return;
	}
}

/**
 * @brief Determine if drivetrain should halt. If no command has been recently received and 
 * drivetrain is not currently halted, halt.
 * 
 */
bool DriveController::shouldHalt(void)
{
	return (
		// Do not halt repeatedly
		(this->lastIssuedCommand != DrivetrainManualCommand::Halt) &&

		// Do not halt if executing an automated command
		(this->automatedCommandState != DrivetrainAutomatedResponse::InProgress) &&

		// Halt if too much time elapsed since received command
		((millis() - this->lastReceivedValidManualCommandTime) > DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND)
	);
}

/**
 * @brief Bring drivetrain to a halt without direct command.
 * 
 */
void DriveController::voluntaryHalt(bool startWithBrake)
{
    // Brake first
    if (startWithBrake)
    {
        this->voluntaryBrake();
        delay(DRIVETRAIN_BRAKE_TIME_BEFORE_HALT_VOLUNTARY);
    }

	// Come to a stop and notify
	this->processManualCommand(DrivetrainManualCommand::Halt);
	this->sendDrivetrainManualResponse(DrivetrainManualResponse::NotifyHalting);
}

/**
 * @brief Apply full brake without direct command.
 * 
 */
void DriveController::voluntaryBrake(void)
{
	// Come to a stop and notify
	this->processManualCommand(DrivetrainManualCommand::Brake);
	this->sendDrivetrainManualResponse(DrivetrainManualResponse::NotifyBraking);
}

/**
 * @brief Receive manual and automated commands and issue to Drivetrain.
 *  New automated commands will be discard forever while an automated command is in progress.
 *  Manual commands will overwrite and clear automated commands. Drivetrain will voluntarily halt
 *  when an automated command is completed or if no received manual command has been recently
 *  received and drivetrain is in motion.
 * 
 */
void DriveController::process(void)
{
	// Get commands
	DrivetrainManualCommand currentManualCommand = this->getDrivetrainManualCommand();
	this->checkDrivetrainAutomatedCommand();

	// Arbitrate commands
	this->arbitrateCommands(currentManualCommand);

	// Check if should halt based on no recent commands
	if (this->shouldHalt())
	{
		this->voluntaryHalt();
		return;
	}
}
