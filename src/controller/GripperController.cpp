#include "MemoryUtilities.h"
#include "GripperController.h"
#include <Translate.h>

/**
 * @brief Construct a new GripperController
 * 
 * @param gripper
 */
GripperController::GripperController(Gripper *gripper) :
	gripper(gripper),
    hasUnaddressedCommand(false),
    lastReceivedCommand(GripperCommand::NoReceived),
    lastReceivedActionableCommand(GripperCommand::NoReceived) {}

/**
 * @brief Read input messages for GripperCommand type
 * 
 */
void GripperController::checkGripperCommand(void)
{
	// Dequeue GripperCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::GripperCommand, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
        GripperCommand command = GripperCommandTranslation.asEnum(&message);
        
        // Require valid commands
        if (
            (command == GripperCommand::Invalid) || (command == GripperCommand::NoReceived)
        )
            return;

        // Note valid command
		this->hasUnaddressedCommand = true;
        this->lastReceivedCommand = command;

        // If this is not a ping, store as actionable
        this->lastReceivedActionableCommand = command;

		// Eliminate stale messages
		this->purge(MessageType::GripperCommand);
	}
}

/**
 * @brief Send gripper state
 * 
 */
void GripperController::sendGripperState(void)
{
	Message message;
	GripperStateTranslation.asMessage(this->getCurrentState(), &message);
	this->post(&message);
}

/**
 * @brief Get current state
 * 
 * @return state
 */
GripperState GripperController::getCurrentState(void)
{
    // Check if at home
    if (this->gripper->isAtHome()) return GripperState::Home;

    // Check other states
    bool armExtended = this->gripper->isArmExtended();
    bool wristClosed = this->gripper->isWristClosed();    
    if (armExtended && wristClosed) return GripperState::ExtendedClosed;
    else if (armExtended) return GripperState::ExtendedOpened;
    else if (wristClosed) return GripperState::ReadyClosed;
    else return GripperState::ReadyOpened;
}

/**
 * @brief Check if gripper should actuate
 * 
 */
bool GripperController::shouldGripperActuate(void)
{
    return (
        // Command is actionable
        (this->lastReceivedActionableCommand != GripperCommand::NoReceived)
    );
}

/**
 * @brief Actuate gripper
 * 
 */
void GripperController::actuateGripper(void)
{
    switch (this->lastReceivedActionableCommand)
    {
        case GripperCommand::Home:
        {
            this->gripper->goHome();
            break;
        }
        case GripperCommand::Extend:
        {
            this->gripper->extendArm();
            break;
        }
        case GripperCommand::Ready:
        {
            this->gripper->readyArm();
            break;
        }
        case GripperCommand::Close:
        {
            this->gripper->closeWrist();
            break;
        }
        case GripperCommand::Open:
        {
            this->gripper->openWrist();
            break;
        }
        default:
        {
            break;
        }
    }
    this->lastReceivedActionableCommand = GripperCommand::NoReceived;
}

/**
 * @brief Actuate gripper upon request
 * 
 */
void GripperController::process(void)
{
	// Monitor incoming messages
	this->checkGripperCommand();

    // Check if has request
    if (this->hasUnaddressedCommand)
    {
        // Actuate gripper if actionable command
        if (this->shouldGripperActuate()) this->actuateGripper();

        // Send state
        this->sendGripperState();

        // Clear command
        this->hasUnaddressedCommand = false;
        this->lastReceivedCommand = GripperCommand::NoReceived;
    }
}
