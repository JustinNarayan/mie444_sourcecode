#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>
#include "Gripper.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInGripper = MessageTypes<
    MessageType::GripperCommand // Gripper request
>;
using MessageTypesOutGripper = MessageTypes<
    MessageType::GripperState // Gripper state
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class GripperController : public Controller<
	MessageTypesInGripper, 
	MessageTypesOutGripper
>
{
private:
	/**
	 * Reference to Gripper
	 */
	Gripper* gripper;

	/**
	 * Whether a command was received and not yet addressed
	 */
	bool hasUnaddressedCommand;

	/**
	 * @brief Current gripper command and state
	 * 
	 */
    GripperCommand lastReceivedCommand;
    GripperCommand lastReceivedActionableCommand;

	/**
	 * @brief Communication utilities
	 */
	void checkGripperCommand(void);
	void sendGripperState(void);

    /**
     * @brief Process utilities
     * 
     */
    GripperState getCurrentState(void);
    bool shouldGripperActuate(void);
    void actuateGripper(void);
public:
	GripperController(Gripper* gripper);
	void process(void);
};