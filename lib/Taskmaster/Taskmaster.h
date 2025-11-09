#pragma once
#include <CommsInterface.h>
#include <Controller.h>

/*****************************************************
 *                 COMPILER UTILITIES                *
 *****************************************************/
#define LOOP_CONTROLLER_IDX(controllerVar)                                \
    for (size_t controller_idx = 0; controller_idx < numControllers; ++controller_idx)
		
#define CONTROLLERS_NUM_ELEMENTS(c) (size_t)(sizeof(c)/sizeof(c[0]))
#define TASKMASTER_DECLARE(name, comms, controllers) \
	Taskmaster name(comms, controllers, CONTROLLERS_NUM_ELEMENTS(controllers));

/**
 * @brief A Taskmaster object arbitrates between the CommsInterface and all Controller objects.
 * Every loop, the Taskmaster:
 * - Reads the CommsInterface for any Message object
 * - If a Message is received, it will disseminate it to all Controller objects receiving that 
 * type of Message
 * - Allow all Controller objects to process
 * - Checks for all Controllers with Message objects to send and preaches them
 * 
 * @tparam numControllers 
 */
class Taskmaster 
{
private:
	CommsInterface* comms;
	ControllerGeneric** controllers;
	size_t numControllers;

	void receive(void);
	bool poll(Message* message);
	void dispatch(Message* message);
	void process(void);
	void collect(void);
public:
	Taskmaster(
		CommsInterface* comms, ControllerGeneric* controllers[], size_t numControllers
	) : comms(comms), controllers(controllers), numControllers(numControllers) {};

	void execute(void);
};
