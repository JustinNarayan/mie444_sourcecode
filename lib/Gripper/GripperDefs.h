#pragma once
#include "Settings.h"
#include "Types.h"

/*****************************************************
 *                       ENUMS                       *
 *****************************************************/

/**
 * Valid commands to be issued to the gripper.
 */
enum class GripperCommand
{
	Invalid,
	NoReceived,
    Home,
	Extend,
	Ready,
	Open,
	Close,
    Ping,

	Count
};

/**
 * Valid states from the gripper
 */
enum class GripperState
{
	Invalid,
	NoReceived,
    Home,
    ReadyClosed,
    ReadyOpened,
    ExtendedClosed,
    ExtendedOpened,

	Count
};