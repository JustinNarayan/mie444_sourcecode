#pragma once
#include "Types.h"
#include <MotorController.h>

#define RET_SET_COMMAND_SUCCESS (0)
#define RET_SET_COMMAND_FAILURE (-1)

#define RET_HALT_COMMAND_FAILURE (-1)
#define RET_HALT_COMMAND_SUCCESS (0)

class Drivetrain
{
private:
	MotorController* motor1;
	MotorController* motor2;
	MotorController* motor3;

public:
	Drivetrain(void) {};
	~Drivetrain()
	{
		delete motor1;
		delete motor2;
		delete motor3;
	}

	void init(
		uint8_t motor1_enable, uint8_t motor1_in1, uint8_t motor1_in2,
		uint8_t motor2_enable, uint8_t motor2_in1, uint8_t motor2_in2,
		uint8_t motor3_enable, uint8_t motor3_in1, uint8_t motor3_in2
	);
	int setTranslate(uint8_t speed, bool isForward);
	int setRotate(uint8_t speed, bool isLeft);
	int halt(void);
};
