#pragma once
#include "Types.h"
#include <Encoder.h>

#define RET_INIT_SUCCESS (0)
#define RET_INIT_FAILURE (-1)

class DrivetrainEncoders
{
private:
	Encoder* encoder1;
	Encoder* encoder2;
	Encoder* encoder3;
	static DrivetrainEncoders* instance; // To enable ISRs to access private variables

public:
	DrivetrainEncoders(void) { instance = this; };
	~DrivetrainEncoders()
	{
		delete encoder1;
		delete encoder2;
		delete encoder3;
	}

	void init(
		uint8_t encoder1_a, uint8_t encoder1_b,
		uint8_t encoder2_a, uint8_t encoder2_b,
		uint8_t encoder3_a, uint8_t encoder3_b
	);
	void getCurrentDistances(long *distance1, long *distance2, long* distance3);

	/**
	 * @brief Static ISR functions
	 * 
	 */
	static void ISR_encoder1() { instance->encoder1->updateEncoderISR(); }
	static void ISR_encoder2() { instance->encoder2->updateEncoderISR(); }
	static void ISR_encoder3() { instance->encoder3->updateEncoderISR(); }
};