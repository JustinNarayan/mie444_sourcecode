#include "DrivetrainEncoders.h"

/**
 * @brief There is one global DrivetrainEncoders instance which must be statically declared to 
 * access ISRs
 * 
 */
DrivetrainEncoders* DrivetrainEncoders::instance = nullptr;

/**
 * @brief Initialize all motor controllers
 * 
 */
void DrivetrainEncoders::init(
	uint8_t encoder1_a, uint8_t encoder1_b,
	uint8_t encoder2_a, uint8_t encoder2_b,
	uint8_t encoder3_a, uint8_t encoder3_b
)
{
	this->encoder1 = new Encoder(encoder1_a, encoder1_b);
	this->encoder2 = new Encoder(encoder2_a, encoder2_b);
	this->encoder3 = new Encoder(encoder3_a, encoder3_b);
}

/**
 * @brief Return the current values of all encoders, all as long values
 * 
 * @param distance1 
 * @param distance2 
 * @param distance3 
 */
void DrivetrainEncoders::getCurrentDistances(long *distance1, long *distance2, long* distance3)
{
	*distance1 = this->encoder1->getEncoderCount();
	*distance2 = this->encoder2->getEncoderCount();
	*distance3 = this->encoder3->getEncoderCount();
}
