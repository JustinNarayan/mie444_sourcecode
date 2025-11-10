#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <CommsEnvoy.h>
#include <Controller.h>

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInEncoder = MessageTypes<
    MessageType::DrivetrainEncoderRequest // Request for Encoder readings
>;
using MessageTypesOutEncoder = MessageTypes<
    MessageType::DrivetrainEncoderPinged // Indicate Encoder pinged
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class EncoderRequestController : public Controller<
	MessageTypesInEncoder, 
	MessageTypesOutEncoder
>
{
private:
	/**
	 * Reference to Comms Envoy
	 */
	CommsEnvoy* envoy;

	/**
	 * Whether a request was received and not yet addressed
	 */
	bool hasUnaddressedRequest;

	/**
	 * @brief Communication utilities
	 */
	void checkEncoderRequest(void);
	void envoyRequest(void);
	void sendEncoderPinged(void);

	/**
	 * @brief Process utilities
	 */
	bool shouldEnvoyRequest(void);
public:
	EncoderRequestController(CommsEnvoy* envoy);
	void process(void);
};