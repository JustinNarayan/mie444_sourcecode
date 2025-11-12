#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <CommsEnvoy.h>
#include <Controller.h>

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInEncoder = MessageTypes<
    MessageType::DrivetrainEncoderState // Request for Encoder readings
>;
using MessageTypesOutEncoder = MessageTypes<
    MessageType::DrivetrainEncoderState // Indicate Encoder pinged
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
	void checkEncoderState(void);
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