#include "UltrasonicController.h"
#include <Translate.h>

/**
 * @brief Construct a new Ultrasonic Controller
 * 
 * @param ultrasonic1 First ultrasonic sensor
 * @param ultrasonic2 Second ultrasonic sensor
 * @param envoy Peripheral envoy
 * @param forwarding Peripheral forwarding controller
 */
UltrasonicController::UltrasonicController(
    Ultrasonic* ultrasonic1, 
    Ultrasonic* ultrasonic2,
    PeripheralEnvoy* envoy,
    PeripheralForwardingController* forwarding
) : ultrasonic1(ultrasonic1), 
    ultrasonic2(ultrasonic2), 
    envoy(envoy), 
    forwarding(forwarding),
    sweepState(UltrasonicSweepState::Idle),
    encodersStart({0}),
    encodersNow({0}),
    targetAngle(0) {};

/**
 * @brief Read input messages for UltrasonicState type
 * 
 */
void UltrasonicController::checkUltrasonicState(void)
{
	// Dequeue UltrasonicState
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::UltrasonicState, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		// Confirm message is a request, so it's valid
		if (UltrasonicStateTranslation.asEnum(&message) != UltrasonicState::Request)
			return;

		// Determine if request is valid
		if (this->shouldAcceptNewRequests())
		{
			// Accept
			this->sweepState = UltrasonicSweepState::FirstEncoderReading;

            // Ping encoders
            this->envoy->envoyEncoderRequest();
		}
		else
		{
			// Reject
			this->sendUltrasonicState(UltrasonicState::Rejected);
		}

		// Eliminate stale requests
		this->purge(MessageType::UltrasonicState);
	}
}

/**
 * @brief Read input messages for DrivetrainEncoderDistances type
 * 
 */
void UltrasonicController::checkDrivetrainEncoderDistances(void)
{
    // Dequeue DrivetrainEncoderDistances
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderDistances, &message);
    
    if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
        // Only save if waiting on encoder data
        if (
            (this->sweepState == UltrasonicSweepState::FirstEncoderReading) ||
            (this->sweepState == UltrasonicSweepState::SweepEncoderReading)
        )
        {
            // Unpack information
            DrivetrainEncoderDistances reading;
            DrivetrainEncoderDistancesTranslation.asStruct(&message, &reading);

            // Save as either initial reading or current reading
            if (this->sweepState == UltrasonicSweepState::FirstEncoderReading)
            {
                memoryCopy(&this->encodersStart, &reading, sizeof(DrivetrainEncoderDistances));
                this->sweepState == UltrasonicSweepState::IssuingStep;
            }
            else
            {
                memoryCopy(&this->encodersNow, &reading, sizeof(DrivetrainEncoderDistances));
                this->sweepState == UltrasonicSweepState::SendingPoint;
            }
        }

        // No need to purge
    }
}

/**
 * @brief Read input messages for DrivetrainAutomatedResponse type
 * 
 */
void UltrasonicController::checkDrivetrainAutomatedResponse(void)
{
    // Dequeue DrivetrainAutomatedResponse
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainAutomatedResponse, &message);
    
    if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
        DrivetrainAutomatedResponse response = DrivetrainAutomatedResponseTranslation.asEnum(
            &message
        );

        // Only consider messages regarding end of command
        if ( 
            (response != DrivetrainAutomatedResponse::Success) && 
            (response != DrivetrainAutomatedResponse::Failure)
        )
            return;

        // Step sweep if waiting on command completion
        if (this->sweepState == UltrasonicSweepState::ConfirmingStep)
            this->sweepState == UltrasonicSweepState::SweepEncoderReading;
    }
}

/**
 * @brief Notify current Ultrasonic state
 * 
 */
void UltrasonicController::sendUltrasonicState(UltrasonicState state)
{
	Message message;
	UltrasonicStateTranslation.asMessage(state, &message);
	ControllerMessageQueueOutput result = this->post(&message);
	
	if (this->sweepState == UltrasonicSweepState::Complete)
		if (result == ControllerMessageQueueOutput::EnqueueSuccess)
			this->sweepState = UltrasonicSweepState::Idle;
}

/**
 * @brief Send an ultrasonic point reading
 * 
 * @param reading 
 */
void UltrasonicController::sendUltrasonicPointReading(UltrasonicPointReading* reading)
{
	Message message;
	UltrasonicPointReadingTranslation.asMessage(reading, &message);
	ControllerMessageQueueOutput result = this->post(&message);
}

/**
 * @brief 
 * 
 */
// void UltrasonicController:requestDrivetrainStep()

/**
 * @brief Determine if Ultrasonics should accept new requests
 * 
 * @return Whether a new sweep can be initiated
 */
bool UltrasonicController::shouldAcceptNewRequests(void)
{
    return (
        this->sweepState == UltrasonicSweepState::Idle
    );
}

/**
 * @brief Initiate and execute an ultrasonic sweep upon requests
 * 
 */
void UltrasonicController::process(void)
{
	// Monitor incoming messages
	this->checkUltrasonicState();
    this->checkDrivetrainEncoderDistances();
    this->checkDrivetrainAutomatedResponse();
}
