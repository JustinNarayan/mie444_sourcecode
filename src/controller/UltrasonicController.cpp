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
    sweepStartTime(0),
    encodersStart({0}),
    encodersNow({0}),
    hasPingedEncodersAtThisPosition(false),
    encoderPingTime(0),
    automatedCommandTime(0) {};

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

            // Set sweep start time
            this->sweepStartTime = millis();

            // Note that encoders should be pinged
            this->hasPingedEncodersAtThisPosition = false;
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

            // If this is the first encoder reading, store it as the start
            if (this->sweepState == UltrasonicSweepState::FirstEncoderReading)
                memoryCopy(&this->encodersStart, &reading, sizeof(DrivetrainEncoderDistances));
            
            // Store this as the current encoder reading
            memoryCopy(&this->encodersNow, &reading, sizeof(DrivetrainEncoderDistances));

            // Move onto sending point
            this->sweepState = UltrasonicSweepState::SendingPoint;
            
            // Mark that encoder ping is now outdated
            this->hasPingedEncodersAtThisPosition = false;
        }

        // Eliminate stale values
		this->purge(MessageType::DrivetrainEncoderDistances);
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
            this->sweepState = UltrasonicSweepState::SweepEncoderReading;
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
	
    // Continue to try sending complete until successfully enqueued
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
	this->post(&message);
}

/**
 * @brief Request drivetrain to rotate an increment
 * 
 */
void UltrasonicController::requestDrivetrainStep(void)
{
    DrivetrainAutomatedCommand command = { 
        .dX_in = 0, .dY_in = 0, .dTheta_deg = ULTRASONIC_SWEEP_INCREMENTS_DEG
    };
    this->envoy->envoyDrivetrainAutomatedCommand(&command);

    // Record when the command was sent
    this->automatedCommandTime = millis();
    
    // Move the state to confirming the command
    this->sweepState = UltrasonicSweepState::ConfirmingStep;
}

/**
 * @brief Request encoders to provide current readings
 * 
 */
void UltrasonicController::requestEncoders(void)
{
    this->envoy->envoyEncoderRequest();

    // Mark encoders as pinged
    this->hasPingedEncodersAtThisPosition = true;
    this->encoderPingTime = millis();
}

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
 * @brief Determine if sweep should be considered complete
 * 
 * @return Whether to complete
 */
bool UltrasonicController::shouldFinishSweep(void)
{
    // Too much time has elapsed
    if ((millis() - this->sweepStartTime) > ULTRASONIC_SWEEP_MAX_TIME_MS) return true;

    // Compute encoder distances
    bool encoder1Swept = \
        fabsf(this->encodersNow.encoder1Dist - this->encodersStart.encoder1Dist) > ULTRASONIC_SWEEP_MINIMUM_ENCODER_DISTANCE;
    bool encoder2Swept = \
        fabsf(this->encodersNow.encoder2Dist - this->encodersStart.encoder2Dist) > ULTRASONIC_SWEEP_MINIMUM_ENCODER_DISTANCE;
    bool encoder3Swept = \
        fabsf(this->encodersNow.encoder3Dist - this->encodersStart.encoder3Dist) > ULTRASONIC_SWEEP_MINIMUM_ENCODER_DISTANCE;
    
    // Any encoder has carved a circle (presumably all have)
    return encoder1Swept || encoder2Swept || encoder3Swept;
}

/**
 * @brief Assuming in a state to ping encoders, should they be pinged right now?
 * 
 * @return Whether to ping
 */
bool UltrasonicController::shouldPingEncoders(void)
{
    // Assuming state is valid
    return (
        (this->hasPingedEncodersAtThisPosition == false) ||
        ((millis() - this->encoderPingTime) > ULTRASONIC_TIME_TO_PING_ENCODERS_REPEATEDLY)
    );
}

/**
 * @brief Assuming waiting on a drivetrain response, move on if taking too long
 * 
 */
void UltrasonicController::ignoreDrivetrainAutomatedResponseIfTooLong(void)
{
    // Assuming state is valid
    if (
        (this->sweepState == UltrasonicSweepState::ConfirmingStep) &&
        ((millis() - this->automatedCommandTime) > ULTRASONIC_TIME_TO_IGNORE_AUTOMATED_COMMAND_RESPONSE)
    )
    {
        this->sweepState = UltrasonicSweepState::SweepEncoderReading;
    }
}

/**
 * @brief Ping both ultrasonics and send the point readings
 * 
 */
void UltrasonicController::pingUltrasonicsAndSend(void)
{
    // Create ultrasonic point readings
    UltrasonicPointReading reading1, reading2;
    reading1.whichUltrasonic = 1;
    reading2.whichUltrasonic = 2;
    
    // Set encoder values
    memoryCopy(&reading1.encoders, &this->encodersNow, sizeof(DrivetrainEncoderDistances));
    memoryCopy(&reading2.encoders, &this->encodersNow, sizeof(DrivetrainEncoderDistances));

    // Get pings
    reading1.distance = this->ultrasonic1->ping();
    reading2.distance = this->ultrasonic2->ping();

    // Send readings
    this->sendUltrasonicPointReading(&reading1);
    this->sendUltrasonicPointReading(&reading2);

    // Move to stepping the drivetrain
    this->sweepState = UltrasonicSweepState::IssuingStep;
}

/**
 * @brief Process ultrasonic sweep state
 * 
 */
void UltrasonicController::processSweep(void)
{
    // Check if sweep is complete
    if (
        (this->sweepState != UltrasonicSweepState::Idle) &&
        (this->sweepState != UltrasonicSweepState::Complete)
    )
    {
        if (this->shouldFinishSweep()) this->sweepState = UltrasonicSweepState::Complete;
    }

    // Execute state machine
    switch(this->sweepState)
    {
        // Do nothing
        default:
        case UltrasonicSweepState::Idle:
        {
            break;
        }
        // Send complete message if done
        case UltrasonicSweepState::Complete:
        {
            this->sendUltrasonicState(UltrasonicState::Complete);
            break;
        }
        // Ping encoders
        case UltrasonicSweepState::FirstEncoderReading:
        case UltrasonicSweepState::SweepEncoderReading:
        {
            if (this->shouldPingEncoders()) this->requestEncoders();
            break;
        }
        // Send a point
        case UltrasonicSweepState::SendingPoint:
        {
            this->pingUltrasonicsAndSend();
            break;
        }
        // Step the drivetrain
        case UltrasonicSweepState::IssuingStep:
        {
            this->requestDrivetrainStep();
            break;
        }
        // Wait for response
        case UltrasonicSweepState::ConfirmingStep:
        {
            this->ignoreDrivetrainAutomatedResponseIfTooLong();
            break;
        }
    }
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

    // Process the ultrasonic sweep state
    this->processSweep();
}
