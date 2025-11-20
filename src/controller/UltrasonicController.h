#pragma once
#include "Types.h"
#include <Ultrasonic.h>
#include <Controller.h>
#include "PeripheralEnvoy.h"
#include "PeripheralForwardingController.h"
#include "DrivetrainDefs.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInUltrasonic = MessageTypes<
    MessageType::UltrasonicState, // Request for ultrasonic ping
    MessageType::DrivetrainEncoderDistances, // For determining current heading
    MessageType::DrivetrainAutomatedResponse // for determin
>;
using MessageTypesOutUltrasonic = MessageTypes<
    MessageType::UltrasonicPointReading, // Single Ultrasonic point reading
	MessageType::UltrasonicState // Indicates if read successful/failure and when sent
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class UltrasonicController : public Controller<
	MessageTypesInUltrasonic, 
	MessageTypesOutUltrasonic
>
{
private:
    /**
     * Reference to Ultrasonic sensors
     */
    Ultrasonic* ultrasonic1;
    Ultrasonic* ultrasonic2;

	/**
	 * Reference to Peripheral Envoy to request encoder readings before Ultrasonic reading
	 */
	PeripheralEnvoy* envoy;

    /**
     * Reference to Peripheral Forwarding Controller to request drivetrain rotations
     * 
     */
    PeripheralForwardingController* forwarding;

    /**
     * Record the encoders and the current state of an ultrasonic sweep
     */
    UltrasonicSweepState sweepState;
    DrivetrainEncoderDistances encodersStart;
    DrivetrainEncoderDistances encodersNow;
    ultrasonicAngle_deg targetAngle;

    /**
     * Communications utilities
     */
    void checkUltrasonicState(void);
    void checkDrivetrainEncoderDistances(void);
    void checkDrivetrainAutomatedResponse(void);
	void sendUltrasonicState(UltrasonicState state);
    void sendUltrasonicPointReading(UltrasonicPointReading* reading);
    void requestDrivetrainStep(void);

    /**
     * Process utilities
     */
    bool shouldAcceptNewRequests(void);
public:
    UltrasonicController(
        Ultrasonic* ultrasonic1, 
        Ultrasonic* ultrasonic2,
        PeripheralEnvoy* envoy,
        PeripheralForwardingController* forwarding
    );
    void process(void);
};