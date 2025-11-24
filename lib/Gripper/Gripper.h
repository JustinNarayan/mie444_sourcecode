#pragma once
#include "Settings.h"
#include "Types.h"
#include "GripperDefs.h"
#include <Servo.h>

/**
 * Gripper
 */
class Gripper
{
private:
	Servo* servoArm;
    Servo* servoWrist;
    bool atHome;
    bool armExtended;
    bool wristClosed;

public:
	Gripper(void) {};
	~Gripper()
	{
		delete servoArm;
		delete servoWrist;
	}

    /**
     * @brief Initialize a gripper object with two servo motors
     * 
     * @param servoArm_pwm 
     * @param servoWrist_pwm 
     */
	void init(uint8_t servoArm_pwm, uint8_t servoWrist_pwm)
    {
        this->servoArm = new Servo(
            servoArm_pwm, 
            GRIPPER_ARM_RETRACTED_POS,
            GRIPPER_ARM_RETRACTED_POS,
            GRIPPER_ARM_EXTENDED_POS
        );
        this->servoWrist = new Servo(
            servoWrist_pwm, 
            GRIPPER_WRIST_REST_POS,
            GRIPPER_WRIST_OPENED_POS,
            GRIPPER_WRIST_CLOSED_POS
        );
        this->atHome = false;
        this->armExtended = false;
        this->wristClosed = false;

        // Go home
        this->goHome();
    }

    /**
     * @brief Go home
     * 
     */
    void goHome(void)
    {
        this->servoArm->reposition(GRIPPER_ARM_RETRACTED_POS, true);
        this->servoWrist->reposition(GRIPPER_WRIST_REST_POS, true);
        this->atHome = true;
        this->armExtended = false;
        this->wristClosed = false;
    }

    /**
     * @brief Extend arm servo
     * 
     */
    void extendArm(void) { 
        this->servoArm->reposition(GRIPPER_ARM_EXTENDED_POS); 
        this->atHome = false;
        this->armExtended = true;
    }

    /**
     * @brief Ready arm servo
     * 
     */
    void readyArm(void) { 
        this->servoArm->reposition(GRIPPER_ARM_READY_POS); 
        this->atHome = false;
        this->armExtended = false;
    }

    /**
     * @brief Open wrist servo
     * 
     */
    void openWrist(void) { 
        this->servoWrist->reposition(GRIPPER_WRIST_OPENED_POS); 
        this->atHome = false;
        this->wristClosed = false;
    }
    
    /**
     * @brief Close wrist servo
     * 
     */
    void closeWrist(void) { 
        this->servoWrist->reposition(GRIPPER_WRIST_CLOSED_POS); 
        this->atHome = false;
        this->wristClosed = true;
    }

    /**
     * @brief Check if at home
     * 
     */
    bool isAtHome(void)
    {
        return this->atHome;
    }

    /**
     * @brief Check if arm extended
     * 
     */
    bool isArmExtended(void)
    {
        return this->armExtended;
    }

    /**
     * @brief Check if wrist closed
     * 
     */
    bool isWristClosed(void)
    {
        return this->wristClosed;
    }
};
