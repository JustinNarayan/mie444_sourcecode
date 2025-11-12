#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>
#include "DriveController.h"
#include <Lidar.h>

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInLidar = MessageTypes<
    MessageType::LidarState // Request for Lidar ping
>;
using MessageTypesOutLidar = MessageTypes<
    MessageType::LidarPointReading, // Single Lidar point reading
	MessageType::LidarState // Indicates if read successful/failure and when sent
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class LidarController : public Controller<
	MessageTypesInLidar, 
	MessageTypesOutLidar
>
{
private:
	/**
	 * Reference to Lidar
	 */
	Lidar* lidar;

	/**
	 * Reference to DriveController to request halts before Lidar reading
	 */
	DriveController* driveController;

	/**
	 * @brief Current Lidar data
	 * 
	 */
	LidarReading reading;

	/**
	 * Whether a request was received and not yet addressed
	 */
	bool hasUnaddressedRequest;
	unsigned long lastCompleteSentTimestampMillis;
	bool hasNotSentComplete;

	/**
	 * @brief Low-level utilities
	 * 
	 */
	void refreshLidarReading(void);

	/**
	 * @brief Communication utilities
	 */
	void checkLidarState(void);
	bool trySendNextLidarReadingPoint(void);
	void sendLidarState(LidarState state);
	void sendLidarData(void);

	/**
	 * @brief Process utilities
	 */
	bool hasUnsentReading(void);
	bool shouldAcceptNewRequests(void);
	bool shouldRefreshLidarReading(void);
	bool shouldSendLidarReading(void);
	bool shouldRequestPrioritizedSender(void);
public:
	LidarController(Lidar* lidar, DriveController* driveController);
	void process(void);
};