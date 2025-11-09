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
    MessageType::LidarRequest // Request for Lidar ping
>;
using MessageTypesOutLidar = MessageTypes<
    MessageType::LidarPointReading, // Single Lidar point reading
    MessageType::Error // Errors
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

	/**
	 * @brief Low-level utilities
	 * 
	 */
	int refreshLidarReading(void);

	/**
	 * @brief Communication utilities
	 */
	void checkLidarRequest(void);
	void sendLidarReadingPoint(void);
	void sendLidarData(void);

	/**
	 * @brief Process utilities
	 */
	bool hasUnsentReading(void);
	bool shouldAcceptNewRequests(void);
	bool shouldRefreshLidarReading(void);
	bool shouldSendLidarReading(void);
public:
	LidarController(Lidar* lidar, DriveController* driveController);
	void process(void);
};