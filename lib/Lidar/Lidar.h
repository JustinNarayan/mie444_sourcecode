#pragma once
#if defined(BOARD_CONTROLLER)
#include "LidarDefs.h"
#include <RPLidar.h>

#define LIDAR_READING_FAILURE (-1)
#define LIDAR_READING_SUCCESS (0)

/**
 * LiDAR sensor interface
 * 
 */
class Lidar
{
private:
	uint8_t controlPin;
	HardwareSerial* port;
	RPLidar rpLidar;

public:
	Lidar(void) {};
	void init(uint8_t controlPin, HardwareSerial* port);
	int requestReading(LidarReading* reading);
};

#endif