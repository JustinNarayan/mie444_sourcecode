#pragma once
#include "Types.h"
#include "Settings.h"

/*****************************************************
 *                 COMPILER UTILITIES                *
 *****************************************************/
#define BITMASK_IS_SET(b, n) (( (b)[(n)/8] & (1 << ((n)%8)) ) != 0)
#define BITMASK_SET(b, n)      ((b)[(n)/8] |=  (1 << ((n)%8)))
#define BITMASK_CLEAR(b, n)    ((b)[(n)/8] &= ~(1 << ((n)%8)))

/*****************************************************
 *                       ENUMS                       *
 *****************************************************/

/**
 * Valid messages for Lidar communication
 */
enum class LidarState
{
	Invalid,
	NoReceived,
	Request,
	Rejected,
	NotOpen,
	CannotScan,
	HealthUnknown,
	NotHealthy,
	Success,
	Complete,

	Count
};

/*****************************************************
 *                      STRUCTS                      *
 *****************************************************/

/**
 * Structure for individual Lidar point readings
 */
struct LidarPointReading
{
	lidarAngle_deg angle;
	lidarDistance_in distance;
};

/**
 * Structure for recording results of a complete Lidar sweep
 * Bitmask records which points have been with sufficient quality
 */
struct LidarReading
{
	LidarPointReading point[LIDAR_GRANULARITY_NUM_POINTS];
	uint8_t bitmask[(LIDAR_GRANULARITY_NUM_POINTS + 7) / 8];
	lidarPointIndex numCollected;
	lidarPointIndex numProcessed;
	lidarPointIndex indexToProcess;
};


/*****************************************************
 *                     UTILITIES                     *
 *****************************************************/

static inline bool isLidarReadingEmpty(LidarReading* reading)
{
	return (reading->numCollected == 0);	
}

static inline bool isLidarReadingFullyProcessed(LidarReading* reading)
{
	return (reading->numCollected <= reading->numProcessed);
}

static inline void stepLidarReadingToNextValidPoint(LidarReading* reading)
{
	if (isLidarReadingFullyProcessed(reading)) 
	{
		reading->indexToProcess = 0;
	}

    do
    {
        if (BITMASK_IS_SET(reading->bitmask, reading->indexToProcess))
            return;

        reading->indexToProcess++;

    } while (reading->indexToProcess < LIDAR_GRANULARITY_NUM_POINTS);
}

static inline void markLidarReadingIndexProcessed(LidarReading* reading)
{
	BITMASK_CLEAR(reading->bitmask, reading->indexToProcess);
	reading->numProcessed++;
}

static inline LidarPointReading* getCurrentLidarPointReading(LidarReading* reading)
{
	return &(reading->point[reading->indexToProcess]);
}