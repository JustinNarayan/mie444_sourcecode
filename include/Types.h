#pragma once
#include <Arduino.h>

/*****************************************************
 *                  UNIT CONVERSIONS                 *
 *****************************************************/

#define MM_TO_INCH(mm) (0.0393701f * mm)
#define M_TO_INCH(m) (39.3701f * m)
#define MS_TO_S(ms) ((float32_t) ms / 1000.0f)

/*****************************************************
 *                     DATA TYPES                    *
 *****************************************************/

typedef float float32_t;
typedef double float64_t;

typedef float32_t time_s; // Seconds
typedef unsigned long time_ms; // Millis
typedef unsigned long time_us; // Micros

typedef int16_t lidarAngle_deg;
typedef int16_t lidarDistance_in;
typedef uint16_t lidarPointIndex;

typedef float32_t encoderDistance_in;

typedef float32_t motorSpeedRaw;
typedef uint8_t motorSpeedApplied;
typedef bool motorDirection; // true is forward

typedef int16_t ultrasonicAngle_deg;
typedef float32_t ultrasonicDistance_in;
