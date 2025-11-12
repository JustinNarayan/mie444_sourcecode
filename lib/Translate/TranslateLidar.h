#pragma once
#include <MessageType.h>
#include <LidarDefs.h>
#include "TranslateEnumDefs.h"
#include "TranslateStructDefs.h"

/*****************************************************
 *                    ENUM MAPPING                   *
 *****************************************************/
static constexpr EnumStringMap LidarStateMap[] = {
    ENUM_MAP_ENTRY(LidarState::Invalid,			"invalid"), // placeholder
    ENUM_MAP_ENTRY(LidarState::NoReceived,		""),
    ENUM_MAP_ENTRY(LidarState::Request,			"l"),
	ENUM_MAP_ENTRY(LidarState::Rejected, 		"reject"),
    ENUM_MAP_ENTRY(LidarState::NotOpen,			"notopen"),
    ENUM_MAP_ENTRY(LidarState::CannotScan,		"cannotscan"),
    ENUM_MAP_ENTRY(LidarState::HealthUnknown,	"cannotverify"),
    ENUM_MAP_ENTRY(LidarState::NotHealthy,		"cannotscan"),
    ENUM_MAP_ENTRY(LidarState::Success,			"success"),
    ENUM_MAP_ENTRY(LidarState::Complete,		"complete"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(LidarStateMap, LidarState);

/*****************************************************
 *                  STRUCT MAPPING                   *
 *****************************************************/
COMPILE_TIME_ENFORCE_STRUCT_SIZE(LidarPointReading);
