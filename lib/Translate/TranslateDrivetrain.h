#pragma once
#include <MessageType.h>
#include <DrivetrainDefs.h>
#include "TranslateEnumDefs.h"
#include "TranslateStructDefs.h"

/*****************************************************
 *                    ENUM MAPPING                   *
 *****************************************************/
static constexpr EnumStringMap DrivetrainManualCommandMap[] = {
    ENUM_MAP_ENTRY(DrivetrainManualCommand::Invalid,           "invalid"), // placeholder
    ENUM_MAP_ENTRY(DrivetrainManualCommand::NoReceived,        ""),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::TranslateForward,  "w"),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::TranslateBackward, "s"),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::RotateLeft,        "a"),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::RotateRight,       "d"),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::Halt,              "h"),
    ENUM_MAP_ENTRY(DrivetrainManualCommand::Automated,         "a"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainManualCommandMap, DrivetrainManualCommand);

static constexpr EnumStringMap DrivetrainManualResponseMap[] = {
    ENUM_MAP_ENTRY(DrivetrainManualResponse::Invalid,       "invalid"), // placeholder
    ENUM_MAP_ENTRY(DrivetrainManualResponse::NoReceived,    ""),
    ENUM_MAP_ENTRY(DrivetrainManualResponse::Acknowledge,   "ackval"),
    ENUM_MAP_ENTRY(DrivetrainManualResponse::NotifyHalting, "notifhalt"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainManualResponseMap, DrivetrainManualResponse);

static constexpr EnumStringMap DrivetrainAutomatedResponseMap[] = {
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::Invalid,        "invalid"), // placeholder
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::NoReceived,     ""),
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::Acknowledge,    "ack"),
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::InProgress,    "inprog"),
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::Success,        "success"),
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::Failure,        "failure"),
    ENUM_MAP_ENTRY(DrivetrainAutomatedResponse::Aborted,        "abort"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainAutomatedResponseMap, DrivetrainAutomatedResponse);

static constexpr EnumStringMap DrivetrainEncoderStateMap[] = {
    ENUM_MAP_ENTRY(DrivetrainEncoderState::Invalid,		"invalid"), // placeholder
    ENUM_MAP_ENTRY(DrivetrainEncoderState::NoReceived,	""),
    ENUM_MAP_ENTRY(DrivetrainEncoderState::Request,		"e")
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainEncoderStateMap, DrivetrainEncoderState);

/*****************************************************
 *                  STRUCT MAPPING                   *
 *****************************************************/
COMPILE_TIME_ENFORCE_STRUCT_SIZE(DrivetrainEncoderDistances);
COMPILE_TIME_ENFORCE_STRUCT_SIZE(DrivetrainAutomatedCommand);
