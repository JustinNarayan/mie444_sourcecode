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
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainManualCommandMap, DrivetrainManualCommand);

static constexpr EnumStringMap DrivetrainManualResponseMap[] = {
    ENUM_MAP_ENTRY(DrivetrainManualResponse::Invalid,                   "invalid"), // placeholder
    ENUM_MAP_ENTRY(DrivetrainManualResponse::NoReceived,                ""),
    ENUM_MAP_ENTRY(DrivetrainManualResponse::AcknowledgeValidCommand,   "ackval"),
    ENUM_MAP_ENTRY(DrivetrainManualResponse::AcknowledgeInvalidCommand, "ackinv"),
    ENUM_MAP_ENTRY(DrivetrainManualResponse::NotifyHalting,             "notifhalt"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(DrivetrainManualResponseMap, DrivetrainManualResponse);

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
