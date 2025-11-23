#pragma once
#include <MessageType.h>
#include <GripperDefs.h>
#include "TranslateEnumDefs.h"

/*****************************************************
 *                    ENUM MAPPING                   *
 *****************************************************/
static constexpr EnumStringMap GripperCommandMap[] = {
    ENUM_MAP_ENTRY(GripperCommand::Invalid,     "invalid"), // placeholder
    ENUM_MAP_ENTRY(GripperCommand::NoReceived,  ""),
    ENUM_MAP_ENTRY(GripperCommand::Home,        "home"),
    ENUM_MAP_ENTRY(GripperCommand::Extend,      "extend"),
    ENUM_MAP_ENTRY(GripperCommand::Ready,       "ready"),
    ENUM_MAP_ENTRY(GripperCommand::Open,        "open"),
    ENUM_MAP_ENTRY(GripperCommand::Close,       "close"),
    ENUM_MAP_ENTRY(GripperCommand::Ping,        "ping"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(GripperCommandMap, GripperCommand);

static constexpr EnumStringMap GripperStateMap[] = {
    ENUM_MAP_ENTRY(GripperState::Invalid,           "invalid"), // placeholder
    ENUM_MAP_ENTRY(GripperState::NoReceived,        ""),
    ENUM_MAP_ENTRY(GripperState::Home,              "home"),
    ENUM_MAP_ENTRY(GripperState::ReadyClosed,       "ready_c"),
    ENUM_MAP_ENTRY(GripperState::ReadyOpened,       "ready_o"),
    ENUM_MAP_ENTRY(GripperState::ExtendedClosed,    "ext_c"),
    ENUM_MAP_ENTRY(GripperState::ExtendedOpened,    "ext_c"),
};
COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(GripperStateMap, GripperState);
