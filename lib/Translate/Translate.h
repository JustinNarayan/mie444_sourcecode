#pragma once
#include "Message.h"
#include "TranslateDrivetrainManual.h"

/**
 * @brief Instantiate all EnumMessageMap objects for translation
 * 
 * Each call to ENUM_MESSAGE_MAP_TRANSLATION(enumType) creates enumTypeTranslation
 */
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualCommand)
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualResponse)