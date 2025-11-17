#pragma once
#include "Message.h"
#include "TranslateDrivetrain.h"
#include "TranslateSensors.h"

/**
 * @brief Instantiate all EnumMessageMap objects for translation
 * 
 * Each call to ENUM_MESSAGE_MAP_TRANSLATION(enumType) creates enumTypeTranslation
 */
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualCommand)
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualResponse)
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainAutomatedResponse)
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainEncoderState)
ENUM_MESSAGE_MAP_TRANSLATION(LidarState)

/**
 * @brief Instantiate all StructMessageMap objects for translation
 * 
 * Each call to STRUCT_MESSAGE_MAP_TRANSLATION(structType) creates structTypeTranslation
 */
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainEncoderDistances)
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainAutomatedCommand)
STRUCT_MESSAGE_MAP_TRANSLATION(LidarPointReading)
