#pragma once
#include "Message.h"
#include "TranslateDrivetrain.h"

/**
 * @brief Instantiate all EnumMessageMap objects for translation
 * 
 * Each call to ENUM_MESSAGE_MAP_TRANSLATION(enumType) creates enumTypeTranslation
 */
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualCommand)
ENUM_MESSAGE_MAP_TRANSLATION(DrivetrainManualResponse)

/**
 * @brief Instantiate all StructMessageMap objects for translation
 * 
 * Each call to STRUCT_MESSAGE_MAP_TRANSLATION(structType) creates structTypeTranslation
 */
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainEncoderDistances)