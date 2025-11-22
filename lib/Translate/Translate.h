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
#if defined(BOARD_CONTROLLER)
ENUM_MESSAGE_MAP_TRANSLATION(LidarState)
ENUM_MESSAGE_MAP_TRANSLATION(UltrasonicState)
#endif

/**
 * @brief Instantiate all StructMessageMap objects for translation
 * 
 * Each call to STRUCT_MESSAGE_MAP_TRANSLATION(structType) creates structTypeTranslation.
 * 
 * All structs MUST be marked as __attribute__((packed))
 */
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainEncoderDistances)
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainAutomatedCommand)
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainDisplacements)
STRUCT_MESSAGE_MAP_TRANSLATION(DrivetrainMotorCommand)
#if defined(BOARD_CONTROLLER)
STRUCT_MESSAGE_MAP_TRANSLATION(LidarPointReading)
STRUCT_MESSAGE_MAP_TRANSLATION(UltrasonicPointReading)
#endif