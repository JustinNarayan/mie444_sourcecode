#include <Translate.h>

/**
 * @brief For all StructMessageMap instantiatons, define the particular deserialization function.
 * 
 * This can NOT be generally done with one function.
 * 
 */
/// DrivetrainEncoderDistances
template <>
void StructMessageMap<DrivetrainEncoderDistances>::strToStruct(
    DrivetrainEncoderDistances* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as three float32_t, little endian
    s->encoder1Dist = (float32_t)(
        asBytes[0] | 
        ((uint32_t)(asBytes[1]) << 8) | 
        ((uint32_t)(asBytes[2]) << 16) | 
        ((uint32_t)(asBytes[3]) << 24)
    );
    s->encoder2Dist = (float32_t)(
        asBytes[4] | 
        ((uint32_t)(asBytes[5]) << 8) | 
        ((uint32_t)(asBytes[6]) << 16) | 
        ((uint32_t)(asBytes[7]) << 24)
    );
    s->encoder3Dist = (float32_t)(
        asBytes[8] | 
        ((uint32_t)(asBytes[9]) << 8) | 
        ((uint32_t)(asBytes[10]) << 16) | 
        ((uint32_t)(asBytes[11]) << 24)
    );
    // float32_t = encoderDistance_in
}

/// DrivetrainAutomatedCOmmand
template <>
void StructMessageMap<DrivetrainAutomatedCommand>::strToStruct(
    DrivetrainAutomatedCommand* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as three int16_t, little endian
    s->dX_in =      (int16_t)(asBytes[0] | ((uint16_t)(asBytes[1]) << 8));
    s->dY_in =      (int16_t)(asBytes[2] | ((uint16_t)(asBytes[3]) << 8));
    s->dTheta_deg = (int16_t)(asBytes[4] | ((uint16_t)(asBytes[5]) << 8));
}

/// DrivetrainDisplacements
template <>
void StructMessageMap<DrivetrainDisplacements>::strToStruct(
    DrivetrainDisplacements* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as three float32_t, little endian
    s->dX_in = (float32_t)(
        asBytes[0] | 
        ((uint32_t)(asBytes[1]) << 8) | 
        ((uint32_t)(asBytes[2]) << 16) | 
        ((uint32_t)(asBytes[3]) << 24)
    );
    s->dY_in = (float32_t)(
        asBytes[4] | 
        ((uint32_t)(asBytes[5]) << 8) | 
        ((uint32_t)(asBytes[6]) << 16) | 
        ((uint32_t)(asBytes[7]) << 24)
    );
    s->dTheta_rad = (float32_t)(
        asBytes[8] | 
        ((uint32_t)(asBytes[9]) << 8) | 
        ((uint32_t)(asBytes[10]) << 16) | 
        ((uint32_t)(asBytes[11]) << 24)
    );
}

/// DrivetrainMotorCommand
template <>
void StructMessageMap<DrivetrainMotorCommand>::strToStruct(
    DrivetrainMotorCommand* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as three pairs of bool then float32_t, little endian
    s->is1Forward = asBytes[0];
    s->speed1 = (float32_t)(
        asBytes[1] | 
        ((uint32_t)(asBytes[2]) << 8) | 
        ((uint32_t)(asBytes[3]) << 16) | 
        ((uint32_t)(asBytes[4]) << 24)
    );
    s->is2Forward = asBytes[5];
    s->speed2 = (float32_t)(
        asBytes[6] | 
        ((uint32_t)(asBytes[7]) << 8) | 
        ((uint32_t)(asBytes[8]) << 16) | 
        ((uint32_t)(asBytes[9]) << 24)
    );
    s->is3Forward = asBytes[10];
    s->speed3 = (float32_t)(
        asBytes[11] | 
        ((uint32_t)(asBytes[12]) << 8) | 
        ((uint32_t)(asBytes[13]) << 16) | 
        ((uint32_t)(asBytes[14]) << 24)
    );
}

#if defined(BOARD_CONTROLLER)
// LidarPointReading
template <>
void StructMessageMap<LidarPointReading>::strToStruct(
    LidarPointReading* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as two int16_t, little endian
    s->angle =    (int16_t)(asBytes[0] | ((uint16_t)(asBytes[1]) << 8));
    s->distance = (int16_t)(asBytes[2] | ((uint16_t)(asBytes[3]) << 8));
    // int16_t = lidarAngle_deg, lidarDistance_in
}

// UltrasonicPointReading
template <>
void StructMessageMap<UltrasonicPointReading>::strToStruct(
    UltrasonicPointReading* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    // Interpret as int16_t and float32_t, little endian
    s->whichUltrasonic = asBytes[0];

    s->encoders.encoder1Dist = (float32_t)(
        asBytes[1] | 
        ((uint32_t)(asBytes[2]) << 8) | 
        ((uint32_t)(asBytes[3]) << 16) | 
        ((uint32_t)(asBytes[4]) << 24)
    );
    s->encoders.encoder2Dist = (float32_t)(
        asBytes[5] | 
        ((uint32_t)(asBytes[6]) << 8) | 
        ((uint32_t)(asBytes[7]) << 16) | 
        ((uint32_t)(asBytes[8]) << 24)
    );
    s->encoders.encoder3Dist = (float32_t)(
        asBytes[9] | 
        ((uint32_t)(asBytes[10]) << 8) | 
        ((uint32_t)(asBytes[11]) << 16) | 
        ((uint32_t)(asBytes[12]) << 24)
    );
    
    s->distance = (float32_t)(
        asBytes[13] | 
        ((uint32_t)(asBytes[14]) << 8) | 
        ((uint32_t)(asBytes[15]) << 16) | 
        ((uint32_t)(asBytes[16]) << 24)
    );
    // uint8_t = which ultrasonic
    // float32_t = encoderDistance_in
    // float32_t = ultrasonicDistance_in
}
#endif