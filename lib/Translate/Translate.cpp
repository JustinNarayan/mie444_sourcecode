#include <Translate.h>

/**
 * @brief For all StructMessageMap instantiatons, define the particular deserialization function.
 * 
 * This can NOT be generally done with one function.
 * 
 */

/*****************************************************
 *                      HELPERS                      *
 *****************************************************/

inline float32_t deserializeF32(const uint8_t*& b) // passed as reference
{
    uint32_t raw = b[0] | (((uint32_t)b[1])<<8) | (((uint32_t)b[2])<<16) | (((uint32_t)b[3])<<24);
    b += sizeof(float32_t); // step pointer
    float32_t ret;
    memoryCopy(&ret, &raw, sizeof(float32_t)); // Interpret raw bytes as a float32_t
    return ret;
}

inline int16_t deserializeI16(const uint8_t*& b) // passed as reference
{
    uint16_t raw = b[0] | (((uint16_t)b[1])<<8);
    b += sizeof(int16_t); // step pointer
    int16_t ret;
    memoryCopy(&ret, &raw, sizeof(int16_t)); // Interpret raw bytes as an int16_t
    return ret;
}

inline uint8_t deserializeU8(const uint8_t*& b) // passed as reference
{
    b += sizeof(uint8_t); // step pointer
    return b[0]; // Interpret raw byte as a uint8_t
}

/*****************************************************
 *                  DESERIALIZATION                  *
 *****************************************************/

template <>
void StructMessageMap<DrivetrainEncoderDistances>::strToStruct(
    DrivetrainEncoderDistances* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->encoder1Dist = (encoderDistance_in)deserializeF32(asBytes);
    s->encoder2Dist = (encoderDistance_in)deserializeF32(asBytes);
    s->encoder3Dist = (encoderDistance_in)deserializeF32(asBytes);
}

template <>
void StructMessageMap<DrivetrainAutomatedCommand>::strToStruct(
    DrivetrainAutomatedCommand* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->dX_in =      deserializeI16(asBytes);
    s->dY_in =      deserializeI16(asBytes);
    s->dTheta_deg = deserializeI16(asBytes);
}

template <>
void StructMessageMap<DrivetrainDisplacements>::strToStruct(
    DrivetrainDisplacements* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->dX_in =      deserializeF32(asBytes);
    s->dY_in =      deserializeF32(asBytes);
    s->dTheta_rad = deserializeF32(asBytes);
}

template <>
void StructMessageMap<DrivetrainMotorCommand>::strToStruct(
    DrivetrainMotorCommand* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->is1Forward = (bool)deserializeU8(asBytes);
    s->speed1 =     (motorSpeedRaw)deserializeF32(asBytes);
    s->is2Forward = (bool)deserializeU8(asBytes);
    s->speed2 =     (motorSpeedRaw)deserializeF32(asBytes);
    s->is3Forward = (bool)deserializeU8(asBytes);
    s->speed3 =     (motorSpeedRaw)deserializeF32(asBytes);
}

#if defined(BOARD_CONTROLLER)
template <>
void StructMessageMap<LidarPointReading>::strToStruct(
    LidarPointReading* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->angle = (lidarAngle_deg)deserializeI16(asBytes);
    s->distance =  (lidarDistance_in)deserializeI16(asBytes);
}

template <>
void StructMessageMap<UltrasonicPointReading>::strToStruct(
    UltrasonicPointReading* s, const char* buffer) const
{
    const uint8_t* asBytes = (const uint8_t*)buffer;
    s->whichUltrasonic = deserializeU8(asBytes);
    DrivetrainEncoderDistancesTranslation.strToStruct(&(s->encoders), (const char*)asBytes);
    s->distance = (ultrasonicDistance_in)deserializeF32(asBytes);
}
#endif