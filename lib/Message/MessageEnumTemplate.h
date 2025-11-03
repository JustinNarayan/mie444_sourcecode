// // NOTE: This file is designed for multiple inclusion. Do NOT add #pragma once or include guards.
// // It must be included *after* defining MESSAGE_ENUM_NAME and MESSAGE_ENUM_TABLE.

// #ifndef MESSAGE_ENUM_NAME
// #error "MESSAGE_ENUM_NAME must be defined before including MessageEnumTemplate.h"
// #endif

// #ifndef MESSAGE_ENUM_TABLE
// #error "MESSAGE_ENUM_TABLE must be defined before including MessageEnumTemplate.h"
// #endif

// #include "Settings.h"
// #include "MemoryUtilities.h"

// // Concatenation template helpers
// #define ME_JOIN2(a, b) a##b
// #define ME_JOIN3(a, b, c) a##b##c

// /***************************************************
//  *                   ENUM TYPE                     *
//  **************************************************/
// enum MESSAGE_ENUM_NAME {
//     Invalid,
//     NoReceived,
// #define CMD(name, str) name
//     MESSAGE_ENUM_TABLE
// #undef CMD
//     Count
// };

// /***************************************************
//  *                  TO STRING                      *
//  **************************************************/
// static inline const char* ME_JOIN2(getStringFrom, MESSAGE_ENUM_NAME)(MESSAGE_ENUM_NAME value)
// {
//     switch (value)
//     {
// #define CMD(name, str) case MESSAGE_ENUM_NAME::name: return str;
//         MESSAGE_ENUM_TABLE
// #undef CMD
//         default:
//             return "";
//     }
// }

// /***************************************************
//  *                 FROM STRING                     *
//  **************************************************/
// static inline MESSAGE_ENUM_NAME ME_JOIN3(get, MESSAGE_ENUM_NAME, FromString)(const char* buffer)
// {
//     if (stringLength((char*)buffer) == 0)
//         return MESSAGE_ENUM_NAME::NoReceived;

// #define CMD(name, str) if (stringCompare((char*)buffer, (char*)str)) return MESSAGE_ENUM_NAME::name
//     MESSAGE_ENUM_TABLE
// #undef CMD

//     return MESSAGE_ENUM_NAME::Invalid;
// }

// // ---------- Cleanup ----------
// #undef ME_JOIN2
// #undef MESSAGE_ENUM_NAME
// #undef MESSAGE_ENUM_TABLE
