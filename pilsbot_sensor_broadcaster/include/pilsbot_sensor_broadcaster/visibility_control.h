#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PILSBOT_SENSOR_BROADCASTER_EXPORT __attribute__((dllexport))
#define PILSBOT_SENSOR_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define PILSBOT_SENSOR_BROADCASTER_EXPORT __declspec(dllexport)
#define PILSBOT_SENSOR_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef PILSBOT_SENSOR_BROADCASTER_BUILDING_DLL
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC PILSBOT_SENSOR_BROADCASTER_EXPORT
#else
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC PILSBOT_SENSOR_BROADCASTER_IMPORT
#endif
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC_TYPE PILSBOT_SENSOR_BROADCASTER_PUBLIC
#define PILSBOT_SENSOR_BROADCASTER_LOCAL
#else
#define PILSBOT_SENSOR_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define PILSBOT_SENSOR_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define PILSBOT_SENSOR_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC
#define PILSBOT_SENSOR_BROADCASTER_LOCAL
#endif
#define PILSBOT_SENSOR_BROADCASTER_PUBLIC_TYPE
#endif