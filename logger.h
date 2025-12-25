#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// 1. Log Levels
enum LogLevel {
  LOG_LEVEL_NONE,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_INFO,
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_VERBOSE
};

// 2. Logger Configuration
extern LogLevel currentLogLevel;

// 3. Function Prototypes
void logger_init(LogLevel initialLevel = LOG_LEVEL_INFO);
void setLogLevel(LogLevel level);
const char* levelToString(LogLevel level);

// 4. The Powerful Log Macro
// This macro will print the log level, file name, line number, and the formatted message.
#define LOG(level, format, ...) \
  do { \
    if (level <= currentLogLevel) { \
      Serial.printf("[%s] [%s:%d] ", levelToString(level), __FILE__, __LINE__); \
      Serial.printf(format, ##__VA_ARGS__); \
      Serial.println(); \
    } \
  } while (0)

// 5. Helper Macros for each level
#define LOG_ERROR(format, ...)   LOG(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)    LOG(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOG_DEBUG(format, ...)   LOG(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOG_VERBOSE(format, ...) LOG(LOG_LEVEL_VERBOSE, format, ##__VA_ARGS__)


#endif // LOGGER_H
