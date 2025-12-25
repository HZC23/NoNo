#include "logger.h"

// Define the global log level variable
LogLevel currentLogLevel;

// Initialize the logger
void logger_init(LogLevel initialLevel) {
  currentLogLevel = initialLevel;
  LOG_INFO("Logger initialized with level %s", levelToString(initialLevel));
}

// Set a new log level
void setLogLevel(LogLevel level) {
  if (level >= LOG_LEVEL_NONE && level <= LOG_LEVEL_VERBOSE) {
    currentLogLevel = level;
    LOG_INFO("Log level set to %s", levelToString(level));
  } else {
    LOG_ERROR("Invalid log level: %d", level);
  }
}

// Helper function to convert log level enum to string
const char* levelToString(LogLevel level) {
  switch (level) {
    case LOG_LEVEL_NONE:    return "NONE";
    case LOG_LEVEL_ERROR:   return "ERROR";
    case LOG_LEVEL_INFO:    return "INFO";
    case LOG_LEVEL_DEBUG:   return "DEBUG";
    case LOG_LEVEL_VERBOSE: return "VERBOSE";
    default:                return "UNKNOWN";
  }
}
