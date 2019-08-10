#ifndef LOGGING_H
#define LOGGING_H
#include <iostream>

#define LOG_INFO(module, msg) std::cout << " [INFO]:" << module << ": " << msg;
#define LOG_DEBUG(module, msg) std::cout << "[DEBUG]:" << module << ": " << msg;
#define LOG_ERROR(module, msg) std::cout << "[ERROR]:" << module << ": " << msg;
#define LOG_WARNING(module, msg) std::cout << " [WARN]:" << module << ": " << msg;
#define LOG_FATAL(module, msg) std::cout << "[FATAL]:" << module << ": " << msg;

#endif /// LOGGING_H