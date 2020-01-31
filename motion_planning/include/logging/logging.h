///
/// @file
///
#ifndef LOGGING_H_
#define LOGGING_H_

#include <iostream>
#include <sstream>

#define LOG_INFO(module, msg) std::cout << " [INFO]:" << module << ": " << msg;
#define LOG_ERROR(module, msg) std::cout << "[ERROR]:" << module << ": " << msg;
#define LOG_WARNING(module, msg) std::cout << " [WARN]:" << module << ": " << msg;
#define LOG_FATAL(module, msg) std::cout << "[FATAL]:" << module << ": " << msg;

#ifdef NDEBUG
#define LOG_DEBUG(module, msg) std::cout << "[DEBUG]:" << module << ": " << msg;
#else
#define LOG_DEBUG(module, msg) ;
#endif  // NDEBUG

#endif  /// LOGGING_H_
