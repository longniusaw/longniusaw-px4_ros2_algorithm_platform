#include "algorithm_platform/utils/logger.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>

namespace algorithm_platform {
namespace utils {

Logger::Logger(const std::string& name, const std::string& filepath)
  : name_(name), enabled_(true) {
  if (!filepath.empty()) {
    log_file_.open(filepath, std::ios::out | std::ios::app);
  }
}

Logger::~Logger() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void Logger::log(Level level, const std::string& message) {
  if (!enabled_) return;
  
  std::string prefix;
  switch(level) {
    case Level::INFO: prefix = "[INFO]"; break;
    case Level::WARN: prefix = "[WARN]"; break;
    case Level::ERROR: prefix = "[ERROR]"; break;
    case Level::DEBUG: prefix = "[DEBUG]"; break;
  }
  
  auto now = std::time(nullptr);
  std::string timestamp = std::to_string(now);
  
  std::string formatted = timestamp + " " + name_ + " " + prefix + " " + message;
  
  if (log_file_.is_open()) {
    log_file_ << formatted << std::endl;
  }
  if (log_to_console_) {
    std::cout << formatted << std::endl;
  }
}

} // namespace utils
} // namespace algorithm_platform
