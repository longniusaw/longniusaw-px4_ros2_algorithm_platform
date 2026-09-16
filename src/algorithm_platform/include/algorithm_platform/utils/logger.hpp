#pragma once
#include <string>
#include <fstream>

namespace algorithm_platform {
namespace utils {

class Logger {
public:
  enum class Level { INFO, WARN, ERROR, DEBUG };
  
  Logger(const std::string& name, const std::string& filepath = "");
  ~Logger();
  
  void log(Level level, const std::string& message);
  void enable(bool enabled) { enabled_ = enabled; }
  void setConsoleOutput(bool enable) { log_to_console_ = enable; }
  
private:
  std::string name_;
  std::ofstream log_file_;
  bool enabled_ = true;
  bool log_to_console_ = true;
};

} // namespace utils
} // namespace algorithm_platform
