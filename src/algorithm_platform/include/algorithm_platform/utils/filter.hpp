#pragma once

namespace algorithm_platform {
namespace utils {

class LowPassFilter {
public:
  LowPassFilter(double cutoff_freq, double dt);
  double filter(double input);
  void reset(double value = 0.0);
  
private:
  double alpha_;
  double output_ = 0.0;
};

class RateLimiter {
public:
  RateLimiter(double max_rate, double dt);
  double limit(double input, double previous_output);
  
private:
  double max_rate_;
  double dt_;
};

} // namespace utils
} // namespace algorithm_platform
