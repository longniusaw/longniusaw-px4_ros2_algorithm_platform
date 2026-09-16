#include "algorithm_platform/utils/filter.hpp"
#include <cmath>

namespace algorithm_platform {
namespace utils {

LowPassFilter::LowPassFilter(double cutoff_freq, double dt)
  : alpha_(1.0 / (1.0 + 1.0 / (2.0 * M_PI * cutoff_freq * dt))) {}

double LowPassFilter::filter(double input) {
  output_ = alpha_ * input + (1.0 - alpha_) * output_;
  return output_;
}

void LowPassFilter::reset(double value) {
  output_ = value;
}

RateLimiter::RateLimiter(double max_rate, double dt)
  : max_rate_(max_rate), dt_(dt) {}

double RateLimiter::limit(double input, double previous_output) {
  double max_change = max_rate_ * dt_;
  double diff = input - previous_output;
  if (std::abs(diff) > max_change) {
    return previous_output + std::copysign(max_change, diff);
  }
  return input;
}

} // namespace utils
} // namespace algorithm_platform
