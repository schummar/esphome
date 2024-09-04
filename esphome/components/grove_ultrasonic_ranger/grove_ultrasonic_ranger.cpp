#include "grove_ultrasonic_ranger.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace grove_ultrasonic_ranger {

static const char *const TAG = "ultrasonic.sensor";

void GroveUltrasonicRangerSensorComponent::set_pin(InternalGPIOPin *pin) { pin_ = pin; }

void GroveUltrasonicRangerSensorComponent::set_timeout_m(uint32_t timeout_m) {
  ESP_LOGD(TAG, "Setting timeout. m=%, us=%", timeout_m, m_to_us(timeout_m));
  timeout_us_ = us_to_m(timeout_m);
}

void GroveUltrasonicRangerSensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Ultrasonic Sensor...");
  pin_->setup();
  pin_->digital_write(false);
  pin_isr_ = pin_->to_isr();
}

void GroveUltrasonicRangerSensorComponent::update() {
  this->pin_isr_.digital_write(false);
  delayMicroseconds(2);
  this->pin_isr_.digital_write(true);
  delayMicroseconds(5);
  this->pin_isr_.digital_write(false);

  const uint32_t start = micros();
  while (micros() - start < timeout_us_ && pin_isr_.digital_read())
    ;
  while (micros() - start < timeout_us_ && !pin_isr_.digital_read())
    ;
  const uint32_t pulse_start = micros();
  while (micros() - start < timeout_us_ && pin_isr_.digital_read())
    ;
  const uint32_t pulse_end = micros();

  ESP_LOGV(TAG, "Echo took %" PRIu32 "µs", pulse_end - pulse_start);

  if (pulse_end - start >= timeout_us_) {
    ESP_LOGD(TAG, "'%s' - Distance measurement timed out!", this->name_.c_str());
    this->publish_state(NAN);
  } else {
    float result = GroveUltrasonicRangerSensorComponent::us_to_m(pulse_end - pulse_start);
    ESP_LOGD(TAG, "'%s' - Got distance: %.3f m", this->name_.c_str(), result);
    this->publish_state(result);
  }
}
void GroveUltrasonicRangerSensorComponent::dump_config() {
  LOG_SENSOR("", "Ultrasonic Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " µs", this->timeout_us_);
  LOG_UPDATE_INTERVAL(this);
}

float GroveUltrasonicRangerSensorComponent::us_to_m(uint32_t us) {
  const float speed_sound_m_per_s = 343.0f;
  const float time_s = us / 1e6f;
  const float total_dist = time_s * speed_sound_m_per_s;
  return total_dist / 2.0f;
}

float GroveUltrasonicRangerSensorComponent::m_to_us(float m) {
  const float speed_sound_m_per_s = 343.0f;
  const float time_s = m / speed_sound_m_per_s;
  return time_s * 1e6f;
}

float GroveUltrasonicRangerSensorComponent::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace grove_ultrasonic_ranger
}  // namespace esphome
