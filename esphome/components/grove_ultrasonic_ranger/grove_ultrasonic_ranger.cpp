#include "grove_ultrasonic_ranger.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "Arduino.h"

namespace esphome {
namespace grove_ultrasonic_ranger {

static const char *const TAG = "grove_ultrasonic_ranger.sensor";

void GroveUltrasonicRangerSensorComponent::set_pin(InternalGPIOPin *pin) { pin_ = pin; }

void GroveUltrasonicRangerSensorComponent::set_timeout_m(uint32_t timeout_m) { timeout_us_ = m_to_us(timeout_m) * 2; }

void GroveUltrasonicRangerSensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Ultrasonic Sensor...");
  pin_->setup();
  pin_isr_ = pin_->to_isr();
}

void GroveUltrasonicRangerSensorComponent::update() {
  pin_isr_.pin_mode(gpio::FLAG_OUTPUT);
  pin_isr_.digital_write(0);
  delayMicroseconds(2);
  pin_isr_.digital_write(1);
  delayMicroseconds(5);
  pin_isr_.digital_write(0);
  pin_isr_.pin_mode(gpio::FLAG_INPUT);

  const uint32_t threshold = micros() + timeout_us_;

  while (micros() < threshold && pin_isr_.digital_read())
    ;

  while (micros() < threshold && !pin_isr_.digital_read())
    ;

  const uint32_t pulse_start = micros();

  while (micros() < threshold && pin_isr_.digital_read())
    ;

  const uint32_t pulse_end = micros();

  ESP_LOGV(TAG, "Echo took %" PRIu32 "µs", pulse_end - pulse_start);

  if (pulse_end >= threshold) {
    ESP_LOGD(TAG, "'%s' - Distance measurement timed out!", this->name_.c_str());
    this->publish_state(NAN);
  } else {
    float result = us_to_mm(pulse_end - pulse_start);
    ESP_LOGD(TAG, "'%s' - Got distance: %.3f mm", this->name_.c_str(), result);
    this->publish_state(result);
  }
}
void GroveUltrasonicRangerSensorComponent::dump_config() {
  LOG_SENSOR("", "Ultrasonic Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " µs", this->timeout_us_);
  LOG_UPDATE_INTERVAL(this);
}

float GroveUltrasonicRangerSensorComponent::get_setup_priority() const { return setup_priority::DATA; }

float GroveUltrasonicRangerSensorComponent::us_to_mm(uint32_t us) {
  const float speed_sound_m_per_s = 343.0f;
  const float time_s = us / 1e6f;
  const float total_dist = time_s * speed_sound_m_per_s;
  return total_dist / 2.0f * 1000;
}

float GroveUltrasonicRangerSensorComponent::m_to_us(float m) {
  const float speed_sound_m_per_s = 343.0f;
  const float time_s = m / speed_sound_m_per_s;
  return time_s * 1e6f;
}

}  // namespace grove_ultrasonic_ranger
}  // namespace esphome
