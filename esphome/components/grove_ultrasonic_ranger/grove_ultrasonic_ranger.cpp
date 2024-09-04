#include "grove_ultrasonic_ranger.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "Arduino.h"

namespace esphome {
namespace grove_ultrasonic_ranger {

static const char *const TAG = "grove_ultrasonic_ranger.sensor";

void GroveUltrasonicRangerSensorComponent::set_pin(GPIOPin *pin) { pin_ = pin; }

void GroveUltrasonicRangerSensorComponent::set_timeout_m(uint32_t timeout_m) {
  ESP_LOGD(TAG, "Setting timeout. m=%, us=%", timeout_m, m_to_us(timeout_m));
  this->timeout_m_ = timeout_m;
  timeout_us_ = m_to_us(timeout_m) * 2;
  timeout_us_ = 1000000L;
}

void GroveUltrasonicRangerSensorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Ultrasonic Sensor...");
  pin_->setup();
  // pin_->digital_write(false);
  // pin_isr_ = pin_->to_isr();
}

static uint32_t MicrosDiff(uint32_t begin, uint32_t end) { return end - begin; }

static uint32_t pulseIn(GPIOPin *pin, uint32_t state, uint32_t timeout = 1000000L) {
  uint32_t begin = micros();

  // wait for any previous pulse to end
  while (pin->digital_read())
    if (MicrosDiff(begin, micros()) >= timeout)
      return 0;

  // wait for the pulse to start
  while (!pin->digital_read())
    if (MicrosDiff(begin, micros()) >= timeout)
      return 0;
  uint32_t pulseBegin = micros();

  // wait for the pulse to stop
  while (pin->digital_read())
    if (MicrosDiff(begin, micros()) >= timeout)
      return 0;
  uint32_t pulseEnd = micros();

  return MicrosDiff(pulseBegin, pulseEnd);
}

void GroveUltrasonicRangerSensorComponent::update() {
  // int _pin = 20;

  // pinMode(_pin, OUTPUT);
  // digitalWrite(_pin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(_pin, HIGH);
  // delayMicroseconds(5);
  // digitalWrite(_pin, LOW);
  // pinMode(_pin, INPUT);
  // long duration;
  // duration = pulseIn(_pin, HIGH);
  // long RangeInCentimeters;
  // RangeInCentimeters = duration / 29 / 2;

  // ESP_LOGD(TAG, "Distance: %d cm", RangeInCentimeters);

  GPIOPin *pin = pin_;
  pin->pin_mode(gpio::FLAG_OUTPUT);
  pin->digital_write(0);
  delayMicroseconds(2);
  pin->digital_write(1);
  delayMicroseconds(5);
  pin->digital_write(0);
  pin->pin_mode(gpio::FLAG_INPUT);

  // long duration;
  // duration = pulseIn(pin, HIGH);
  // long RangeInCentimeters;
  // RangeInCentimeters = duration / 29 / 2;
  // ESP_LOGD(TAG, "Distance: %d cm", RangeInCentimeters);

  const uint32_t start = micros();
  while (micros() - start < timeout_us_ && pin->digital_read())
    ;
  ESP_LOGD(TAG, "state=%d", pin->digital_read());

  while (micros() - start < timeout_us_ && !pin->digital_read())
    ;
  ESP_LOGD(TAG, "state=%d", pin->digital_read());

  const uint32_t pulse_start = micros();
  while (micros() - start < timeout_us_ && pin->digital_read())
    ;
  ESP_LOGD(TAG, "state=%d", pin->digital_read());

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
  ESP_LOGCONFIG(TAG, "  Timeout m: %" PRIu32 " m", this->timeout_m_);
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
