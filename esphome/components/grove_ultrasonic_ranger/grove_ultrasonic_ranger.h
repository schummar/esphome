#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include <cinttypes>

namespace esphome {
namespace grove_ultrasonic_ranger {

class GroveUltrasonicRangerSensorComponent : public sensor::Sensor, public PollingComponent {
 public:
  void set_pin(InternalGPIOPin *pin);
  void set_timeout_m(uint32_t timeout_m);

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

 protected:
  static float us_to_m(uint32_t us);
  static float m_to_us(float m);

  InternalGPIOPin *pin_;
  ISRInternalGPIOPin pin_isr_;
  uint32_t timeout_us_{};
};

}  // namespace grove_ultrasonic_ranger
}  // namespace esphome
