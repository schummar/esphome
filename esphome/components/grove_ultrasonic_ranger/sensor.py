from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_PIN,
    CONF_TIMEOUT,
    ICON_ARROW_EXPAND_VERTICAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
)

grove_ultrasonic_ranger_ns = cg.esphome_ns.namespace("grove_ultrasonic_ranger")
GroveUltrasonicRangerSensorComponent = grove_ultrasonic_ranger_ns.class_(
    "GroveUltrasonicRangerSensorComponent", sensor.Sensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        GroveUltrasonicRangerSensorComponent,
        unit_of_measurement=UNIT_MILLIMETER,
        icon=ICON_ARROW_EXPAND_VERTICAL,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_TIMEOUT, default="2m"): cv.distance,
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    trigger = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(trigger))

    cg.add(var.set_timeout_m(config[CONF_TIMEOUT]))
