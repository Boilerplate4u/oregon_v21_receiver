"""Oregon Scientific v2.1 decoder for ESPHome remote_receiver.

Decodes 433.92 MHz Manchester encoded messages from Oregon Scientific sensors
such as the THGR122N (used with the RAR501 weather station) and publishes
temperature, humidity and low battery state per configured channel.
"""

import esphome.codegen as cg
from esphome.components import binary_sensor, remote_base, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CHANNEL,
    CONF_DEVICES,
    CONF_HUMIDITY,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@Boilerplate4U"]
DEPENDENCIES = ["remote_receiver"]
AUTO_LOAD = ["sensor", "binary_sensor"]

CONF_RECEIVER_ID = "receiver_id"
CONF_LEGACY_RECEIVER_ID = "remote_receiver_id"
CONF_ROLLING_CODE = "rolling_code"
CONF_BATTERY_LOW = "battery_low"
CONF_TIMING = "timing"
CONF_MIN_PULSE = "min_pulse"
CONF_LONG_PULSE = "long_pulse"
CONF_MAX_PULSE = "max_pulse"
CONF_END_GAP = "end_gap"
CONF_MIN_PREAMBLE = "min_preamble"

oregon_ns = cg.esphome_ns.namespace("oregon_v21_receiver")
OregonV21Receiver = oregon_ns.class_(
    "OregonV21Receiver", cg.Component, remote_base.RemoteReceiverListener
)
OregonDevice = oregon_ns.class_("OregonDevice")
ManchesterTiming = oregon_ns.namespace("ManchesterDecoder").struct("Timing")


def _rename_legacy_receiver_key(config):
    """Accept the key used by the first version of this component."""
    if CONF_LEGACY_RECEIVER_ID in config:
        if CONF_RECEIVER_ID in config:
            raise cv.Invalid(
                f"Use either '{CONF_RECEIVER_ID}' or '{CONF_LEGACY_RECEIVER_ID}', not both"
            )
        config = dict(config)
        config[CONF_RECEIVER_ID] = config.pop(CONF_LEGACY_RECEIVER_ID)
    return config


def _validate_devices(config):
    seen = set()
    for device in config.get(CONF_DEVICES, []):
        key = (device[CONF_CHANNEL], device.get(CONF_ROLLING_CODE))
        if key in seen:
            raise cv.Invalid(
                f"Duplicate device for channel {key[0]}"
                + (f" and rolling code {key[1]}" if key[1] is not None else "")
            )
        seen.add(key)
    return config


def _validate_timing(config):
    if not (
        config[CONF_MIN_PULSE] < config[CONF_LONG_PULSE] < config[CONF_MAX_PULSE]
        < config[CONF_END_GAP]
    ):
        raise cv.Invalid(
            "Timing must satisfy min_pulse < long_pulse < max_pulse < end_gap"
        )
    return config


DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OregonDevice),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=15),
        cv.Optional(CONF_ROLLING_CODE): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

TIMING_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_MIN_PULSE, default="200us"): cv.positive_time_period_microseconds,
            cv.Optional(CONF_LONG_PULSE, default="700us"): cv.positive_time_period_microseconds,
            cv.Optional(CONF_MAX_PULSE, default="1200us"): cv.positive_time_period_microseconds,
            cv.Optional(CONF_END_GAP, default="2500us"): cv.positive_time_period_microseconds,
            cv.Optional(CONF_MIN_PREAMBLE, default=24): cv.int_range(min=8, max=64),
        }
    ),
    _validate_timing,
)

CONFIG_SCHEMA = cv.All(
    _rename_legacy_receiver_key,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OregonV21Receiver),
            cv.GenerateID(CONF_RECEIVER_ID): cv.use_id(
                remote_base.RemoteReceiverBase
            ),
            cv.Optional(CONF_DEVICES, default=[]): cv.ensure_list(DEVICE_SCHEMA),
            cv.Optional(CONF_TIMING, default={}): TIMING_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_devices,
)


def _us(value):
    return int(value.total_microseconds)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
    if hasattr(remote_base, "add_listener"):
        # ESPHome 2026.10 and later count listener slots at code generation.
        remote_base.add_listener(receiver, var)
    else:
        cg.add(receiver.register_listener(var))

    timing = config[CONF_TIMING]
    cg.add(
        var.set_timing(
            cg.StructInitializer(
                ManchesterTiming,
                ("min_pulse_us", _us(timing[CONF_MIN_PULSE])),
                ("long_pulse_us", _us(timing[CONF_LONG_PULSE])),
                ("max_pulse_us", _us(timing[CONF_MAX_PULSE])),
                ("end_gap_us", _us(timing[CONF_END_GAP])),
                ("min_preamble", timing[CONF_MIN_PREAMBLE]),
                ("min_bytes", 8),
            )
        )
    )

    for device_config in config[CONF_DEVICES]:
        device = cg.new_Pvariable(device_config[CONF_ID])
        cg.add(device.set_channel(device_config[CONF_CHANNEL]))
        if CONF_ROLLING_CODE in device_config:
            cg.add(device.set_rolling_code(device_config[CONF_ROLLING_CODE]))
        if CONF_TEMPERATURE in device_config:
            sens = await sensor.new_sensor(device_config[CONF_TEMPERATURE])
            cg.add(device.set_temperature_sensor(sens))
        if CONF_HUMIDITY in device_config:
            sens = await sensor.new_sensor(device_config[CONF_HUMIDITY])
            cg.add(device.set_humidity_sensor(sens))
        if CONF_BATTERY_LOW in device_config:
            bsens = await binary_sensor.new_binary_sensor(
                device_config[CONF_BATTERY_LOW]
            )
            cg.add(device.set_battery_low_binary_sensor(bsens))
        cg.add(var.add_device(device))
