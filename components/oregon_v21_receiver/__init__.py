"""Oregon Scientific v2.1 decoder for ESPHome remote_receiver.

Decodes 433.92 MHz Manchester encoded messages from Oregon Scientific sensors
such as the THGR122N (used with the RAR501 weather station) and publishes
temperature, humidity and low battery state per configured channel.
"""

import logging

import esphome.codegen as cg
from esphome.components import binary_sensor, remote_base, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CHANNEL,
    CONF_DEVICES,
    CONF_FILTER,
    CONF_HUMIDITY,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)
import esphome.final_validate as fv

_LOGGER = logging.getLogger(__name__)

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
CONF_VERIFY_BIT_PAIRS = "verify_bit_pairs"
CONF_DIAGNOSTICS = "diagnostics"
CONF_MESSAGES = "messages"
CONF_CHECKSUM_ERRORS = "checksum_errors"
CONF_BIT_PAIR_ERRORS = "bit_pair_errors"
CONF_REJECTED = "rejected"
CONF_ABORTED = "aborted"
DIAGNOSTIC_KEYS = (
    CONF_MESSAGES,
    CONF_CHECKSUM_ERRORS,
    CONF_BIT_PAIR_ERRORS,
    CONF_REJECTED,
    CONF_ABORTED,
)
UNIT_MESSAGES = "messages"

# Oregon v2.1 half bits are about 488 us; RXB6 spaces measure about 475 us.
MAX_RECEIVER_FILTER_US = 400
# Below this the receiver noise fills the capture buffer and messages are split.
MIN_RECOMMENDED_RECEIVER_FILTER_US = 100

oregon_ns = cg.esphome_ns.namespace("oregon_v21_receiver")
OregonV21Receiver = oregon_ns.class_(
    "OregonV21Receiver", cg.PollingComponent, remote_base.RemoteReceiverListener
)
OregonDevice = oregon_ns.class_("OregonDevice")
ManchesterTiming = oregon_ns.namespace("ManchesterDecoder").struct("Timing")


def _migrate_legacy_config(config):
    """Accept or explain the keys used by the first version of this component."""
    legacy_sensors = [
        k for k in ("sensor_ch1", "sensor_ch2", "sensor_ch3") if k in config
    ]
    if legacy_sensors:
        raise cv.Invalid(
            f"'{legacy_sensors[0]}' is no longer supported. Use a 'devices:' list with "
            "'channel:', 'temperature:', 'humidity:' and 'battery_low:' (a binary sensor) "
            "per sensor, see example/oregon-v21-receiver.yaml.",
            path=[legacy_sensors[0]],
        )
    if CONF_LEGACY_RECEIVER_ID in config:
        if CONF_RECEIVER_ID in config:
            raise cv.Invalid(
                f"Use either '{CONF_RECEIVER_ID}' or '{CONF_LEGACY_RECEIVER_ID}', not both"
            )
        # Copy keeps the config type (and with it line information for errors).
        config = config.copy()
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
        config[CONF_MIN_PULSE]
        < config[CONF_LONG_PULSE]
        < config[CONF_MAX_PULSE]
        < config[CONF_END_GAP]
    ):
        raise cv.Invalid(
            "Timing must satisfy min_pulse < long_pulse < max_pulse < end_gap"
        )
    return config


DEVICE_SCHEMA = cv.All(
    cv.Schema(
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
    ),
    cv.has_at_least_one_key(CONF_TEMPERATURE, CONF_HUMIDITY, CONF_BATTERY_LOW),
)

TIMING_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(
                CONF_MIN_PULSE, default="200us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(
                CONF_LONG_PULSE, default="700us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(
                CONF_MAX_PULSE, default="1200us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(
                CONF_END_GAP, default="2500us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(CONF_MIN_PREAMBLE, default=24): cv.int_range(min=8, max=64),
        }
    ),
    _validate_timing,
)

_COUNTER_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_MESSAGES,
    accuracy_decimals=0,
    state_class=STATE_CLASS_TOTAL_INCREASING,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

DIAGNOSTICS_SCHEMA = cv.Schema(
    {cv.Optional(key): _COUNTER_SCHEMA for key in DIAGNOSTIC_KEYS}
)

CONFIG_SCHEMA = cv.All(
    _migrate_legacy_config,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OregonV21Receiver),
            cv.GenerateID(CONF_RECEIVER_ID): cv.use_id(remote_base.RemoteReceiverBase),
            cv.Optional(CONF_DEVICES, default=[]): cv.ensure_list(DEVICE_SCHEMA),
            cv.Optional(CONF_TIMING, default={}): TIMING_SCHEMA,
            cv.Optional(CONF_VERIFY_BIT_PAIRS, default=True): cv.boolean,
            cv.Optional(CONF_DIAGNOSTICS, default={}): DIAGNOSTICS_SCHEMA,
        }
    ).extend(cv.polling_component_schema("60s")),
    _validate_devices,
)


def _final_validate(config):
    """Check that the remote_receiver filter lets Oregon pulses through."""
    fconf = fv.full_config.get()
    try:
        path = fconf.get_path_for_id(config[CONF_RECEIVER_ID])[:-1]
        receiver = fconf.get_config_for_path(path)
    except KeyError:
        return config
    flt = receiver.get(CONF_FILTER)
    if flt is None:
        return config
    filter_us = int(flt.total_microseconds)
    if filter_us >= MAX_RECEIVER_FILTER_US:
        raise cv.Invalid(
            f"remote_receiver filter {filter_us}us removes the Oregon pulses (about 488us); "
            f"use 100us to {MAX_RECEIVER_FILTER_US - 1}us",
            path=[CONF_RECEIVER_ID],
        )
    if filter_us < MIN_RECOMMENDED_RECEIVER_FILTER_US:
        _LOGGER.warning(
            "remote_receiver filter is %sus. With an RXB6 the noise can then fill the "
            "receive buffer and split Oregon messages; 100us is recommended.",
            filter_us,
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


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

    cg.add(var.set_verify_bit_pairs(config[CONF_VERIFY_BIT_PAIRS]))

    for key in DIAGNOSTIC_KEYS:
        if key in config[CONF_DIAGNOSTICS]:
            sens = await sensor.new_sensor(config[CONF_DIAGNOSTICS][key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

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
