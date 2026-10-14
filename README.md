# oregon_v21_receiver

ESPHome external component that decodes **Oregon Scientific v2.1** 433.92 MHz sensors
(for example the **THGR122N** used with the **RAR501** weather station) from the pulse
stream of a standard `remote_receiver`, such as an RXB6 superheterodyne receiver on an ESP32.

Temperature, humidity and low battery state are published to Home Assistant per channel.

## How it works

```
RXB6 DATA ─► remote_receiver (RMT, raw pulse widths)
                 │  listener callback, same stream as rc_switch, nexa, ...
                 ▼
          ManchesterDecoder   pulse widths ─► Manchester bits ─► bytes
                 ▼            (keeps every second bit, v2.1 sends each bit twice)
          parse_oregon_v21    sync nibble, sensor id, checksum, BCD values
                 ▼
          OregonDevice        match channel (and optional rolling code) ─► sensors
```

* `manchester_decoder.*` and `oregon_protocol.*` are plain C++ without ESPHome
  dependencies and are covered by host unit tests.
* The component is a `remote_receiver` listener, so it can share the receiver with
  `rc_switch`, `nexa` and other protocols on the same pin.
* Every message is validated with the Oregon nibble checksum, BCD checks and range
  checks. Frames that are not Oregon messages are ignored.
* Oregon sensors send each message twice. The identical copy within 1.5 s is skipped.

## Supported sensors

| Sensor id | Model | Values |
|---|---|---|
| 0x1D20 | THGR122N, THGR228N | temperature, humidity, battery |
| 0x1D30 | THGR968 | temperature, humidity, battery |
| 0xEC40 | THN132N, THR228N | temperature, battery |

Other sensor ids are logged at DEBUG level as `unknown sensor id` together with the raw
message, which makes it straightforward to add them.

## Wiring (ESP32 DevKit / ESP32-WROOM-32D)

| RXB6 | ESP32 |
|---|---|
| VCC | 5V |
| GND | GND |
| DATA | GPIO17 |
| ANT | 17.3 cm wire (quarter wave) |

## Configuration

```yaml
remote_receiver:
  id: rf_in
  pin: GPIO17
  tolerance: 25%
  filter: 100us        # must stay well below the 488 us Oregon half bit
  idle: 4ms
  buffer_size: 10kb

external_components:
  - source: github://boilerplate4u/oregon_v21_receiver@main
    components: [oregon_v21_receiver]

oregon_v21_receiver:
  receiver_id: rf_in
  devices:
    - channel: 1
      temperature:
        name: "Oregon CH1 Temperature"
      humidity:
        name: "Oregon CH1 Humidity"
      battery_low:
        name: "Oregon CH1 Battery Low"
```

A complete example is in [`example/oregon-v21-receiver.yaml`](example/oregon-v21-receiver.yaml).

### Options

| Key | Description |
|---|---|
| `receiver_id` | ID of the `remote_receiver`. `remote_receiver_id` is accepted as an alias. |
| `devices` | List of sensors to publish. |
| `devices[].channel` | Channel as shown on the base station. The THGR122N transmits 1, 2 and 4 for channels 1, 2 and 3; this is translated, so use 1 to 3. |
| `devices[].rolling_code` | Optional (0 to 255). Only needed when several sensors share a channel. It changes when the batteries are replaced. |
| `devices[].temperature` | Sensor, outdoor sensor air temperature in °C. All sensor options (filters etc.) are available. |
| `devices[].humidity` | Sensor, relative humidity in %. |
| `devices[].battery_low` | Binary sensor, on when the sensor reports low battery. |
| `timing` | Advanced pulse classification: `min_pulse` (200us), `long_pulse` (700us), `max_pulse` (1200us), `end_gap` (2500us), `min_preamble` (24). The defaults match rene-d/oregon. |

### Finding channels and rolling codes

Every decoded message is logged at INFO level, also when no device matches:

```
[I][oregon_v21]: THGR122N/THGR228N channel 3 (raw 4) rolling code 245: 8.1 °C, 75 %, battery ok
[I][oregon_v21]: No device configured for channel 3 rolling code 245 (add it under 'devices:' to use it)
```

### ESPHome versions

Compiled for ESP32 with ESP-IDF using ESPHome **2026.9.1** and **2026.10.0b4**. From
2026.10 listeners must be registered through `remote_base.add_listener`; the component
uses it when available and falls back to `register_listener` on older versions.

## Tests

The host tests use real pulse captures from an RXB6 (`tests/data/rxb6_raw_frames.txt`)
and synthetic messages (negative temperature, low battery, temperature only sensor).

```sh
cd tests
g++ -std=c++17 -Wall -Wextra -I../components/oregon_v21_receiver test_decoder.cpp \
    ../components/oregon_v21_receiver/manchester_decoder.cpp \
    ../components/oregon_v21_receiver/oregon_protocol.cpp -o test_decoder
./test_decoder
```

## Credits

* Pulse state machine ported from [rene-d/oregon](https://github.com/rene-d/oregon)
  (released into the public domain), based on the OOK decoder by Jean-Claude Wippler
  (MIT) with Oregon v2 additions by Dominique Pierre and Olivier Lebrun.
* Sensor ids, field layout and checksum ranges cross checked against
  [rtl_433](https://github.com/merbanan/rtl_433) `src/devices/oregon_scientific.c`.
