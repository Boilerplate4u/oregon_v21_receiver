# oregon_v21_receiver

[![Tests](https://github.com/boilerplate4u/oregon_v21_receiver/actions/workflows/tests.yml/badge.svg)](https://github.com/boilerplate4u/oregon_v21_receiver/actions/workflows/tests.yml)

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
                 ▼            (v2.1 sends each bit with its complement; both are checked)
          parse_oregon_v21    sync nibble, sensor id, checksum, BCD and range checks
                 ▼
          OregonDevice        match channel (and optional rolling code) ─► sensors
```

* `manchester_decoder.*` and `oregon_protocol.*` are plain C++ without ESPHome
  dependencies and are covered by host unit and fuzz tests.
* The component is a `remote_receiver` listener, so it can share the receiver with
  `rc_switch`, `nexa` and other protocols on the same pin.
* A message is accepted only when every bit pair is a bit followed by its complement,
  the Oregon nibble checksum matches and all values are valid BCD within range. In fuzz
  tests with 800 000 damaged captures, 45 damaged messages still had a valid checksum and
  wrong values; the bit pair check rejected 43 of them. The remaining kind of damage keeps
  every bit pair valid, so only the 8 bit checksum protects against it (see
  [Outliers](#outliers)).
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
        filters:
          - timeout: 5min
      humidity:
        name: "Oregon CH1 Humidity"
        filters:
          - timeout: 5min
      battery_low:
        name: "Oregon CH1 Battery Low"
```

A complete example, including the diagnostic counters, is in
[`example/oregon-v21-receiver.yaml`](example/oregon-v21-receiver.yaml).

### Options

| Key | Description |
|---|---|
| `receiver_id` | ID of the `remote_receiver`. `remote_receiver_id` is accepted as an alias. |
| `devices` | List of sensors to publish. Each device needs at least one of `temperature`, `humidity` and `battery_low`. |
| `devices[].channel` | Channel as shown on the base station. The THGR122N transmits 1, 2 and 4 for channels 1, 2 and 3; this is translated, so use 1 to 3. |
| `devices[].rolling_code` | Optional (0 to 255). Only needed when several sensors share a channel. It changes when the batteries are replaced. |
| `devices[].temperature` | Sensor, temperature measured by the outdoor sensor in °C. All sensor options (filters etc.) are available. |
| `devices[].humidity` | Sensor, relative humidity measured by the outdoor sensor in %. |
| `devices[].battery_low` | Binary sensor, on when the sensor reports low battery. |
| `verify_bit_pairs` | Default `true`. Reject messages where a bit and its complement are equal. Set to `false` only if valid messages are rejected; the errors are then still logged. |
| `diagnostics` | Optional counters since boot: `messages`, `checksum_errors`, `bit_pair_errors`, `rejected` and `aborted`. Each takes sensor options such as `name`. |
| `update_interval` | How often the diagnostic counters are published. Default `60s`. |
| `timing` | Advanced pulse classification: `min_pulse` (200us), `long_pulse` (700us), `max_pulse` (1200us), `end_gap` (2500us), `min_preamble` (24). The defaults match rene-d/oregon. |

### Stale values

The sensors transmit every 39 to 43 seconds (measured on a RAR501 setup: channel 1
every 39 s, channel 3 every 43 s). Without a filter Home Assistant keeps showing the
last value if a sensor stops transmitting. ESPHome's `timeout` filter makes the value
unknown instead, as in the examples above.

### Outliers

A damaged message that still passes all checks is rare but possible, since the checksum
is only 8 bits. Where a single wrong value matters, add a median filter. It drops a
single outlier at the cost of reporting the middle of the last three values:

```yaml
      temperature:
        name: "Oregon CH1 Temperature"
        filters:
          - median:
              window_size: 3
              send_every: 1
          - timeout: 5min
```

### Diagnostic counters

| Counter | Meaning |
|---|---|
| `messages` | Valid messages, repeats not counted. |
| `checksum_errors` | Complete messages with a wrong checksum. |
| `bit_pair_errors` | Messages rejected because a bit and its complement were equal. |
| `rejected` | Other rejected messages: unknown sensor id, invalid values, too short. |
| `aborted` | Messages that started after a valid preamble but broke off before the end. |

The RXB6 has no signal strength output, so the ratio of errors to `messages` is the
best available measure of reception quality, for example when moving the antenna.
Plain noise from the receiver is not counted.

### Finding channels and rolling codes

The first message from a sensor without a matching device is logged at INFO level:

```
[I][oregon_v21]: THGR122N/THGR228N on channel 3 with rolling code 245 is not configured; add it under 'devices:' to use it (further messages from it are logged at DEBUG)
```

Every reading is logged at DEBUG level:

```
[D][oregon_v21]: THGR122N/THGR228N channel 3 (raw 4) rolling code 245: 8.1 °C, 75 %, battery ok
```

### Neighbours, battery changes and spoofing

* A device without `rolling_code` accepts every sensor on its channel, including a
  neighbour's. When two rolling codes are heard on such a channel within 10 minutes, a
  warning suggests the `rolling_code` to set. A device with a matching `rolling_code`
  always takes precedence over a device for the whole channel.
* The rolling code changes when the sensor batteries are replaced. A device with a fixed
  `rolling_code` then stops updating (the `timeout` filter shows it as unknown) and the
  new rolling code is logged at INFO level; update the YAML.
* 433 MHz sensors are not authenticated. Anyone nearby can transmit fake readings, so do
  not use them for safety critical automations.

### Receiver settings

The component checks the `remote_receiver` it listens to when the configuration is
validated:

* `filter` of 400us or more is an error, because it removes the Oregon pulses.
* `filter` below 100us gives a warning. RXB6 noise then fills captures up to the
  receiver limit (`receive_symbols`, 192 symbols on the ESP32) and messages risk being
  split. In a 32 second log with a filter of about 10us, 53 of 102 captures hit that
  limit; with 100us no capture came close and every Oregon message arrived complete in
  about 115 symbols.

### Upgrading from the first version

The first version used `remote_receiver_id`, which is still accepted, and
`sensor_ch1` to `sensor_ch3`, which are not. Replace them with a `devices:` list as in the
example. `battery` (a percentage) is now `battery_low` (a binary sensor), because the
sensors only report a low battery flag.

## Logging

| Level | Output |
|---|---|
| WARN | Two rolling codes on a channel that has no `rolling_code` configured (see below). |
| INFO | The first message from each sensor that is not configured, with its channel and rolling code. |
| DEBUG | Every decoded reading, the raw bytes of every message and the reason when a message is rejected. |
| VERBOSE | Dump of every Manchester message: byte count, bit pair errors, average short and long pulse width, the bytes and all raw bit pairs grouped per nibble. Also messages abandoned after the preamble and ignored repeats. |

Example of the VERBOSE dump:

```
[V][oregon_v21]: Manchester message: 10 bytes, 0 bit pair errors, short 488 us (n=75), long 976 us (n=122)
[V][oregon_v21]:   bytes: 1A 2D 40 F5 10 08 50 C7 49 C4
[V][oregon_v21]:   raw nibbles  0- 7: 01100110 10010101 10011010 01100101 01010101 01011001 10011001 10101010
[V][oregon_v21]:   raw nibbles  8-15: 01010101 10010101 01010110 01010101 01010101 10011001 10101001 01011010
[V][oregon_v21]:   raw nibbles 16-19: 10010110 01011001 01011001 01011010
```

Each group of 8 raw bits is one nibble sent as four bit pairs, least significant bit
first: `01100110` is the pairs `01 10 01 10`, which gives the bits 0 1 0 1, so the sync
nibble 0xA. The output above comes from a real RXB6 capture in `tests/data`.

The pulse width averages help tune `timing` if a receiver produces unusual pulse widths.
The pulse trains themselves are logged by `remote_receiver` with `dump: raw`.

## ESPHome versions

Compiled for ESP32 with ESP-IDF using ESPHome **2026.9.1** and **2026.10.0b4**. From
2026.10 listeners must be registered through `remote_base.add_listener`; the component
uses it when available and falls back to `register_listener` on older versions.

## Tests

`tests/run_tests.sh` builds and runs three test programs with AddressSanitizer and UBSan
(Debian/WSL, Linux or macOS):

* `test_component` runs the ESPHome glue (device matching, repeats, warnings, counters,
  VERBOSE dump) against minimal ESPHome header stubs in `tests/stubs`, with the same
  captures. The real ESPHome API is covered by the compile in GitHub Actions.
* `test_decoder` covers the parser and decoder with real pulse captures from an RXB6
  (`tests/data/rxb6_raw_frames.txt`) and synthetic messages (negative temperature, low
  battery, temperature only sensor, bit pair errors, two messages in one capture).
* `fuzz_decoder` feeds random pulse trains, random byte buffers and damaged versions of
  the captures into the decoder with a fixed seed, and checks that no damaged capture is
  accepted with wrong values.

```sh
tests/run_tests.sh        # a few seconds
tests/run_tests.sh 10     # ten times more fuzz iterations
```

GitHub Actions runs the tests and compiles the example for ESP32 with ESPHome 2026.9.1,
the latest release and the dev branch on every push and pull request
([`.github/workflows/tests.yml`](.github/workflows/tests.yml)).

## Credits

* Pulse state machine ported from [rene-d/oregon](https://github.com/rene-d/oregon)
  (released into the public domain), based on the OOK decoder by Jean-Claude Wippler
  (MIT) with Oregon v2 additions by Dominique Pierre and Olivier Lebrun.
* Sensor ids, field layout and checksum ranges cross checked against
  [rtl_433](https://github.com/merbanan/rtl_433) `src/devices/oregon_scientific.c`.
