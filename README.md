# ESP32 Sensor + LED Hub

Single-ESP32 firmware (`Esp32_Sensor_Hub.ino`) that replaces the sensor side of the old Teensy sketch plus the two separate ESP32 boards (data hub and LED/WCD).

The ZLAC motor driver talks to the Jetson directly, so this firmware does **no** motor control, PID or encoder reading.

> Status: written against the Teensy sketch and the block diagram. Not yet compiled or tested on hardware. Build it once and check the items in [Before first run](#before-first-run).

## What it does

| Function | Details |
| --- | --- |
| IMU | BMI088 gyro over I2C, 200 Hz, yaw integrated on the ESP (2 s bias calibration at boot) |
| Power monitoring | MCP3208 ADC over SPI: battery voltage, current, battery %, charger detection (>0.3 A) |
| E-stop feedback | Debounced (50 ms), sends `estop` / `estart` events |
| Main switch sequence | Main switch off + e-stop pressed: 5x `stop` to the Jetson, relay cut after 15 s |
| Ultrasonics | 4 sensors, interrupt-timed echoes, median-of-3 filter, ~50 Hz each |
| Cliff sensors | 2 digital inputs |
| IR docking array | Raw digital pin levels (placeholder, see below) |
| LED trays | 3 x 100 WS2812B strips, patterns commanded over USB |
| Waiter Call Device | ESP-NOW link, forwarded to/from the Jetson over USB serial |
| Link to Jetson | USB serial, 921600 baud, text lines |

## Hardware

### Libraries (Arduino Library Manager)

- `BMI088` by Bolder Flight Systems
- `Mcp320x` by labfruits
- `FastLED`

Board: **ESP32 Dev Module** (arduino-esp32 core 2.x or 3.x).

### Pin map (classic ESP32 DevKit)

Change any of these in the "Pins" block at the top of the sketch.

| Function | Pin(s) | Notes |
| --- | --- | --- |
| I2C (BMI088) | SDA 21, SCL 22 | Keep on defaults, the BMI088 library calls `Wire.begin()` itself. Gyro address 0x69 |
| SPI (MCP3208) | SCK 18, MISO 19, MOSI 23, CS 5 | Channels: 6 = current, 7 = current ref, 4 = voltage (same as Teensy) |
| E-stop feedback | 32 | Internal pull-up. 1 = released, 0 = pressed |
| Main switch feedback | 33 | Internal pull-up |
| Main relay | 16 | Driven high at boot, cut by the shutdown sequence |
| Ultrasonic 1 (HC-SR04) | TRIG 12, ECHO 14 | From the Ultrasonic_TestCOde repo |
| Ultrasonic 2 (AJ-SR04M) | TRIG 26, ECHO 25 | From the Ultrasonic_TestCOde repo |
| Ultrasonic 3 | TRIG 27, ECHO 36 | New, assumed HC-SR04 type |
| Ultrasonic 4 | TRIG 4, ECHO 39 | New, assumed HC-SR04 type |
| Cliff left / right | 34 / 35 | Input-only pins, **no internal pull-up** |
| IR docking array | 17 | **Placeholder**, set `IR_PINS` to your array |
| LED tray 1 / 2 / 3 | 13 / 15 / 2 | WS2812B data lines |

### Wiring notes

- **ESP32 GPIOs are 3.3 V only.** Sensors powered from 5 V need a divider on every ECHO line:

  ```
  sensor ECHO --[ 1k ]--+-- ESP32 GPIO
                        +--[ 2k ]-- GND
  ```

- **Strapping pins:** GPIO 2, 12 and 15 affect boot. If the board fails to boot with a sensor or LED strip attached, move that line to another pin.
- **AJ-SR04M:** must be in its default trigger/echo mode. It cannot measure closer than about 20 cm.
- **LED power:** 300 LEDs can draw many amps at full white. Power the strips from a proper 5 V supply (not the ESP's USB), share the ground with the ESP, and keep that supply separate from the MCP3208 reference to avoid noisy current and voltage readings. Set `LED_MAX_MA` to what your supply can give; global brightness is limited by `LED_BRIGHTNESS` (default 80/255).
- **WCD:** put the device's MAC address in `WCD_MAC` (default is broadcast). The WCD must use the same WiFi channel as the ESP, which is channel 1 by default.

## Build and flash

1. Install the three libraries above.
2. Open `Esp32_Sensor_Hub.ino` in the Arduino IDE (board: ESP32 Dev Module) or compile with `arduino-cli`.
3. Flash, then open the serial monitor at **921600** baud.
4. Keep the robot still for the first ~2 seconds while the gyro calibrates.

## USB serial protocol

921600 baud, text lines ending in `\n`. Lines starting with `#` are debug and can be ignored by the Jetson.

### ESP to Jetson

Sensor packet, sent at `SENSOR_RATE_HZ` (default 100):

```
d,<seq>,<t_ms>,<yaw>,<batt%>,<estop>,<charging>,<volts>,<amps>,<us1>,<us2>,<us3>,<us4>,<cliffL>,<cliffR>,<ir_bits>
```

| Field | Meaning |
| --- | --- |
| `seq` | Packet counter |
| `t_ms` | ESP uptime in ms |
| `yaw` | `(degrees + 180) * 100`, so 0..36000. `44444` means gyro error (same as the Teensy) |
| `batt%` | Battery percent, 0 to 100 |
| `estop` | 1 = released, 0 = pressed |
| `charging` | 1 = battery current above 0.3 A |
| `volts`, `amps` | Filtered battery voltage and current |
| `us1`..`us4` | Filtered distance in mm, `-1` = no echo / out of range |
| `cliffL`, `cliffR` | 1 = cliff detected |
| `ir_bits` | Bit i = raw level of `IR_PINS[i]` |

Status and other lines:

| Line | When |
| --- | --- |
| `s,estop` | E-stop pressed for over 200 ms, repeated every 5 s while pressed |
| `s,estart` | E-stop released |
| `s,stop` | Main switch shutdown sequence (sent 5 times) |
| `s,charger_on` / `s,charger_off` | Charger connected / disconnected |
| `w,<mac>,<payload>` | Message from the WCD. Payload is text if printable, otherwise `0x` plus hex |
| `ack,led`, `ack,r` | Command accepted |
| `# ...` | Debug and stats |

### Jetson to ESP

| Command | Effect |
| --- | --- |
| `led,<tray>,<pattern>,<r>,<g>,<b>` | Tray 0 = all trays, 1 to 3 = one tray. Patterns: 0 off, 1 solid, 2 blink, 3 breathe, 4 chase, 5 rainbow |
| `w,<text>` | Send the text to the WCD over ESP-NOW (up to 200 bytes) |
| `r` | Reset yaw to 0 |

The LED and WCD formats are defaults chosen for this firmware. Change `handleLine()` if your Jetson node already uses something else.

### Reading the packet on the Jetson (Python)

```python
import serial

ser = serial.Serial("/dev/ttyUSB0", 921600, timeout=1)
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("d,"):
        f = line.split(",")
        yaw_deg = int(f[3]) / 100.0 - 180.0
        us_mm = [int(x) for x in f[9:13]]
        print(yaw_deg, us_mm)
    elif line.startswith("s,"):
        print("status:", line[2:])

# Send an LED command (tray 1, solid green):
# ser.write(b"led,1,1,0,255,0\n")
```

## How it is organized

| Core | Task | Rate | Job |
| --- | --- | --- | --- |
| 1 | `fast` | 200 Hz | Gyro, ADC, e-stop, cliff, IR, main-switch sequence |
| 1 | `us` | 100 Hz slots | Fires and collects the ultrasonics (non-blocking) |
| 0 | `comms` | `SENSOR_RATE_HZ` | Packet output, command parsing, status and WCD forwarding |
| 0 | `led` | 50 fps | Renders LED patterns, pushes to the strips only when needed |

Ultrasonic timing: sensors are split into two slots by `US_SLOT` (default 1+3 together, then 2+4). Each slot is 10 ms, so every sensor is measured at about 50 Hz with a maximum range of roughly 1.6 m. Put sensors that face each other in different slots to avoid hearing each other's pings. The 100 Hz packet repeats the latest ultrasonic values between measurements.

## Stats log

Every 2 seconds (`STATS_PERIOD_MS`), unless `STATS_ENABLED` is false:

```
# STATS fast=200.0Hz max_us=1450 late=0 | tx=100.0Hz drops=0 cmds=3 wcd_rx=0 | us_ok=98,99,100,97
```

(The numbers above only illustrate the format.)

- `fast`: rate of the 200 Hz sensor loop, `max_us`: its longest iteration, `late`: iterations more than 1.5 ms late. `late` should stay at 0.
- `tx`: packet rate to the Jetson, `drops`: lines dropped because the serial buffer was full (the Jetson is not reading fast enough).
- `us_ok`: for each ultrasonic, valid echoes out of measurements in the last window (shown as `ok/total`).

## Before first run

- [ ] Pins in the pin block match your wiring.
- [ ] 1k/2k dividers on all four ultrasonic ECHO lines.
- [ ] `IR_PINS` set for your docking array (or its I2C/UART interface added).
- [ ] `WCD_MAC` set and the WCD on WiFi channel 1.
- [ ] `LED_MAX_MA` matches your 5 V supply.
- [ ] Jetson reads the `d,` packet over USB serial (the Teensy used UDP) and refuses motion while `charging` is 1.
- [ ] Gyro bias line appears at boot (`# gyro bias z: ...`). If you see `BMI088 not found`, yaw reports 44444.

## Known limitations

- **LED flicker:** on the classic ESP32, heavy WiFi/ESP-NOW traffic can make WS2812 strips flicker. An ESP32-S3 or a DMA/I2S-based LED driver avoids it.
- **Pin count:** the classic ESP32 is nearly out of usable pins with this setup. More sensors, or an IR array needing several pins, probably means an ESP32-S3.
- **Ultrasonic range:** limited to about 1.6 m by the 10 ms slot, and the AJ-SR04M may be less reliable at this rate because its transducer needs time to stop ringing.
- **Safety:** the physical e-stop must stay hardware-wired to the contactor. This firmware only reports e-stop state.
