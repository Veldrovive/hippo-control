# Hippo Control

Firmware and a browser control panel for a gimbal BLDC motor driven by an ESP32 + SimpleFOC Mini, with an AS5600 magnetic encoder. The motor is run in closed-loop velocity control and can be commanded over USB serial or Bluetooth LE from a web page.

```
firmware/
  encoder_test/            1. read the encoder only (motor off)
  constant_velocity_test/  2. spin the motor, tune over serial
  ble_velocity_control/    3. full control over Bluetooth LE (SimpleFOC's velocity loop)
  simple_velocity_control/ 4. our own simple PI loop in volts + BLE  <- recommended
  driver_test/             diagnostics: driver, motor wiring, encoder (no FOC)
webapp/
  index.html               Web Bluetooth control panel (single file, no build)
```

Work through the sketches in order the first time you set up a board: each one checks one more piece of the hardware.

---

## Hardware

| Part | Notes |
|---|---|
| ESP32 dev board | "ESP32 Dev Module" (e.g. HiLetgo ESP32 DevKit) |
| SimpleFOC Mini | 3-phase BLDC driver, 5–35 V input |
| Gimbal BLDC motor | 11 pole pairs, 7.2 Ω phase resistance, KV 61 RPM/V, 2.6 mH |
| AS5600 | I2C magnetic encoder, diametric magnet on the motor shaft |
| Bench supply | 24 V (12 V also works; see [Supply voltage](#supply-voltage)) |

### Wiring

| ESP32 pin | Connects to |
|---|---|
| GPIO 14 | SimpleFOC Mini IN1 |
| GPIO 25 | SimpleFOC Mini IN2 |
| GPIO 26 | SimpleFOC Mini IN3 |
| GPIO 27 | SimpleFOC Mini EN |
| GPIO 21 (SDA) | AS5600 SDA |
| GPIO 22 (SCL) | AS5600 SCL |
| 3V3 | AS5600 VCC |
| GND | AS5600 GND, SimpleFOC Mini GND, supply GND (**all grounds common**) |

The motor's three phase wires go to the SimpleFOC Mini's motor outputs; the bench supply goes to the Mini's power input. The pins match the earlier PID teaching tool project, so the same board can run both.

### Powering the ESP32

The ESP32 is normally powered over USB while you're developing.

**Do not power the ESP32 from the SimpleFOC Mini's 3V3 pin.** That regulator is rated for 20 mA; an ESP32 draws ~100 mA running and 300–500 mA spikes when the radio transmits, so it will brown out or overheat the regulator. (The Mini's 3V3 is fine for the AS5600, which draws ~6.5 mA.)

To run everything from the one bench supply, use a small buck converter (24 V → 5 V, ≥ 500 mA) into the dev board's **5V/VIN** pin. Never put 24 V straight into VIN: the dev board's own regulator (usually an AMS1117) is only rated for about 15 V input.

---

## Software setup

### Clone the repo

```bash
git clone https://github.com/Veldrovive/hippo-control.git
cd hippo-control/
```

The repo is private, so ask for access if the clone fails.

### Arduino IDE

Install the [Arduino IDE](https://www.arduino.cc/en/software/).

### ESP32 board package

1. Open Preferences (File > Preferences on Windows, Arduino IDE > Settings on macOS).
2. Under "Additional Boards Manager URLs", add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`.
3. Click OK.
4. Go to Tools > Board > Boards Manager.
5. Search for "esp32" and install **esp32 by Espressif Systems** (tested with 3.3.11).
6. Go to Tools > Board > esp32 and select **ESP32 Dev Module**.

The Bluetooth headers (`BLEDevice.h`, `BLEServer.h`, `BLEUtils.h`, `BLE2902.h`) come with this board package; no separate BLE library is needed.

### SimpleFOC

Complete install instructions are [here](https://docs.simplefoc.com/library_download).

1. Open the Library Manager: Tools > Manage Libraries...
2. Search for **Simple FOC** and install it (tested with 2.4.0).

### Opening a sketch

File > Open, then pick the `.ino` file inside its folder, e.g. `firmware/encoder_test/encoder_test.ino`. The Arduino IDE needs each `.ino` to stay in a folder with the same name, so don't move or rename them.

Upload, then open Tools > Serial Monitor (or Serial Plotter) at **115200 baud**.

---

## Firmware

### 1. `encoder_test`: check the encoder

Reads the AS5600 and streams the angle for the Serial Plotter. The motor driver is held disabled, so you can turn the shaft by hand.

- `angle_deg`: multi-turn shaft angle. Should track smoothly and keep counting past 360°.
- `mech_deg`: single-turn angle, 0–360. Should wrap back to 0.

If nothing answers on I2C you'll see `# WARNING: AS5600 not found at 0x36`. If a line jumps around or stays flat, check the magnet gap and the wiring.

### 2. `constant_velocity_test`: spin the motor

Runs SimpleFOC closed-loop velocity control and lets you tune it live over serial. At startup the motor twitches while SimpleFOC aligns the encoder to the motor (see [Startup alignment](#startup-alignment)); check the status line afterwards reads `motor_status=4`.

Serial commands, one per line:

| Command | Effect |
|---|---|
| `V<rad/s>` | target velocity, e.g. `V2`, `V-5`, `V0` |
| `P<val>` | velocity loop P gain (A per rad/s) |
| `I<val>` | velocity loop I gain (also clears the integrator) |
| `F<sec>` | velocity low-pass filter time constant |
| `?` | print the current settings |

Every command is echoed back (`# rx "V2"`) with the resulting settings. The Serial Plotter shows `target_vel`, `shaft_vel`, and `sensor_vel` (raw encoder speed, independent of the motor setup).

### 3. `ble_velocity_control`: Bluetooth control

The same motor setup as `constant_velocity_test`, plus a Bluetooth LE service so the [web app](#web-app) can drive it. The board advertises as **"Hippo"**. Serial commands still work, plus `E1` / `E0` to enable/disable the motor.

**Safety:** if the Bluetooth connection drops, the target velocity is set to 0 and the board starts advertising again.

See [BLE protocol](#ble-protocol) if you want to write your own client.

### 4. `simple_velocity_control`: simple PI loop (recommended)

Same Bluetooth protocol and web app as `ble_velocity_control`, but the control loop is ours and fits on one screen (`controlStep()` at the top of the sketch). SimpleFOC only reads the encoder and turns "apply U volts" into phase voltages. Nothing is estimated, so the voltage you see is exactly what the motor gets.

```cpp
error     = target_velocity - velocity;
integral += Ki * error * dt;          // clamped to ±voltage_limit (anti-windup)
voltage   = Kp * error + integral;    // clamped to ±voltage_limit
```

**Voltage limit = torque limit.** At standstill, current = V / 7.2 Ω, so 12 V gives ~1.7 A (strong, but the motor heats up quickly if held stalled). `ble_velocity_control` capped this at 1 A estimated (~7 V), which is why it felt weak.

Extra commands on top of `V`, `P`, `I`, `F`, `E`:

| Command | Effect |
|---|---|
| `U<volts>` | direct voltage mode: constant torque with no loop, e.g. `U3`. `V…` switches back to velocity mode. |
| `L<volts>` | voltage (torque) limit, up to 13.5 V on 24 V |

P and I are in volts here: P is V per rad/s (default 0.3), I is V per rad/s per second (default 3.0). The web app has a "Torque test" box for `U` and `L`.

Velocity is computed over a 2 ms window (`VEL_SAMPLE_TIME`) instead of SimpleFOC's 0.1 ms default. One count of AS5600 flicker over 0.1 ms reads as several rad/s of fake velocity; over 2 ms it's ~20x smaller.

### Diagnostics: `driver_test`

When something isn't working, this sketch tests each piece on its own, without the FOC loop or the startup alignment. Run the steps in order; the first one that fails is where the problem is.

| Step | Command | What should happen | If it doesn't |
|---|---|---|---|
| 1. Driver outputs | `h` (all phases at half supply) | Multimeter (DC) from each motor output to GND reads ~half the supply (12 V on 24 V). Unplug the motor first for the cleanest reading. | Driver not powered, EN (GPIO 27) not wired, an IN pin miswired, or the driver is damaged. If one output differs, check that IN pin. |
| 2. Each phase | `a`, then `b`, then `c` (3 V on one phase) | Motor snaps to and holds a position; each phase holds a **different** position. | No hold on any: motor not connected or driver not switching. One phase dead: that motor wire or driver output. |
| 3. Open-loop spin | `o5` (5 rad/s, encoder not used) | Motor turns smoothly. | Stutters or vibrates in place: a phase is missing or miswired (recheck step 2). |
| 4. Encoder | while `o5` runs, watch `enc_vel` | `enc_vel` ≈ ±5 (the sign depends on mounting). | ~0: encoder not reading (see `encoder_test`). Way off, e.g. ±10 or ±2.5: pole pair count is wrong. |

`x` stops everything; `?` prints status. Status also prints every 500 ms. If your SimpleFOC Mini has a fault output (nFT / FAULT), wire it to a free GPIO and set `FAULT_PIN` to show it in the status line.

If all four steps pass but the closed-loop sketches still fail, the hardware is fine and the problem is in alignment or tuning: see [Configuration and tuning](#configuration-and-tuning).

---

## Web app

`webapp/index.html` is a single file with no build step. It connects to the board over Web Bluetooth and gives you:

- Connect / Disconnect, and a large **STOP** button (the space bar also stops)
- velocity slider with adjustable range, number entry, and preset buttons
- Enable / Disable motor
- live readouts: target and measured velocity (rad/s and RPM), shaft angle, motor voltage
- live chart of target vs measured velocity (5–60 s window), with hover readout, pause, and CSV download
- gain editor (P, I, filter) showing the values currently on the board

### Running it

Web Bluetooth only works in **Chrome or Edge** (not Safari or Firefox), and only from `localhost` or HTTPS:

```bash
cd webapp
python3 -m http.server 8000
```

Then open <http://localhost:8000>, click **Connect**, and pick "Hippo".

On macOS, the first time you connect, Chrome may need Bluetooth permission in System Settings > Privacy & Security > Bluetooth.

---

## Configuration and tuning

All settings are `#define`s at the top of each sketch. The ones that matter:

### Supply voltage

```cpp
#define VOLTAGE_POWER_SUPPLY 24.0
```

**This must match the real supply.** SimpleFOC computes PWM duty cycles from it, so a mismatch scales every commanded voltage. Firmware set to 12 V on a 24 V supply doubles everything; firmware set to 24 V on a 12 V supply halves everything (and alignment will likely fail). Change this before changing the supply.

### Voltage and current limits

| Setting | Value | Why |
|---|---|---|
| `DRIVER_VOLTAGE_LIMIT` | = supply | The driver clamps each phase to 0..this limit, centered at half of it. Setting it below the supply silently halves the usable voltage (the original PID tool had this set to 4 V). |
| `MOTOR_VOLTAGE_LIMIT` | 12 V | Max voltage the motor may be driven with; sets top speed (≈ KV × V). The most available on 24 V with space-vector PWM is about 13.9 V. |
| `CURRENT_LIMIT` | 1.0 A | Caps the velocity loop's output. SimpleFOC estimates voltage as I·R + back-EMF, so this limits stall voltage to ~7 V (~7 W of heat) while still letting voltage rise with speed. Lower it if the motor gets hot. |

### Velocity loop gains

| Setting | Default | Notes |
|---|---|---|
| `FOC_VEL_P` | 0.05 | SimpleFOC's defaults (P=0.5, I=10) oscillated badly on this motor. |
| `FOC_VEL_I` | 1.0 | Raise if the motor sticks at low speed or can't reach the target. |
| `FOC_VEL_LPF_TF` | 0.03 s | Raise to 0.05–0.1 if the velocity reading is noisy. |

Tune live with `P`, `I`, `F` (serial or the web app), then copy the values you like into the `#define`s.

### Startup alignment

On every boot, SimpleFOC turns the motor one electrical revolution each way to find how the encoder lines up with the motor. If the motor doesn't move enough, it reports `Failed to notice movement` and leaves the motor disabled. The sketches start the alignment at `VOLTAGE_SENSOR_ALIGN` (3 V) and retry at 4.5 V and 6 V.

---

## BLE protocol

Service `6e1a0001-8b3c-4f2a-9d4e-2b7c1f0a5e01`. All binary values are little-endian. Packets fit the default 20-byte BLE payload, so no MTU negotiation is needed.

| Characteristic | UUID | Properties | Format |
|---|---|---|---|
| Command | `6e1a0002-…` | write | ASCII, same commands as serial (`V2`, `P0.05`, `E1`, …) |
| Telemetry | `6e1a0003-…` | notify, 20 Hz | `uint32 ms`, `float target_vel`, `float shaft_vel`, `float shaft_angle`, `float voltage_q` (20 bytes) |
| State | `6e1a0004-…` | read, notify, 2 Hz | `float P`, `float I`, `float F`, `uint8 motor_status`, `uint8 enabled` (14 bytes) |

`motor_status`: 4 = ready, 14 = calibration failed, 15 = init failed.

---

## Troubleshooting

**`fatal error: ….h: No such file or directory`**
- `SimpleFOC.h`: install the Simple FOC library.
- `BLEDevice.h`, `BLEServer.h`, `BLE2902.h`, or `Preferences.h`: install the esp32 board package and select **ESP32 Dev Module**. These only exist when an ESP32 board is selected.
- "Multiple libraries were found for BLEDevice.h": the ArduinoBLE library is installed and clashing with the ESP32 one. Uninstall ArduinoBLE.

**`ERR-MOT:Failed to notice movement` / `motor_status=14`**: the motor didn't move during alignment. Check that the supply is on and matches `VOLTAGE_POWER_SUPPLY`, that EN (GPIO 27) is wired, and that the motor phases are connected. If it only passes on a retry, raise `VOLTAGE_SENSOR_ALIGN` to that voltage.

**Velocity reads 0 even when turning the shaft by hand**: same cause as above. When alignment fails, SimpleFOC leaves the sensor direction unknown (0), and the velocity is multiplied by it.

**Velocity reads ±2 rad/s or more when the motor is still**: the velocity window is too short for the encoder's resolution. Use `simple_velocity_control` (2 ms window) or set `sensor.min_elapsed_time = 0.002` before `sensor.init()`. If it's still noisy, check the magnet gap (~0.5–3 mm) and centering, and raise the filter (`F0.05`).

**Weak torque**: use `simple_velocity_control` and raise the voltage limit (`L`). Test with `U3`, `U6`: the motor should resist being turned noticeably harder at each step. If torque is weak even in direct voltage mode, recheck alignment (`motor_status=4` at boot) and that `MOTOR_POLE_PAIRS` is right (`driver_test` step 4).

**Driver outputs ~1 V at `h12` while the Mini's power input reads 24 V**: the driver chip is dead. Replace the SimpleFOC Mini.

**Motor oscillates or buzzes**: gains too high. Halve P, then I, until it stops.

**Motor sluggish or stalls at low speed**: raise I first, then P, in small steps.

**Board resets randomly (brownout)**: the ESP32 isn't getting enough current. See [Powering the ESP32](#powering-the-esp32).

**"Hippo" doesn't show up in the web app's device list**: make sure the BLE sketch is the one flashed, the board finished booting (watch the serial monitor for `# BLE advertising`), no other device is already connected to it, and you're using Chrome/Edge on `localhost` or HTTPS.

### Compiling from the command line

The Arduino IDE bundles `arduino-cli`. On macOS:

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn esp32:esp32:esp32 firmware/ble_velocity_control
```
