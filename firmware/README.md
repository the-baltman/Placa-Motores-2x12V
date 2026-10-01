# Placa-Motores-2x12V - firmware

Two 12 V DC motors over ESP-NOW. Arduino-ESP32 core 3.x (tested against 3.3.0 headers), ESP32-WROOM-32 only.
Pins: Pines-ESP32 v1.1 (frozen). All pins, limits and gains live in `receiver/config.h`.

## Layout (one concern per module)

| Path | Role | Build step it belongs to |
|---|---|---|
| `libraries/PlacaProtocol/` | packet format, CRC-16, freshness filter (pure C++) | Step: radio link test |
| `receiver/config.h` | pins, timing, limits, gains | all |
| `receiver/motor.*`, `motor_math.h` | LEDC 20 kHz 10 bit timer0, ramp, deadband, min duty 10 %, EN_MCU | Step: motor outputs |
| `receiver/sensing.*`, `sensing_math.h` | ADC1: current (divided by duty), VBAT, NTC, soft limits | Step: sensing |
| `receiver/failsafe_hw.*` | WDT_KICK (GPIO17), ESTOP_SENSE, FLT_ANY, reset cause | Step: failsafe test |
| `receiver/espnow_link.*` | ESP-NOW rx, 300 ms link timeout (configurable) | Step: radio link test |
| `receiver/supervisor.*` | safety state machine (pure) | Step: failsafe test |
| `receiver/indicators.*`, `indicator_patterns.h` | WS2812 + buzzer | Step: indicators |
| `receiver/receiver.ino` | boot order, control task (10 ms) | flash step |
| `transmitter/transmitter.ino` | serial/joystick test transmitter | radio link test |
| `test/` | PC unit tests | - |

## Behaviour summary

- Boot: EN_MCU low, LEDC duty 0, WDT_KICK low. Motors stay at 0 until ARMED.
- ARMED = valid link + no blocking fault + a NEUTRAL command received. After link loss, E-stop, overtemp or battery cut, a neutral command is required again.
- No valid packet for 300 ms: ramp to 0 (0.2 s from full), then EN_MCU = 0. Change at runtime: type `t 500` in the receiver serial monitor (100..2000 ms).
- Packet: 13 bytes, magic/version, session, seq (u16), flags, m1, m2 (permille), CRC-16/CCITT-FALSE. Bad CRC, out-of-range, reserved flags, stale or duplicate seq, other session while linked: dropped.
- Current: IS reads D x I, so I = V_net / (0.0588 x D); invalid below D = 10 %. Zero offset calibrated at boot with drivers off. Soft limit 12 A for 1 s derates; > 15 A for 30 ms trips the channel (latched; clear with the CLEAR_FAULT flag at neutral, `c` on the transmitter serial).
- Soft cuts: battery 9.6 V (LiPo) / 10.5 V (Pb), overtemp NTC < 0.295 V (release > 0.43 V), OV 15.5 V (release 15.0 V) cuts EN_MCU (coast, no regenerative ramp) and disarms; re-arm needs a neutral command.
- Arming also needs 4 consecutive healthy WDT kicks (>= 1 rising edge). More than 50 ms (`CONTROL_STALL_MS`) between rising edges on WDT_KICK, or a single control period above it, stops and disarms immediately (several late cycles add up); re-arming needs a neutral command and 4 healthy kicks (>= 1 rising edge). An init failure shows FALLA_SISTEMA (red 8 Hz), never blue.

## Build (arduino-cli)

```
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries firmware/libraries firmware/receiver
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries firmware/libraries firmware/transmitter
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COMx firmware/receiver
```
Arduino IDE: copy `libraries/PlacaProtocol` into your `Arduino/libraries`.

### Pairing (mandatory before any motor with load)

1. Flash the receiver, read its MAC from the serial boot line.
2. Put it in `RX_MAC` of `transmitter.ino` (unicast; the transmitter refuses to start with all zeros).
3. Read the transmitter's own MAC from its serial boot line (`TX MAC ...`, printed even when `RX_MAC` is still zeros) and put it in `cfg::TX_MAC_FILTER` of `receiver/config.h` and reflash. All zeros = bench mode: the receiver prints `WARN: bench mode` and accepts ANY transmitter on the channel.
4. Keep `ESPNOW_CHANNEL` equal in both.

## PC tests (protocol, CRC, ramp, current maths, supervisor, LED patterns)

```
sh firmware/test/run_tests.sh      # needs g++; expect "N checks, 0 failed"
```

## Bench test (config B, 12 V supply limited to 1 A, NO motors)

Measure with a scope/multimeter on the motor terminals or the IBT-2 inputs; the supply limit protects against mistakes.

1. Power with E-stop closed. Serial 115200: expect reset cause, MAC, no `ERROR:` lines. `cal=1 sens=1`. If `cal=0`: CS offset > 100 mV, check wiring before anything else.
2. VBAT reading `vbat=` equals the multimeter within ~0.2 V (ADC calibration is a to-verify item). `ntc=` about 1100 mV at 25 C (two 10 k NTC in parallel = 5 k against the 10 k pull-up).
3. LED: blue blink at boot, then yellow breathing (SIN_ENLACE). Turn the transmitter on (`m 0 0` or just wait): neutral -> green solid + 1 beep.
4. `m 500 0` : GPIO25 shows 20 kHz PWM, duty ramping to ~50 % mapped; GPIO26 stays 0. `m -500 0`: swaps legs through 0. The transmitter command expires after 0.5 s (dead-man) unless resent.
5. Link loss: send `m 800 800`, then power off the transmitter. Within 300 ms + 0.2 s ramp the PWM is 0 and EN_MCU (GPIO27) goes low. LED orange fast blink. Powering the transmitter again with `m 800 800` must NOT start anything; send `m 0 0` first.
6. E-stop: press it while a command is active: LED red solid, 3 beeps, PWM 0. Release: needs neutral again.
7. WDT_KICK (GPIO17): square wave ~50 Hz (toggle every 10 ms) on a scope while healthy. Verify the board hardware cuts EN_DRV when you stall the firmware (hold the control task by pulling the debugger/RESET is not a test; use a deliberate `while(1)` build) - test T6 of ERC-DRC-y-pruebas.
8. FLT_ANY / CS: with drivers on and no motor, short a CS node through a resistor to raise it > 1.18 V: FLT_ANY goes low, red blinks (1 or 2 flashes = channel), channel stops, latched until `c` + neutral.
9. Battery: lower the supply to 9.8 V -> magenta double flash; below 9.6 V for 0.5 s -> motors disarm.
10. Serial status line each second: `ctl_max` must stay near 10000 us, `late=0`, `sens_max` well under 10000 us (measure and write the number in the manual).

## Known limits (v1)

Encoders (PCNT), I2C and telemetry back to the transmitter are not implemented. ESP-NOW is not encrypted: set `TX_MAC_FILTER` in `config.h` for anything but bench use.
