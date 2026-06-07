# Auto-Stabilizing Wi-Fi Quadcopter

Custom flight controller firmware for an auto-stabilizing quadcopter, running on a Raspberry Pi Pico 2 W (RP2350) in `no_std` Rust with the Embassy async framework. The drone is controlled entirely over a custom Wi-Fi UDP protocol from a PC — no RC radio.

**Author:** Carlos Artacho Leyva

## Functionality

- Reads a MPU-6050 IMU over I2C (calibration + complementary filter for pitch/roll).
- Receives throttle/pitch/roll/yaw commands and live PID gains from a PC over Wi-Fi UDP at 50 Hz.
- Runs a per-axis PID control loop and mixes the output into four motor commands.
- Drives four ESCs with a manually-timed DShot300 signal, generated inside an interrupt-free block for jitter-free timing.
- Software failsafe: motors are cut to zero throttle if UDP packets stop arriving.
- USB CDC serial output for telemetry and debugging.

## Requirements

### Hardware

- Raspberry Pi Pico 2 W (RP2350)
- MPU-6050 GY-521 IMU
- Mamba F45 4-in-1 ESC (DShot300)
- 4x ECOII 2306 brushless motors
- MINI560 buck converter (11.1V → 5V, since the ESC has no BEC)
- 3S 11.1V LiPo battery
- Mark4 quadcopter frame, FR4 perfboard, XT60 connectors

### Software

- Rust nightly toolchain with target `thumbv8m.main-none-eabihf`
- [`picotool`](https://github.com/raspberrypi/picotool) for flashing over USB
- Python 3 (standard library only) for the ground-station client

## Pinout

| Function | Pico GPIO | Notes |
|---|---|---|
| Motor 1 (DShot) | GP0 | front-left |
| Motor 2 (DShot) | GP1 | front-right |
| Motor 3 (DShot) | GP2 | rear-right |
| Motor 4 (DShot) | GP3 | rear-left |
| ESC telemetry RX | GP5 | optional UART |
| I2C SDA | GP6 | MPU-6050 |
| I2C SCL | GP7 | MPU-6050 |

The MPU-6050 is powered from 3V3 OUT (pin 36). The Pico is powered from the MINI560 5V output into VSYS (pin 39).

## Ground Station

The Python client (`udp_client.py`) controls the drone from the keyboard:


| Key | Action |
|---|---|
| `W` / `S` | Throttle up / down |
| Arrows | Pitch / roll |
| `A` / `D` | Yaw |
| `U` / `J` | Kp up / down |
| `I` / `K` | Ki up / down |
| `O` / `L` | Kd up / down |
| `SPACE` | Emergency stop |
| `Q` | Quit |

### Control Protocol

20-byte little-endian UDP packet, sent at 50 Hz:

```
struct.pack('<Hhhhfff', throttle, pitch, roll, yaw_rate, kp, ki, kd)
```

- `throttle`: u16, 0–2000 (DShot)
- `pitch`, `roll`: i16, centidegrees
- `yaw_rate`: i16, centidegrees/s
- `kp`, `ki`, `kd`: f32 PID gains

## Software Design

Five Embassy async tasks share state through `Mutex<CriticalSectionRawMutex, _>` globals:

- `usb_task` — USB CDC telemetry (non-blocking sends)
- `udp_task` — parses control packets, updates setpoint + failsafe timestamp
- `imu_task` — MPU-6050 read, calibration, complementary filter
- `pid_task` — `Ticker`-driven PID loop + motor mixer + failsafe
- `motor_task` — DShot300 frame transmission inside a `critical_section` block

## Project Status

Reached tethered hover at ~50% throttle with working IMU, Wi-Fi control, PID stabilization and live tuning. Development paused after a hardware failure of the Pico during high-thrust bench testing (suspected BEC/Back-EMF event). See the full documentation for the troubleshooting log and failure analysis.