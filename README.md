# Autonomous Vehicle Safety Supervisor

A safety supervisor for a simulated autonomous vehicle, running on **QNX Neutrino RTOS** on a **Raspberry Pi 4**. It is written in C using native QNX calls: message passing, pulses, timers and interrupts.

## How it works

Three processes talk to each other using QNX message passing:

| Process | Priority | Role |
|---|---|---|
| `safety_supervisor` | 25 (highest) | Approves or rejects navigation commands, watches the other processes, and drives the LEDs through its Recovery Manager thread (24) |
| `sensor_monitor` | 15 | Reads the HC-SR04 ultrasonic sensor (distance) and the MPU-6050 IMU (vibration) and reports to the supervisor |
| `navigation` | 10 (lowest) | Takes movement commands, then asks the supervisor for a safety check before continuing |

The supervisor rejects a command if:
- the speed or steering is over the limit
- an obstacle is within 10 cm
- a sensor is faulty or silent
- the vehicle is vibrating (it slows down, then locks if it does not come to rest within 5 s)

A rejection or fault turns the **red LED** on and stops the vehicle. The **green LED** means normal operation. A separate watchdog turns the red LED on if the supervisor itself hangs.

## Hardware

- Raspberry Pi 4 running QNX SDP 8.0
- HC-SR04 ultrasonic sensor: TRIG on GPIO23, ECHO on GPIO24
- MPU-6050 IMU on I2C (`/dev/i2c1`)
- Red LED on GPIO17, green LED on GPIO27

## Build

Source is in [`AIO/src`](AIO/src). Build it in QNX Momentics with the `aarch64le-debug` configuration, or from a shell:

```bash
source /path/to/qnxsdp-env.sh
qcc -Vgcc_ntoaarch64le -Wall -o safety_supervisor AIO/src/safety_supervisor.c
qcc -Vgcc_ntoaarch64le -Wall -o sensor_monitor AIO/src/sensor_monitor.c
qcc -Vgcc_ntoaarch64le -Wall -o navigation AIO/src/navigation.c
```

## Run

Copy the three binaries to the Pi and start each one in its own terminal, **in this order**:

```bash
sudo ./safety_supervisor
sudo ./sensor_monitor
./navigation
```

`sensor_monitor` accepts an `imufail` argument to simulate an IMU failure. The supervisor terminal has a small CLI: `status`, `health`, `history`, `hang` (watchdog demo) and `quit`.

## Files

- `AIO/src/common.h` – shared message struct, pulse codes and helper functions
- `AIO/src/safety_supervisor.c`, `sensor_monitor.c`, `navigation.c` – the three programs
