# STM32 Self-Balancing Robot 🤖⚖️

A C project that transforms an STM32 microcontroller into an autonomous, two-wheeled self-balancing robot. 
This project utilizes an onboard IMU (Accelerometer + Gyroscope), a mathematically tuned Complementary Filter for precise angle estimation, and a high-speed PID controller to maintain equilibrium in real-time.

## 📋 Table of Contents
1. [Features](#features)
2. [Hardware Architecture](#hardware-architecture)
3. [Software & Timing Architecture](#software--timing-architecture)
4. [The Control Loop (Math & Physics)](#the-control-loop-math--physics)
5. [Tuning Guide](#tuning-guide)
6. [Troubleshooting FAQ](#troubleshooting-faq)
7. [Installation & Setup](#installation--setup)

---

## ✨ Features
* **200Hz Deterministic Control Loop:** Driven by a dedicated hardware timer (TIM4) to ensure mathematically perfect `dt` calculations, completely bypassing the need for a bulky RTOS.
* **Auto-Calibration:** On boot, the system averages 50 static sensor readings to automatically calculate and subtract mechanical zero-point offsets.
* **Advanced Sensor Configuration:** Utilizes High-Resolution mode and Block Data Update (BDU) shields on the IMU to prevent "torn reads" and data corruption.
* **Motor Deadzone Compensation:** Maps out the physical static friction of the gear motors so they instantly snap into motion at the slightest command.
* **Safety Kill-Switch:** Automatically detects if the robot has fallen over (tilt > 40 degrees) and cuts all motor power to prevent runaway situations.

---

## 🛠️ Hardware Architecture

### Components Used
* **Microcontroller:** STM32F3 Series (ARM Cortex-M4)
* **Accelerometer:** LSM303 (Communicating via I2C) - Measures absolute tilt relative to gravity.
* **Gyroscope:** L3GD20 (Communicating via SPI) - Measures angular velocity (speed of tilt).
* **Actuators:** 2x DC Gear Motors.
* **Motor Driver:** Dual Channel DC Motor Driver (e.g., L298N, TB6612FNG, or L293D).
* **Power Supply:** 9V-12V Battery Pack for motor torque, with a buck converter stepping down to 5V/3.3V for the logic board.

### Pin Mapping
| Component / Function | STM32 Peripheral | Physical Pin |
| :--- | :--- | :--- |
| **Left Motor Direction F/B** | GPIO Output | `PB0`, `PB1` |
| **Right Motor Direction F/B**| GPIO Output | `PB12`, `PB13`|
| **Motor Speed (PWM)** | TIM2 (CH1 & CH2) | Mapped via CubeMX |
| **Gyroscope CS** | GPIO Output | `PE3` |
| **Gyroscope SPI** | SPI1 | Mapped via CubeMX |
| **Accelerometer I2C** | I2C1 | `PB7` (SDA) / `PB6` (SCL) |

---

## 🧠 Software & Timing Architecture

The firmware is entirely interrupt-driven, ensuring the motors react to physical changes instantly:
1. **TIM4 (The Heartbeat):** Set to fire exactly 200 times a second (`dt = 0.005s`). It reads the sensors, updates the Complementary Filter, calculates the PID error, and updates the motor speeds.
2. **TIM2 (The Muscles):** Configured to generate a high-frequency (10kHz) hardware PWM signal. This high frequency ensures the motors run smoothly without emitting a loud, high-pitched whine. `ARR` is set to 4799.

---

## 🧮 The Control Loop (Math & Physics)

### 1. The Complementary Filter
Motor vibrations act like a localized earthquake to a sensitive accelerometer. To prevent the angle from jumping wildly when the motors engage, we use a **Complementary Filter** heavily weighted toward the Gyroscope:
```c
tilt_angle = 0.999f * (tilt_angle + (gyro_rate * dt)) + 0.001f * acc_angle;
