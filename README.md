# Axon — Wireless IMU-Driven Robotic Hand

A servo-driven robotic hand controlled in real time via a wearable glove, communicating wirelessly over ESP-NOW. Each finger is tracked independently using an MPU6500 IMU, replicating the operator's hand movements with low latency across a wireless link.

Most glove-controlled hands use flex sensors, resistive bend detectors that measure finger curl directly. Axon uses IMUs instead, capturing full orientation data per finger and applying a complementary filter for stable, noise-resistant angle estimation. A wrist cross-talk compensation system corrects for the mechanical coupling between wrist rotation and finger pitch readings, improving accuracy during combined movements.

Potential applications include remote operation in hazardous environments, teleoperation in manufacturing, and movement capture and playback.

---

## Demo


https://github.com/user-attachments/assets/ab849606-339d-44fb-b26b-eb3253a73d4d



---

## System Overview

Two-node ESP-NOW architecture. The glove node reads 6 IMUs (5 fingers + wrist) through a TCA9548A I2C multiplexer, computes finger angles, and transmits a lightweight packet wirelessly at ~50Hz. The hand node receives the packet and drives 5 servos with a smoothing loop to eliminate jitter.

🧤 SENDER (Glove)              📡 ESP-NOW               🤖 RECEIVER (Hand)
├─ 6 × MPU6500 IMUs           ~50Hz packet            ├─ 5 × MG90S Servos
├─ TCA9548A Multiplexer       lightweight struct      └─ Incremental smoothing
├─ Complementary Filter
├─ Wrist Cross-talk Comp.
└─ Per-finger Angle Calc.

---

## How It Works

The glove hosts 6 MPU6500 IMUs, one per finger and one on the wrist, connected to a single I2C bus through a TCA9548A multiplexer. Each sensor is selected individually by toggling the multiplexer channel, raw accelerometer and gyroscope data is read, and the channel is closed before moving to the next. This prevents bus conflicts across sensors sharing the same I2C address.

Raw sensor data is processed through a complementary filter. The gyroscope integrates angular velocity over time for accurate short-term angle tracking, while the accelerometer provides an absolute angle reference to correct gyroscopic drift. A 95/5 blend between the two gives stable, low-noise angle estimates during both fast and slow movements.

A wrist compensation system corrects for mechanical cross-talk. When the wrist rotates, the physical geometry of the hand causes each finger IMU to register a false pitch change. Each finger has an empirically calibrated coefficient that removes this contribution before the angle is mapped to a servo value.

Computed angles are packed into a lightweight struct and transmitted wirelessly via ESP-NOW at approximately 50Hz. The hand node receives each packet, unpacks the struct, and steps each servo incrementally toward its target angle each millisecond, eliminating jitter from abrupt angle changes.

---

## Code

The firmware is split across two files:

`sender/sender.ino` — IMU initialisation and calibration, TCA multiplexer control, complementary filter, wrist cross-talk compensation, ESP-NOW packet construction and transmission.

`receiver/receiver.ino` — ESP-NOW packet reception and unpacking, servo index remapping, inversion handling, incremental smoothing loop.

Built with assistance from AI tooling for implementation. System architecture, sensor selection, and design decisions were my own.

---

## Components

### Glove (x2 required)
| Component | Quantity |
|---|---|
| ESP32-C3 Mini | 1 |
| TCA9548A I2C Multiplexer | 1 |
| MPU6500 IMU | 6 |
| LM2596 DC-DC Buck Converter | 1 |
| 3.7V 400mAh 25C LiPo | 1 |
| Power Switch | 1 |
| Glove | 2 *(inner mounts components and wiring, outer conceals)* |

### Hand
| Component | Quantity |
|---|---|
| ESP32-C3 Mini | 1 |
| MG90S Servo | 5 |
| LM2596 DC-DC Buck Converter | 1 |
| 7.4V 2000mAh 50C LiPo | 1 *(or any supply capable of meeting voltage and current requirements)* |
| 3D Printed Hand | 1 |

---

## Schematics

### Glove
<img width="782" height="790" alt="sender_schematic" src="https://github.com/user-attachments/assets/2bf6ca47-92ad-426b-be3a-d54aac360a6c" />


### Hand
<img width="930" height="722" alt="receiver_schematic" src="https://github.com/user-attachments/assets/74f468f2-cc55-4499-ae9a-d1da7fabf7de" />

### Overall Glove Schematic

<img width="428" height="478" alt="overall_glove_schematic" src="https://github.com/user-attachments/assets/2b9f1a0a-b035-4c53-93a6-91868ce6ff4a" />

---

## Assembly

For mechanical assembly of the robotic hand, refer to the original design:

- [3D Files Robo Hand.zip](https://github.com/user-attachments/files/28094853/3D.Files.Robo.Hand.zip)

- https://youtu.be/Fvg-v8FPcjg?si=IIIji8jIlL_NFhKd

---

## Credits

- Firmware developed with AI tooling assistance
