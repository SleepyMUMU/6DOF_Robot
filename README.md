# 6DOF_Robot: Controlling a 6DOF Robotic Arm 🤖

Welcome to the **6DOF_Robot** project! This repository houses the source code to control a 6-degree-of-freedom (6DOF) robotic arm integrated into a quadruped tracked robot. Combining mobility and precise manipulation, this project is perfect for robotics enthusiasts and tinkerers! 🚀

---

## Project Overview

![image](https://github.com/user-attachments/assets/014d89f8-d0ea-424d-8bf5-62ac5599c5e4)


The 6DOF_Robot is a versatile platform featuring:
- **Quadruped Mobility**: Walks on four legs or rolls on tracks.
- **Tracked Movement**: Powered by DC motors for rugged terrain.
- **6DOF Robotic Arm**: Capable of complex manipulation tasks.

This project builds upon and enhances the design from [oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren](https://oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren). ✨

---

## Hardware Requirements

To bring this project to life, you'll need the following components:

| **Category**          | **Details**                                      |
|-----------------------|-------------------------------------------------|
| Main Control MCU      | ESP32-WROVER-E                                  |
| Robotic Arm DOF       | 6DOF                                            |
| Number of Servo Motors| 6 (serial bus interface, PH2.0-3P, 74HC125N driver) |
| Track Drive           | DC motors, using TB6612FNG chip                 |
| Programming Language  | C++ (Arduino framework)                         |
| Required Libraries    | ESP32 Arduino library, servo control library    |

Additional components (e.g., power supply, wiring) are detailed on the [oshwhub page](https://oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren).

---

## Software Requirements

- **Language**: C++ (Arduino framework)
- **Libraries**:
  - ESP32 Arduino library
  - Servo control library
  - Additional libraries as specified in the code

---

## Installation

Follow these steps to get started:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/SleepyMUMU/6DOF_Robot
