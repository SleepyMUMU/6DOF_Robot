# 6DOF_Robot: Controlling a 6DOF Robotic Arm 🤖

## Introduction

Welcome to the 6DOF_Robot project! This repository contains the source code for controlling a 6-degree-of-freedom (6DOF) robotic arm. The arm is part of a quadruped track robot, combining mobility and manipulation capabilities. 🚀

### Project Overview

The robot features:
- **Main Control MCU**: ESP32-WROVER-E
- **Structure**: Tracks and robotic arm fusion
- **Modes**: Quadruped walking, track movement, and arm manipulation
- **Tracks**: Driven by DC motors
- **Arm**: 6DOF, controlled by serial bus servos

This project replicates and improves upon the design from [oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren](https://oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren). 🌟

### Hardware Requirements

To use this code, you need:
- ESP32-WROVER-E board
- 6 serial bus servos (specific models as per [oshwhub page](https://oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren))
- DC motors for track control
- Power supply and other components as specified in the [oshwhub page](https://oshwhub.com/imumu/si-zu-lv-dai-ji-qi-ren)

### Software Requirements

- **Programming Language**: C++ (with Arduino framework)
- **Libraries**:
  - ESP32 Arduino library
  - Servo control library
  - Any other libraries used in the code

### Installation and Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/SleepyMUMU/6DOF_Robot
