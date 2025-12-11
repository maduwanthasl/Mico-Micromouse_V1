# 🐭 Mico - Micromouse V1

Welcome to the repository for **Mico**, my custom-designed micromouse robot! This project involved extensive research, component selection, PCB design, and firmware development, all aimed at building a highly capable and efficient micromouse for maze-solving applications.

## 📑 Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Components](#hardware-components)
3. [Component Comparison](#component-comparison)
4. [PCB Design](#pcb-design)
5. [Microcontroller - STM32 Blue Pill](#microcontroller---stm32-blue-pill)
6. [Firmware Development](#firmware-development)
7. [Getting Started](#getting-started)
8. [Acknowledgments](#acknowledgments)

## 📝 Project Overview

The goal of this project was to build a fully functional micromouse named **Mico** from scratch. This involved careful consideration of hardware, custom PCB design, and firmware development. Throughout the process, I selected and evaluated various components to ensure optimal performance and compatibility.

## 🔧 Hardware Components

Mico is built using carefully chosen components to maximize performance and reliability. Some of the key components include:

- **Microcontroller (MCU)**: STM32 Blue Pill
- **Power Supply**: 3.3V regulator
- **Motor Driver**: [Specify Motor Driver Here]
- **Sensors**: Infrared emitters and receivers for maze sensing
- **Motors**: Precision motors with encoders for accurate movement
- **Additional Components**: Coupling capacitors (100nF, 100V) for noise reduction, etc.

## 🔍 Component Comparison

| **Component**        | **Option 1**                    | **Option 2**                     | **Notes**                                                                                     |
|----------------------|---------------------------------|----------------------------------|-----------------------------------------------------------------------------------------------|
| **Microcontroller**  | STM32                           | ATmega328P                       | STM32 chosen for better performance and support for PWM control                              |
| **Motor Type**       | N20 Motor with Encoder          | Coreless Motor with Custom Encoder | N20 with encoder used due to budget constraints and time                                     |
| **Battery**          | 7.4V LiPo (Downconversion)      | 3.7V LiPo (Upconversion)         | Upconversion method preferred for weight reduction                                           |
| **IR Sensors**       | TOF Sensors                     | IR Emitter and Phototransistor   | IR Emitter chosen; TOF needs multiplexer, increasing response time                           |
| **Power Regulator**  | AM1117 (3.3V)                   | MT3608 Step-Up Converter         | MT3608 chosen for efficient power conversion                                                 |
| **Motor Driver**     | TB6612FNG                       | L298N                            | TB6612FNG preferred for compact design and compatibility                                     |

_Table 1: Comparison of Key Components for MICO Micromouse Robot_


## 🖥️ PCB Design

I designed a custom PCB specifically for Mico to organize and integrate the various components effectively. The PCB design process involved:

- Component selection based on performance requirements.
- Layout and routing in Altium Designer.
- Prototyping and testing to ensure functionality.

The final PCB was successfully manufactured and assembled, forming the core hardware of the micromouse.

## ⚙️ Microcontroller - STM32 Blue Pill

Initially, I considered using the STM32 Black Pill due to its higher processing capability. However, due to the limited number of available analog channels, I opted for the **STM32 Blue Pill**, which provided the necessary analog inputs for the sensors. The Blue Pill proved to be reliable and sufficient for Mico’s control system.

## 📝 Firmware Development

The firmware for Mico was developed using **STM32CubeIDE**. Key aspects of the firmware include:

- **Sensor Data Processing**: Collects and processes data from the infrared sensors to detect walls and make navigational decisions.
- **Motor Control**: Controls the speed and direction of the motors for precise movement.
- **Maze Solving Algorithm**: Implements a maze-solving algorithm to navigate and solve mazes autonomously.

The firmware was written in C, leveraging various STM32 libraries for peripheral management and efficient control.

## 🚀 Getting Started

To get started with Mico's firmware and hardware setup:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/maduwanthasl/Mico-Micromouse_V1.git
   cd Mico-Micromouse_V1


## 🙏 Acknowledgments
This project was made possible thanks to the support and resources available through the open-source and embedded systems community.

Enjoy exploring Mico’s journey through maze-solving and micromouse challenges! Contributions and feedback are welcome.
