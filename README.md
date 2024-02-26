# Automated_Factory_Guided_Vehicle

This project aimed to design an Automated Guided Vehicle (AGV) using the Parallax Propeller Activity Board WX. The AGV's functionality includes moving widgets from random pickup locations to random drop-off locations. It employs various devices, including infrared sensors, ultrasonic sensors, servos, LEDs, and an LCD.

# Factory Layout
![factory_layout](https://github.com/IJAMUL1/Automated-Factory-Guided-Vehicle/assets/60096099/166a0aa9-6595-4ca9-8fe7-8980b6d10093)

# Robot Demo

<p align="center">
  <img src="https://github.com/IJAMUL1/Automated-Factory-Guided-Vehicle/assets/60096099/1f4b8f3c-ea03-4fb7-ba61-6df783df34a1" alt="Your Image Description" width="600">
</p>

## Table of Contents

- [Introduction](#introduction)
- [Project Requirements](#project-requirements)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Setup Instructions and Usage](#setup-instructions-and-usage)

## Introduction

In this project, we are tasked with designing an Automated Guided Vehicle (AGV) that can navigate through a factory floor to pick up widgets from one location and drop them off at another, while adhering to lane rules, detecting intersections, avoiding obstacles, and performing various tasks autonomously.

## Project Requirements

The project requirements include designing an AGV that can:
- Follow the designated path on the factory floor, navigating through intersections and lanes.
- Detect and indicate intersections using appropriate displays.
- Avoid obstacles dynamically introduced in its path.
- Reduce speed visibly while traversing pickup and drop-off lanes.
- Reach randomly assigned pickup and drop-off locations, indicating successful arrival.
- Calculate and display the distance traveled between pickup and drop-off locations.
- Detect, indicate, and stop at dynamic obstacles in pickup and drop-off lanes.
  
## Hardware Requirements

List all the hardware components required for the project, including but not limited to:
- Chassis and mechanical components
- Parallax Propeller microcontroller
- 2 x Continous Servo Motors
- Pulolo Reflecance Sensor
- 3 x Ultrasonic Sensors
- Parallax 2 X 16 Serial LCD With Piezo Speaker 
- Power supply
- Breadboard
- Wires
- Leds

## Software Requirements

The software components required for the project include:
- Control algorithms for line following and intersection detection.
- Obstacle avoidance algorithms.
- Speed control algorithms for reducing speed in pickup and drop-off lanes.
- Logic for indicating successful pickup and drop-off.
- Distance calculation algorithms.
- Error handling and recovery mechanisms for dynamic obstacles.

## Setup Instructions and Usage

To set up and use the AGV system effectively, follow these steps:
- Assemble the hardware components according to the provided instructions.
- Install the necessary software libraries and dependencies on the microcontroller platform being used.
- Upload the provided codebase onto the microcontroller.
- Calibrate the sensors and actuators to ensure proper functionality.
- Place the AGV at the starting position (home location S) on the factory floor.
- Run the AGV system and observe its behavior as it navigates through the designated path, detects intersections, avoids obstacles, picks up and drops off widgets, and displays relevant information.
- Monitor and troubleshoot any issues that arise during operation, making adjustments to the hardware or software as necessary.
- Demonstrate the AGV system to the project evaluators, showcasing its autonomous capabilities and adherence to the specified requirements.

By following these instructions, you can successfully implement and demonstrate the AGV system for the Propeller Project, fulfilling all project requirements and achieving a high grade in the evaluation process.

