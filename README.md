# CombatCraft Bot Controller

<br />
<div align="center">
  <a>
    <img src="./assets/IBZ-Logo.svg.png" alt="Logo">
  </a>
</div>

<div align="center">
    <b>
        Project Created at CombatCraft under Innovation Boost Zone @ Toronto Metropolitan University
    </b>
</div>

<div align="center">
    <img src="./assets/MIT-License.svg"/>
</div>

## Overview

This project involves the design and implementation of an RC controller for use in the 30-lb Sumo Category of Combat Robotics at the BotBrawl Competition, scheduled for August 2024.

### Components

- **PLA Casing**: Ergonomically designed for comfortable handling.
- **Joysticks (x2)**: Facilitate control of both the weapon and the movement of the robot.
- **ESCs and Motors**: Responsible for the robot’s movement, interfaced with microcontrollers.
- **ESP32 Microcontroller**: Serves as the central processing unit of the controller.
- **nRF24L01 Radio Modules**:
  - **Transmitter (Controller)**: Operates at 2.4 GHz to send commands to the robot.
  - **Receiver (Robot)**: Receives commands from the controller.
- **20x4 LCD Screen**: Displays vital statistics and information.

### LCD Display Stats

- **Battery Status**: Monitors and displays current battery levels.
- **Safety Alerts**: Notifies users of any safety-related issues.
- **Joystick X and Y Coordinates**: Displays real-time joystick positions.
- **Button Press/Release Status**: Indicates the current state of button inputs.
- **Radio Power Switch**: Controls the power state of the nRF24L01, acting as both a safety feature and a power-saving mechanism during transport and troubleshooting.

## Features

- **Real-Time Display**: The LCD screen provides continuous updates on battery levels, safety alerts, joystick coordinates, and button statuses.
- **Safety Alerts**: Alerts related to safety issues and the status of the radio power switch are prominently displayed.
- **Power Management**: The radio power switch enables power conservation and enhances safety during transport.

## Usage

- **Power Switch Activation**: Power is supplied to the ESP32, and the LCD screen is activated.
- **Radio Power Switch Activation**: The nRF24L01 module is turned on for operation.
- **Robot Control**: Joysticks are used to manage weaponry and robot movement.
- **Monitoring**: The LCD screen provides information on battery status, safety alerts, and joystick coordinates.

## Troubleshooting

- **No Display on LCD**: Verify that the ESP32 is powered and properly connected internally.
- **Unresponsive Joysticks**: Check connections and ensure that ESCs and motors are functioning correctly.
