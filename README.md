Overview
The following project is for a RC Homemade controller designed to be used in the 30-lb Sumo Category of Combat Robotics at the BotBrawl Competition, August 2024. 

Components
PLA Casing: Ergonomically designed for comfortable use.
Joysticks (x2): For Weapon control and movement of the robot, respectively.
ESCs and Motors: Manage the robot’s movement and are connected to microcontrollers.
ESP32 Microcontroller: The central processing unit of the controller.
nRF24L01 Radio Modules:
  Transmitter (Controller): Operates at 2.4 GHz to send commands.
  Receiver (Robot): Receives commands from the controller.
20x4 LCD Screen: 
  Displays:
    Battery status
    Safety alerts
    Joystick x and y coordinates
    Button press/release status
Radio Power Switch: Turns the nRF24L01 on and off. Functions as a safety feature and power saver during transportation and troubleshooting.

Features

Real-Time Display: The LCD screen provides real-time information on battery levels, safety alerts, joystick coordinates, and button status.
Safety Alerts: Notifications about safety issues and the radio power switch status are displayed.
Power Management: The radio power switch allows for power-saving and safety during transportation.

Usage

Power Switch: ESP32 is supplied with power and LCD Screen turns on
Radio Power Switch: nRF24L01 module is activated.
Control Robot: Use the joysticks to control weaponry and movement.
Monitor Information: Check the LCD screen for battery status, safety alerts, and joystick coordinates.

Troubleshooting

No Display on LCD: Ensure the ESP32 is powered and properly connected internally.
Unresponsive Joysticks: Check connections and ensure ESCs and motors are functioning.
Contributing

Project Created at CombatCraft under Innovation Boost Zone @ Toronto Metropolitan University 
