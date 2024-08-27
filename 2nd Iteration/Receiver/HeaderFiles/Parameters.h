#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

/* Code Inclusion Parameters */
#define PRINT_EXTRA  // Prints extra information during loop, helpful for troubleshooting purposes.

/* Pin Mapping Parameters */
const uint8_t BATTERY = 27;      // ESP32 GPIO connected to the battery measurement voltage divider.
const uint8_t RADIO_CE = 2;      // ESP32 GPIO connected to the CE pin on the NRF24L01 radio module.
const uint8_t RADIO_CSN = 15;    // ESP32 GPIO connected to the CSN pin on the NRF24L01 radio module.
const uint8_t DRIVE_LEFT = 32;   // ESP32 GPIO connected to the ESC signal pin for the robot's left drive motor.
const uint8_t DRIVE_RIGHT = 33;  // ESP32 GPIO connected to the ESC signal pin for the robot's right drive motor.
const uint8_t WEAPON = 26;       // ESP32 GPIO connected to the ESC signal pin for the robot's weapon motor.

/* Radio Channel Parameters */
const uint8_t RADIO_CH1 = 34;     // Sets the actual radio channel shown in the menu as Channel 1.
const uint8_t RADIO_CH2 = 74;     // Sets the actual radio channel shown in the menu as Channel 2.
const uint8_t RADIO_CH3 = 114;    // Sets the actual radio channel shown in the menu as Channel 3.
const uint8_t PAYLOAD_MAX = 32;   // Maximum number of bytes that can be stored in a dynamic payload.
const uint16_t CH_TO_MHZ = 2400;  // Offset for converting a channel number to its carrier frequency.

/* Other Parameters */
const unsigned long SERIAL_BAUD = 115200;   // Baud rate used for the USB serial channel (serial monitor).
const unsigned long WATCHDOG = 1000;        // Maximum allowed time without receiving a message from the controller.
const unsigned long CHAN_DELAY = 1000;      // The time to search for messages on each channel before switching.
const unsigned long REPLY_TIMEOUT = 200;    // Maximum time allowed for sending a reply message during connection.
const unsigned long WRITE_DELAY = 100;      // Delay in milliseconds between consecutive calls of radio.write().
const unsigned long PRINT_INTERVAL = 250;  // How often new information is printed to the display and serial console.

#endif