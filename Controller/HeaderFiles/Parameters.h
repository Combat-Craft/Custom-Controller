#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

/* Code Inclusion Parameters */
// #define BIDIRECTIONAL_DRIVE  // Changes PWM control for bidirectional ESC firmware.
#define PRINT_EXTRA  // Prints extra information during loop, helpful for troubleshooting purposes.

/* Pin Mapping Parameters */
const uint8_t BATTERY = 27;     // ESP32 GPIO connected to the battery measurement voltage divider.
const uint8_t RADIO_CE = 2;     // ESP32 GPIO connected to the CE pin on the NRF24L01 radio module.
const uint8_t RADIO_CSN = 15;   // ESP32 GPIO connected to the CSN pin on the NRF24L01 radio module.
const uint8_t JOYSTICK_X = 32;  // ESP32 GPIO connected to the x-axis pin of the drive joystick.
const uint8_t JOYSTICK_Y = 33;  // ESP32 GPIO connected to the y-axis pin of the drive joystick.
const uint8_t JOYSTICK_B = 25;  // ESP32 GPIO connected to the switch pin of the drive joystick.
const uint8_t WEAPON_SW = 26;   // ESP32 GPIO connected to the weapon switch.

/* Pulse Width Tuning Parameters */
const uint16_t PULSE_MIN = 1000;   // Minimum pulse width for calibrating ESCs.
const uint16_t PULSE_MAX = 2000;   // Maximum pulse width for calibrating ESCs.
const uint16_t DRIVE_RANGE = 250;  // Maximum pulse width variation for drive motors.
const uint16_t WEAPON_OFF = 1000;  // Pulse width value to turn the weapon motor off.
const uint16_t WEAPON_ON = 1250;   // Pulse width value at which to run the weapon motor.

/* Radio Channel Parameters */
const uint8_t RADIO_CH1 = 34;     // Sets the actual radio channel shown in the menu as Channel 1.
const uint8_t RADIO_CH2 = 74;     // Sets the actual radio channel shown in the menu as Channel 2.
const uint8_t RADIO_CH3 = 114;    // Sets the actual radio channel shown in the menu as Channel 3.
const uint8_t PAYLOAD_MAX = 32;   // Maximum number of bytes that can be stored in a dynamic payload.
const uint16_t CH_TO_MHZ = 2400;  // Offset for converting a channel number to its carrier frequency.

/* Joystick Parameters*/
const bool XAXIS_REVERSED = false;      // Indicates whether to invert the values read from the joystick x-axis.
const bool YAXIS_REVERSED = true;       // Indicates whether to invert the values read from the joystick y-axis.
const bool BUTTON_REVERSED = true;      // Indicates whether to invert the values read from the joystick button.
const uint16_t NOISE_THRESH = 5;        // Minimum registered deviation from drive joystick origin.
const unsigned long RECORD_TIME = 100;  // Duration in milliseconds for which the joystick is sampled to find its origin.

/* Battery Measurement Parameters */
const uint8_t TXBATT_DIVFACTOR = 3;  // Division factor used in the controller's battery voltage divider.
const uint8_t RXBATT_DIVFACTOR = 2;  // Division factor used in the robot's battery voltage divider.
const uint16_t TXBATT_VMIN = 4500;   // Minimum expected controller power supply voltage in millivolts.
const uint16_t TXBATT_VMAX = 6500;   // Maximum expected controller power supply voltage in millivolts.
const uint16_t RXBATT_VMIN = 4500;   // Minimum expected robot power supply voltage in millivolts.
const uint16_t RXBATT_VMAX = 5500;   // Maximum expected robot power supply voltage in millivolts.

/* Liquid Crystal Parameters */
const uint8_t LCD_ADDRESS = 0x27;  // The address that will be assigned to the LCD on the I2C bus.
const uint8_t LCD_COLUMNS = 20;    // The number of characters that the LCD can display in each row.
const uint8_t LCD_ROWS = 4;        // The number of rows of text that the LCD can display simultaneously.

/* Timing Parameters */
const unsigned long MENU_DELAY = 350;      // Delay in milliseconds between any event that occurs in a menu.
const unsigned long REPLY_TIMEOUT = 200;   // Reply timeout in milliseconds for initial connection sequence.
const unsigned long WRITE_DELAY = 100;     // Delay in milliseconds between consecutive calls of radio.write().
const unsigned long ESCCAL_DELAY = 5000;   // Delay in milliseconds for holding min/max PWM values during calibration.
const unsigned long PRINT_INTERVAL = 250;  // How often new information is printed to the display and serial console.

/* Other Parameters */
const uint8_t PACKET_LOSS = 32;            // Maximum number of consecutive packets lost before forcing a reset.
const uint16_t ADC_MAXREAD = 4095;         // Maximum value returned by analogRead() for the NodeMCU ESP32S.
const uint16_t ADC_MAXVOLT = 3300;         // Range in millivolts spanned by analogRead() for the NodeMCU ESP32S.
const unsigned long SERIAL_BAUD = 115200;  // Baud rate used for the USB serial channel (serial monitor).

/* Derived Constants */
const uint16_t TXBATT_AMIN = (uintmax_t)TXBATT_VMIN * (ADC_MAXREAD + 1) / (ADC_MAXVOLT * TXBATT_DIVFACTOR);
const uint16_t TXBATT_AMAX = (uintmax_t)TXBATT_VMAX * (ADC_MAXREAD + 1) / (ADC_MAXVOLT * TXBATT_DIVFACTOR);
const uint16_t RXBATT_AMIN = (uintmax_t)RXBATT_VMIN * (ADC_MAXREAD + 1) / (ADC_MAXVOLT * RXBATT_DIVFACTOR);
const uint16_t RXBATT_AMAX = (uintmax_t)RXBATT_VMAX * (ADC_MAXREAD + 1) / (ADC_MAXVOLT * RXBATT_DIVFACTOR);

#ifdef BIDIRECTIONAL_DRIVE
	const uint16_t MIDPOINT = (PULSE_MIN + PULSE_MAX) / 2;
	const uint16_t DRIVE_MIN = (MIDPOINT - DRIVE_RANGE);
	const uint16_t DRIVE_MAX = (MIDPOINT + DRIVE_RANGE);
#else
	const uint16_t DRIVE_MIN = PULSE_MIN;
	const uint16_t DRIVE_MAX = PULSE_MIN + DRIVE_RANGE * 2;
#endif

#endif