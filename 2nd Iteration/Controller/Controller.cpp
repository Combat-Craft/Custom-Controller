/*
	Changes from Firmware 1.0.0:
	> Created Parameters.h file to separate program parameters from source code.
	> Added the weapon motor pulse width value to the TX payload in loop().
	> Added conditional code inclusion for bidirectional/unidirectional ESC firmware.
	> Added serial print string indicating which radio channel is selected by the user.
	> Changed the live information feed to include drive, weapon, and battery information.
	> Removed the Battery class in favour of constants defined within Parameters.h.
	> Added a reset prompt on the LCD for when the radio module gets disconnected.
	> Changed radio configuration to use dynamic payload lengths and accept ack packets.
	> Changed the name of the controller to "CombatCraftCB1" and the firmware version to 1.0.1.
	> Added a menu on startup for the user to select whether to run the ESC calibration.
	> Added the ESC calibration code (which will now be handled remotely by the controller).
	> Removed code for reading ESC calibration information from the receiver.
	> Changed the structure of the TX payload in loop() to uint16_t array.
	> Added battery graphics to visually display battery charge on the LCD in real time.

TODO:

	> Figure out how to detect that the receiver has reset itself.
	> Figure out how to fix the battery measurement logic.
*/

/* Library Includes */
#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>   // I2C LCD Module
#include <RF24.h>                    // NRF24L01 Module
#include "HeaderFiles/Joystick.h"    // Joystick Module
#include "HeaderFiles/Parameters.h"  // Program Parameters

/* Helper Function Prototypes */
void LCDWrapText(LiquidCrystal_PCF8574 & display, char const * text);
void LCDBatteryGraphic(LiquidCrystal_PCF8574 & display, uint16_t test_value, uint16_t minimum, uint16_t maximum, uint8_t length);

/* Bitmaps for Battery Graphic Display on LCD module */
uint8_t const battery_graphics[] PROGMEM = {
  0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00,  // Bottom Section - Empty
  0x1F, 0x18, 0x1C, 0x1C, 0x1E, 0x1E, 0x1F, 0x00,  // Bottom Section - Half-Full
  0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,  // Bottom Section - Full
  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00,  // Middle Section - Empty
  0x1F, 0x18, 0x1C, 0x1C, 0x1E, 0x1E, 0x1F, 0x00,  // Middle Section - Half-Full
  0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,  // Middle Section - Full
  0x1E, 0x02, 0x03, 0x03, 0x03, 0x02, 0x1E, 0x00,  // Top Section - Empty
  0x1E, 0x1E, 0x1F, 0x1F, 0x1F, 0x1E, 0x1E, 0x00,  // Top Section - Full
};

/* Component Instantiations */
Joystick drive(JOYSTICK_X, JOYSTICK_Y, JOYSTICK_B, XAXIS_REVERSED, YAXIS_REVERSED, BUTTON_REVERSED);
LiquidCrystal_PCF8574 display(LCD_ADDRESS);
RF24 radio(RADIO_CE, RADIO_CSN);

/* Global Variable Definitions */
uint8_t packets_lost = 0;

void setup(void) {
	/* Initialize the serial console at 115200 baud. */
	Serial.begin(SERIAL_BAUD);
	while (!Serial) {}

	/* Initialize the display with the desired behaviour. */
	Serial.println("\nInitializing the LCD module.");
	display.begin(LCD_COLUMNS, LCD_ROWS);
	display.noAutoscroll();
	display.noBlink();
	display.noCursor();
	display.setBacklight(255);

	/* Copy the battery graphics bitmaps into CGRAM.*/
	for (uint8_t char_index = 0; char_index < 8; char_index++) {
		uint8_t bitmap[8];
		memcpy(bitmap, &battery_graphics[char_index * 8], 8);
		display.createChar(char_index, bitmap);
	}

	/* Display the name of the controller and the firmware version. */
	display.setCursor(3, 1);
	display.print("CombatCraftCB2");
	display.setCursor(3, 2);
	display.print("Firmware 1.0.1");
	delay(2000);

	/* Deterime the true origin of the joystick to prevent drive drift. */
	Serial.println("\nRecording the joystick origin.");
	drive.setOrigins(true, true, RECORD_TIME);
	Serial.print("X-Axis Origin: ");
	Serial.println(drive.getOriginX());
	Serial.print("Y-Axis Origin: ");
	Serial.println(drive.getOriginY());

	/* Verify that the NRF24L01 radio module is connected. */
	Serial.println("\nConnecting to the radio module.");
	while (!radio.begin()) {
		/* Inform the user that there's a problem with the radio module. */
		Serial.println("Could not connect to the radio module.");
		LCDWrapText(display, "Could not connect to the radio module.");
		display.setCursor(6, 3);
		display.print(">Retry?");

		/* Allow the user to retry the connection. */
		while (drive.readButton() == LOW) {}
		while (drive.readButton() == HIGH) {}
	}

	/* Initialize the radio module with the desired behaviour. */
	Serial.println("\nConfiguring the radio module.");
	Serial.println("Dynamic payloads enabled: true");
	radio.enableDynamicPayloads();
	Serial.println("Acknowledgement payloads enabled: false");
	radio.disableAckPayload();
	radio.flush_tx();

	/* Set the reading and writing pipe addresses. */
	Serial.print("RX address: ");
	char const rx_address[] PROGMEM = "CNTRL";
	Serial.println(rx_address);
	radio.openReadingPipe(1, (uint8_t*)rx_address);
	Serial.print("TX address: ");
	char const tx_address[] PROGMEM = "ROBOT";
	Serial.println(tx_address);
	radio.openWritingPipe((uint8_t*)tx_address);

	/* Create a menu for the user to choose a radio channel. */
	uint8_t const channels[3] = {RADIO_CH1, RADIO_CH2, RADIO_CH3};
	Serial.println("\nDrawing the channel selection menu.");
	display.setCursor(0, 0);
	display.print("Select Radio Channel");
	display.setCursor(0, 1);
	display.print(" Channel 1: ");
	display.print(RADIO_CH1 + CH_TO_MHZ);
	display.print(" MHz");
	display.setCursor(0, 2);
	display.print(" Channel 2: ");
	display.print(RADIO_CH2 + CH_TO_MHZ);
	display.print(" MHz");
	display.setCursor(0, 3);
	display.print(" Channel 3: ");
	display.print(RADIO_CH3 + CH_TO_MHZ);
	display.print(" MHz");

	/* Facilitate menu functions while the user is deciding. */
	Serial.println("\nObtaining channel from the user.");
	uint8_t channel_selection = 1, channel_previous = 2;
	while (drive.readButton() == LOW) {
		/* Define joystick 'up' and 'down' events. */
		uint16_t const joystick_value = drive.readValueY();
		uint16_t const down_threshold = (drive.getOriginY() + drive.getMinimumY()) / 2;
		uint16_t const up_threshold = (drive.getOriginY() + drive.getMaximumY()) / 2;

		/* Change the selected option depending on the event that occurred. */
		if (joystick_value > up_threshold && channel_selection > 1) {
			channel_selection--;
		} else if (joystick_value < down_threshold && channel_selection < 3) {
			channel_selection++;
		} else if (channel_selection == channel_previous) continue;

		/* Update the menu to highlight the newly-selected option. */
		display.setCursor(0, channel_previous);
		display.print(" ");
		display.setCursor(0, channel_selection);
		display.print(">");
		channel_previous = channel_selection;
		delay(MENU_DELAY);
	}
	while (drive.readButton() == HIGH) {}
	delay(MENU_DELAY);

	Serial.print("Radio Channel: ");
	Serial.print(channels[channel_selection-1]);
	Serial.print(" (");
	Serial.print(channels[channel_selection-1] + CH_TO_MHZ);
	Serial.println(" MHz)");

	/* Configure the radio module to use the selected channel. */
	Serial.println("\nSetting channel on the radio module.");
	radio.setChannel(channels[channel_selection-1]);

	/* Create a menu deciding whether or not to perform the ESC calibration sequence. */
	Serial.println("\nDrawing the ESC arming menu.");
	LCDWrapText(display, "Have the ESCs been armed?");
	display.setCursor(0, 3);
	display.print("    YES       NO    ");

	/* Facilitate menu functions while the user is deciding. */
	Serial.println("\nObtaining decision from the user.");
	bool escs_armed = true, escs_armed_prev = false;
	while (drive.readButton() == LOW) {
		/* Define joystick 'up' and 'down' events. */
		uint16_t const joystick_value = drive.readValueX();
		uint16_t const left_threshold = (drive.getOriginX() + drive.getMinimumX()) / 2;
		uint16_t const right_threshold = (drive.getOriginX() + drive.getMaximumX()) / 2;

		/* Change the selected option depending on the event that occurred. */
		if (joystick_value > right_threshold && escs_armed == true) {
			escs_armed = false;
		} else if (joystick_value < left_threshold && escs_armed == false) {
			escs_armed = true;
		} else if (escs_armed == escs_armed_prev) continue;

		/* Update the menu to highlight the newly-selected option. */
		display.setCursor((escs_armed ? 13 : 3), 3);
		display.print(" ");
		display.setCursor((escs_armed ? 3 : 13), 3);
		display.print(">");
		escs_armed_prev = escs_armed;
		delay(MENU_DELAY);
	}
	while (drive.readButton() == HIGH) {}

	/* Establish a connection with the receiver. */
	Serial.println("\nConnecting to the robot.");
	LCDWrapText(display, "Connecting to the robot.");
	char const init_msg[] PROGMEM = "CombatCraftCB2";
	while (true) {
		/* Send messages until an acknowledgement is received. */
		Serial.println("\nSending connection message.");
		radio.stopListening();
		while (!radio.write(init_msg, strlen(init_msg))) delay(WRITE_DELAY);
		Serial.println("Connection message acknowledged.");
		Serial.println("Listening for reply.");

		/* If a message is received, compare it with the sent message. */
		bool reply_match;
		radio.startListening();
		size_t const reply_timeout = millis() + REPLY_TIMEOUT;
		while (millis() < reply_timeout) {
			/* Wait for a message to arrive from the receiver. */
			if (!radio.available()) continue;

			/* Read the message into memory. */
			uint8_t const read_length = radio.getDynamicPayloadSize();
			char read_buffer[33];
			memset(read_buffer, 0, 33 * sizeof(char));
			radio.read(read_buffer, read_length);
			Serial.print("Received: ");
			Serial.print(read_buffer);
			Serial.print(" (Bytes: ");
			Serial.print(read_length);
			Serial.println(")");

			/* Proceed only if the sent and received messages match. */
			reply_match = !strcmp(read_buffer, init_msg);
			if (reply_match) break;
			Serial.println("Ignoring unexpected response.");
		}
		/* If there was no matching response, send the message again. */
		if (reply_match) break;
		Serial.println("Reply timed out.");
	}
	Serial.println("\nConnected to the robot.");

	/* Perform the ESC calibration sequence only if the ESCs aren't armed. */
	if (!escs_armed) {
		Serial.println("\nStarting ESC calibration.");
		uint16_t esc_payload[3] = {PULSE_MAX, PULSE_MAX, PULSE_MAX};
		unsigned long esccal_holdtime;
		
		Serial.println("Holding outputs at maximum pulse width.");
		LCDWrapText(display, "Holding outputs at maximum pulse width.");
		packets_lost = 0;
		radio.stopListening();
		esccal_holdtime = millis() + ESCCAL_DELAY;
		while (millis() < esccal_holdtime) {
			packets_lost = 0;
			while (!radio.write(esc_payload, 3 * sizeof(uint16_t))) {
				if (++packets_lost > PACKET_LOSS) return;
				delay(WRITE_DELAY);
			}
		}

		Serial.println("Gradually decreasing output pulse width.");
		LCDWrapText(display, "Gradually decreasing output pulse width.");
		while (esc_payload[0] > PULSE_MIN) {
			esc_payload[0]--;
			esc_payload[1]--;
			esc_payload[2]--;
			
			packets_lost = 0;
			while (!radio.write(esc_payload, 3 * sizeof(uint16_t))) {
				if (++packets_lost > PACKET_LOSS) return;
				delay(WRITE_DELAY);
			}
		}

		Serial.println("Holding outputs at minimum pulse width.");
		LCDWrapText(display, "Holding outputs at minimum pulse width.");
		packets_lost = 0;
		esccal_holdtime = millis() + ESCCAL_DELAY;
		while (millis() < esccal_holdtime) {
			packets_lost = 0;
			while (!radio.write(esc_payload, 3 * sizeof(uint16_t))) {
				if (++packets_lost > PACKET_LOSS) return;
				delay(WRITE_DELAY);
			}
		}
	}

	Serial.println("\nEnabling acknowledgement payloads.");
	radio.flush_tx();
	radio.enableAckPayload();
	radio.stopListening();

	packets_lost = 0;
	Serial.println("\nStarting remote control.");
}

void loop(void) {
	/* Check for fatal errors, and determine if the program should stop. */
	if (packets_lost > PACKET_LOSS) {
		/* Inform the user of the error that was detected. */
		if (!radio.isChipConnected()) {
			Serial.println("\nRadio module disconnected.");
			LCDWrapText(display, "Radio module disconnected.");
		} else {
			Serial.println("\nConnection to robot lost.");
			LCDWrapText(display, "Connection to robot lost.");
		}
		display.setCursor(6, 3);
		display.print(">Reset?");
		
		/* Force the controller to be reset. */
		while (drive.readButton() == LOW) {}
		while (drive.readButton() == HIGH) {}
		Serial.println("\nResetting controller.");
		setup();
		return;
	}

	/* Sample all controller inputs. */
	uint16_t const xaxis_test = drive.readValueX();
	uint16_t const yaxis_test = drive.readValueY();

	pinMode(WEAPON_SW, INPUT);
	bool static weapon_enable = false;
	bool const weapon_test = (digitalRead(WEAPON_SW) == HIGH);
	if (weapon_test == false) weapon_enable = true;

	/* Compute the common throttle value for both motors, upon which the differential value is superimposed. */
#ifdef BIDIRECTIONAL_DRIVE
	/* Conditional code inclusion for bidirectional ESC firmware. */
	int16_t common_mode;
	if (yaxis_test > drive.getOriginY() + NOISE_THRESH) {
	  common_mode = map(yaxis_test, drive.getOriginY() + NOISE_THRESH, drive.getMaximumY(), MIDPOINT, DRIVE_MAX);
	} else if (yaxis_test < drive.getOriginY() - NOISE_THRESH) {
	  common_mode = map(yaxis_test, drive.getOriginY() - NOISE_THRESH, drive.getMinimumY(), MIDPOINT, DRIVE_MIN);
	} else {
	  common_mode = MIDPOINT;
	}
	common_mode = constrain(common_mode, DRIVE_MIN, DRIVE_MAX);
#else
	/* Conditional code inclusion for unidirectional ESC firmware. */
	int16_t common_mode;
	if (yaxis_test > drive.getOriginY() + NOISE_THRESH) {
	  common_mode = map(yaxis_test, drive.getOriginY() + NOISE_THRESH, drive.getMaximumY(), DRIVE_MIN, DRIVE_MAX);
	} else {
	  common_mode = DRIVE_MIN;
	}
	common_mode = constrain(common_mode, DRIVE_MIN, DRIVE_MAX);
#endif

	/* Compute the differential throttle value between the left and right motors for differential steering. */
	int16_t differential;
	if (xaxis_test > drive.getOriginX() + NOISE_THRESH) {
	  differential = map(xaxis_test, drive.getOriginX() + NOISE_THRESH, drive.getMaximumX(), 0, DRIVE_MAX - DRIVE_MIN);
	} else if (xaxis_test < drive.getOriginX() - NOISE_THRESH) {
	  differential = map(xaxis_test, drive.getOriginX() - NOISE_THRESH, drive.getMinimumX(), 0, DRIVE_MIN - DRIVE_MAX);
	} else {
	  differential = 0;
	}
	differential = constrain(differential, DRIVE_MIN - DRIVE_MAX, DRIVE_MAX - DRIVE_MIN);

	/* Ensure that all pulse width output values are within the range PULSE_MIN to PULSE_MAX. */
	uint16_t const left_initial = common_mode + (differential / 2);
	uint16_t const right_initial = common_mode - (differential / 2);
	uint16_t const left_bounded = constrain(left_initial, DRIVE_MIN, DRIVE_MAX);
	uint16_t const right_bounded = constrain(right_initial, DRIVE_MIN, DRIVE_MAX);
	uint16_t const left_pulse = (left_initial != left_bounded ? left_bounded : right_bounded + differential);
	uint16_t const right_pulse = (right_initial != right_bounded ? right_bounded : left_bounded - differential);
	uint16_t const weapon_pulse = (weapon_enable && weapon_test ? WEAPON_ON : WEAPON_OFF);

	/* Send a packet containing the PWM values, and track whether it was lost. */
	uint16_t payload[3] = {left_pulse, right_pulse, weapon_pulse};
	radio.stopListening();
	unsigned long tx_start = millis();
	bool const packet_ack = radio.write(payload, 3 * sizeof(uint16_t));
	unsigned long tx_finish = millis();
	if (packet_ack) packets_lost = 0;
	else packets_lost++;

	uint16_t static rxbatt_test = 0;
	if (packet_ack) radio.read(&rxbatt_test, sizeof(uint16_t));

	/* Update the display every 1.5 seconds. (For some reason, it happens much more frequently than that.) */
	unsigned long static packet_echo_timer = 0;
	if (millis() - packet_echo_timer < PRINT_INTERVAL) return;
	else packet_echo_timer = millis();

	char row_buffer[LCD_COLUMNS+1];
	uint8_t const left_percent = map(left_pulse, DRIVE_MIN, DRIVE_MAX, 0, 100);
	uint8_t const right_percent = map(right_pulse, DRIVE_MIN, DRIVE_MAX, 0, 100);
	snprintf(row_buffer, LCD_COLUMNS+1, ("L: %3" PRIu8 "%%      R: %3" PRIu8 "%%"), left_percent, right_percent);
	display.setCursor(0, 0);
	display.print(row_buffer);

	snprintf(row_buffer, LCD_COLUMNS+1, "Weapon: %-12s", (weapon_enable && weapon_test ? "ACTIVE" : "INACTIVE"));
	display.setCursor(0, 1);
	display.print(row_buffer);

	uint16_t const txbatt_test = analogRead(BATTERY);
	uint8_t const txbatt_percent = (uint8_t)constrain(map(txbatt_test, TXBATT_AMIN, TXBATT_AMAX, 0, 100), 0, 100);
	uint16_t const txbatt_voltage = (uint16_t)constrain(map(txbatt_test, TXBATT_AMIN, TXBATT_AMAX, TXBATT_VMIN, TXBATT_VMAX), 0, UINT16_MAX);
	snprintf(row_buffer, LCD_COLUMNS+1, ("TX: %3" PRIu8 "%% %5.2fV \0\0\0\0"), txbatt_percent, (txbatt_voltage / 1000.0f));
	display.setCursor(0, 2);
	display.print(row_buffer);
	LCDBatteryGraphic(display, txbatt_test, TXBATT_AMIN, TXBATT_AMAX, 4);

	uint8_t const rxbatt_percent = (uint8_t)constrain(map(rxbatt_test, RXBATT_AMIN, RXBATT_AMAX, 0, 100), 0, 100);
	uint16_t const rxbatt_voltage = (uint16_t)constrain(map(rxbatt_test, RXBATT_AMIN, RXBATT_AMAX, RXBATT_VMIN, RXBATT_VMAX), 0, UINT16_MAX);
	snprintf(row_buffer, LCD_COLUMNS+1, ("RX: %3" PRIu8 "%% %5.2fV \0\0\0\0"), rxbatt_percent, (rxbatt_voltage / 1000.0f));
	display.setCursor(0, 3);
	display.print(row_buffer);
	LCDBatteryGraphic(display, rxbatt_test, RXBATT_AMIN, RXBATT_AMAX, 4);

#ifdef PRINT_EXTRA

	Serial.println();
	Serial.print("Payload: ");
	Serial.print(left_pulse, HEX);
	Serial.print(right_pulse, HEX);
	Serial.print(weapon_pulse, HEX);
	Serial.print(" (Bytes: ");
	Serial.print(3 * sizeof(uint16_t));
	Serial.print(", Time: ");
	Serial.print(tx_finish - tx_start);
	Serial.println(" ms)");

	Serial.print("ACK Payload: ");
	Serial.print(rxbatt_test, HEX);
	Serial.print(" (Bytes: ");
	Serial.print(sizeof(uint16_t));
	Serial.println(")");

	Serial.print("Left Motor: ");
	Serial.print(left_pulse);
	Serial.print(", Right Motor: ");
	Serial.print(right_pulse);
	Serial.print(", Weapon Motor: ");
	Serial.println(weapon_pulse);

#endif
}

void LCDWrapText(LiquidCrystal_PCF8574 & display, char const * text) {
	char const visible[] PROGMEM = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

	display.clear();
	char const * token_head = strpbrk(text, visible);
	if (token_head == NULL) return;
	size_t token_length = strspn(token_head, visible);

	for (uint8_t row_index = 0; row_index < 4; row_index++) {
		char row_buffer[LCD_COLUMNS+1] = "                    ";

		if (token_length > 20) {
			memcpy(row_buffer, token_head, 20);
			token_head += 20 * sizeof(char);
			display.setCursor(0, row_index);
			display.print(row_buffer);
			continue;
		}

		uint8_t row_length = 0;
		while (row_length + token_length <= 20) {
			memcpy(row_buffer + row_length * sizeof(char), token_head, token_length);
			row_length += token_length + 1;
			token_head = strpbrk(token_head + token_length * sizeof(char), visible);
			if (token_head == NULL) break;
			token_length = strspn(token_head, visible);
		}

		display.setCursor(0, row_index);
		display.print(row_buffer);
		if (token_head == NULL) return;
	}
}

void LCDBatteryGraphic(LiquidCrystal_PCF8574 & display, uint16_t test_value, uint16_t minimum, uint16_t maximum, uint8_t length) {
	if (length < 2 || length > LCD_COLUMNS) return;

	uint8_t const states = length * 2;
	uint16_t const range = (maximum - minimum) / states;

	for (uint8_t char_index = 0; char_index < length; char_index++) {
		uint8_t cgram_address = 0;
		if (char_index > 0) cgram_address += 3;
		if (test_value > minimum + range * (2 * char_index + 1)) cgram_address++;
		if (char_index == length - 1) cgram_address += 3;
		else if (test_value > minimum + range * (2 * char_index + 2)) cgram_address++;
		display.write(cgram_address);
	}
}