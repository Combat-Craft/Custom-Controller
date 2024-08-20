#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>
#include <RF24.h>
#include "HeaderFiles/Battery.h"
#include "HeaderFiles/Joystick.h"

#define ZERO_THR 1000
#define FULL_THR 1250
#define NOISE_TH 5

Joystick drive(36, 39, 25, false, true, true);
// Joystick other(35, 32, 33, true, false, true);
LiquidCrystal_PCF8574 display(0x27);
RF24 radio(2, 15);

unsigned long packet_echo_timer = -1;

void setup(void) {
	Serial.begin(115200);
	while (!Serial) {}

	Serial.println(F("Initializing LCD module."));
	display.begin(20, 4);
	display.setBacklight(255);
	display.noAutoscroll();
	display.noCursor();
	display.noBlink();
	display.setCursor(3, 1);
	display.print(F("CombatCraftCB3"));
	display.setCursor(3, 2);
	display.print(F("Firmware 1.0.0"));
	delay(2000);

	Serial.println(F("\nStarting drive joystick calibration."));

	// Serial.println(F("\nPlease move the joystick in a circle repeatedly. (3s)"));
	// LCDWrapText(display, "\nPlease move the joystick in a circle repeatedly. (3s)");
	// Serial.println(F("Recording joystick extrema."));
	// drive.setExtrema(true, true, 3000);

	// Serial.print(F("X-Axis Minimum: "));
	// Serial.println(drive.getMinimumX());
	// Serial.print(F("X-Axis Maximum: "));
	// Serial.println(drive.getMaximumX());
	// Serial.print(F("Y-Axis Minimum: "));
	// Serial.println(drive.getMinimumY());
	// Serial.print(F("Y-Axis Maximum: "));
	// Serial.println(drive.getMaximumY());

	// Serial.println(F("\nPlease release the drive joystick. (3s)"));
	// LCDWrapText(display, "Please release the drive joystick. (3s)");
	// delay(2000);
	Serial.println(F("Recording joystick origin."));
	drive.setOrigins(true, true, 100);

	Serial.print(F("X-Axis Origin: "));
	Serial.println(drive.getOriginX());
	Serial.print(F("Y-Axis Origin: "));
	Serial.println(drive.getOriginY());

	Serial.println(F("\nDrive joystick calibration complete."));

	Serial.println(F("\nConnecting to radio module"));
	if (!radio.begin()) {
		Serial.println(F("Could not connect to radio module."));
		LCDWrapText(display, "Could not connect to radio module.");
		while (true) {}
	}
	Serial.println(F("Radio module connected."));

	Serial.println(F("\nConfiguring radio module."));

	Serial.println(F("Setting transmit power: PA_MAX"));
	radio.setPALevel(RF24_PA_MAX);

	Serial.println(F("Setting air data rate: 2 Mbps"));
	radio.setDataRate(RF24_2MBPS);

	Serial.println(F("Setting auto-acks: true"));
	radio.setAutoAck(true);

	Serial.println(F("Setting number of retries: 255"));
	Serial.println(F("Setting delay between retries: 64 ms"));
	radio.setRetries(255, 255);

	Serial.print(F("Setting RX address: "));
	uint8_t const rx_address[] PROGMEM = "CNTRL";
	Serial.println((char*)rx_address);
	radio.openReadingPipe(1, rx_address);

	Serial.print(F("Setting TX address: "));
	uint8_t const tx_address[] PROGMEM = "ROBOT";
	Serial.println((char*)tx_address);
	radio.openWritingPipe(tx_address);

	Serial.print(F("Setting TX payload size: "));
	uint8_t const tx_size = 32;
	Serial.println(tx_size);
	radio.setPayloadSize(tx_size);

	Serial.println(F("Radio configuration complete."));

	Serial.println(F("\nSetting up channel selection menu."));
	display.clear();
	display.setCursor(0, 0);
	display.print(F("Select Radio Channel"));
	display.setCursor(1, 1);
	display.print(F("Channel 1: 2424 MHz"));
	display.setCursor(1, 2);
	display.print(F("Channel 2: 2466 MHz"));
	display.setCursor(1, 3);
	display.print(F("Channel 3: 2508 MHZ"));

	Serial.println(F("Obtaining channel from user."));
	uint8_t channel_selection = 1, channel_previous = 2;
	while (drive.readButton() == LOW) {
		uint16_t const joystick_value = drive.readValueY();
		uint16_t const down_threshold = (drive.getOriginY() + drive.getMinimumY()) / 2;
		uint16_t const up_threshold = (drive.getOriginY() + drive.getMaximumY()) / 2;

		if (joystick_value > up_threshold && channel_selection > 1) {
			channel_selection--;
		} else if (joystick_value < down_threshold && channel_selection < 3) {
			channel_selection++;
		} else if (channel_selection == channel_previous) continue;

		display.setCursor(0, channel_previous);
		display.print(" ");
		display.setCursor(0, channel_selection);
		display.print(">");
		channel_previous = channel_selection;
		delay(250);
	}
	while (drive.readButton() == HIGH) {}

	Serial.println(F("Setting channel on radio module."));
	uint8_t const channels[3] = {24, 66, 108};
	radio.setChannel(channels[channel_selection-1]);

	Serial.println(F("\nConnecting to robot."));
	LCDWrapText(display, "Connecting to robot.");

	char const init_msg[] PROGMEM = "CombatCraftCB3";
	while (true) {
		Serial.println(F("Spamming robot with connection messages."));
		radio.stopListening();
		while (!radio.write(init_msg, strlen(init_msg))) delay(100);
		Serial.println(F("Connection message acknowledged."));
		Serial.println(F("Listening for reply."));

		bool reply_match;
		radio.startListening();
		size_t const reply_timeout = millis() + 200;
		while (millis() < reply_timeout) {
			if (!radio.available()) continue;

			char read_buffer[33];
			memset(read_buffer, 0, 33);
			radio.read(read_buffer, 32);
			Serial.print(F("Received: "));
			Serial.println(read_buffer);

			reply_match = !strcmp(read_buffer, init_msg);
			if (reply_match) break;
			Serial.println(F("Ignoring unexpected response."));
		}
		if (reply_match) break;
		Serial.println(F("Reply timed out."));
	}
	Serial.println(F("Response matched expected reply."));

	Serial.println(F("Connected to robot."));
	LCDWrapText(display, "Connected to robot.");
	delay(1000);

	Serial.println(F("\nEchoing robot setup messages.\n"));
	char const setup_complete[] PROGMEM = "\nRobot setup complete.\n";
	radio.startListening();
	while (true) {
		radio.startListening();
		while (!radio.available()) {}

		char read_buffer[33];
		memset(read_buffer, 0, 33);
		radio.read(read_buffer, 32);

		char const * target = &read_buffer[0];
		char const * message = &read_buffer[1];
		if (target[0] & 0x05 != 0) Serial.print(message);
		if (target[0] & 0x50 != 0) LCDWrapText(display, message);
		if (!strcmp(message, setup_complete)) {
			radio.stopListening();
			while (!radio.write(setup_complete, strlen(setup_complete))) delay(100);
			break;
		}
	}

	Serial.println(F("\nStarting remote control."));

	display.clear();
	display.setCursor(0, 0);
	display.print("X-Axis:");
	display.setCursor(0, 1);
	display.print("Y-Axis:");
	display.setCursor(0, 2);
	display.print(F("Left Motor:"));
	display.setCursor(0, 3);
	display.print(F("Right Motor:"));
}

void loop(void) {
	uint16_t const joystick_x = drive.readValueX();
	uint16_t const joystick_y = drive.readValueY();

	int16_t common_mode;
	if (joystick_y > drive.getOriginY() + NOISE_TH) {
		common_mode = map(joystick_y, drive.getOriginY() + NOISE_TH, drive.getMaximumY(), ZERO_THR, FULL_THR);
	} else {
		common_mode = ZERO_THR;
	}
	common_mode = constrain(common_mode, ZERO_THR, FULL_THR);

	int16_t differential;
	if (joystick_x > drive.getOriginX() + NOISE_TH) {
		differential = map(joystick_x, drive.getOriginX() + NOISE_TH, drive.getMaximumX(), 0, FULL_THR - ZERO_THR);
	} else if (joystick_x < drive.getOriginX() - NOISE_TH) {
		differential = map(joystick_x, drive.getOriginX() - NOISE_TH, drive.getMinimumX(), 0, ZERO_THR - FULL_THR);
	} else {
		differential = 0;
	}
	differential = constrain(differential, ZERO_THR - FULL_THR, FULL_THR - ZERO_THR);

	uint16_t const lpwm_initial = common_mode + (differential / 2);
	uint16_t const rpwm_initial = common_mode - (differential / 2);
	uint16_t const lpwm_bounded = constrain(lpwm_initial, ZERO_THR, FULL_THR);
	uint16_t const rpwm_bounded = constrain(rpwm_initial, ZERO_THR, FULL_THR);
	uint16_t const lpwm_correct = (lpwm_initial != lpwm_bounded ? lpwm_bounded : rpwm_bounded + differential);
	uint16_t const rpwm_correct = (rpwm_initial != rpwm_bounded ? rpwm_bounded : lpwm_bounded - differential);

	radio.stopListening();
	uint8_t const payload[4] = {lpwm_correct, lpwm_correct >> 8, rpwm_correct, rpwm_correct >> 8};
	unsigned long tx_start = millis();
	bool const packet_ack = radio.write(payload, 4);
	unsigned long tx_finish = millis();

	uint8_t static packets_lost = 0;
	if (packet_ack) packets_lost = 0;
	else packets_lost++;

	if (packets_lost >= 25) {
		Serial.println(F("\nConnection to robot lost."));
		LCDWrapText(display, "Connection to robot lost.");
		display.setCursor(6, 3);
		display.print(">Reset?");
		
		while (drive.readButton() == LOW) {}
		while (drive.readButton() == HIGH) {}
		Serial.println(F("\nResetting controller."));
		setup();
		return;
	}

	if (packet_echo_timer + millis() < 1500) return;
	else packet_echo_timer = millis();

	char display_buffer[5];
	snprintf(display_buffer, 5, "%-4i", joystick_x);
	display.setCursor(8, 0);
	display.print(display_buffer);
	snprintf(display_buffer, 5, "%-4i", joystick_y);
	display.setCursor(8, 1);
	display.print(display_buffer);
	snprintf(display_buffer, 5, "%-4i", lpwm_correct);
	display.setCursor(12, 2);
	display.print(display_buffer);
	snprintf(display_buffer, 5, "%-4i", rpwm_correct);
	display.setCursor(13, 3);
	display.print(display_buffer);
	
	// Serial.println();
	// Serial.println(packet_ack ? F("Packet acknowledged.F(") : F("Packet lost.")));
	// Serial.print(F("Time: "));
	// Serial.print(tx_finish - tx_start);
	// Serial.println(F(" ms"));
	// Serial.print(F("Payload: "));
	// Serial.print(payload[0], HEX);
	// Serial.print(payload[1], HEX);
	// Serial.print(payload[2], HEX);
	// Serial.println(payload[3], HEX);

	// Serial.println();
	// Serial.print(F("Joystick X: "));
	// Serial.println(joystick_x);
	// Serial.print(F("Joystick Y: "));
	// Serial.println(joystick_y);
	// Serial.print(F("Left Motor PWM: "));
	// Serial.println(lpwm_correct);
	// Serial.print(F("Right Motor PWM: "));
	// Serial.println(rpwm_correct);
}

void LCDWrapText(LiquidCrystal_PCF8574 & display, char const * text) {
	char const visible[] PROGMEM = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

	display.clear();
	char const * token_head = strpbrk(text, visible);
	if (token_head == NULL) return;
	size_t token_length = strspn(token_head, visible);

	for (uint8_t row_index = 0; row_index <= 4; row_index++) {
		char row_buffer[21] = "                    ";

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