/* Library Includes */
#include <Arduino.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include "HeaderFiles/Parameters.h"

/* Component Instantiations */
RF24 radio(RADIO_CE, RADIO_CSN);
Servo left_motor, right_motor, weapon_motor;

void setup(void) {
	/* Initialize the serial console at 115200 baud. */
	Serial.begin(SERIAL_BAUD);
	while (!Serial) {}

	/* Verify that the NRF24L01 radio module is connected. */
	Serial.println("\nConnecting to the radio module.");
	while (!radio.begin()) {
		/* Inform the user that there's a problem with the radio module. */
		Serial.println("Could not connect to the radio module.");
		while (true) {}
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
	char const rx_address[] PROGMEM = "ROBOT";
	Serial.println(rx_address);
	radio.openReadingPipe(1, (uint8_t*)rx_address);
	Serial.print("TX address: ");
	char const tx_address[] PROGMEM = "CNTRL";
	Serial.println(tx_address);
	radio.openWritingPipe((uint8_t*)tx_address);

	/* Establish a connection with the controller. */
	Serial.println("\nConnecting to the transmitter.");
	char const init_msg[] PROGMEM = "CombatCraftCB2";
	uint8_t const channels[3] = {RADIO_CH1, RADIO_CH2, RADIO_CH3};
	size_t const channel_wait = CHAN_DELAY;
	while (true) {
		/* Start scanning the three radio channels cyclically. */
		radio.startListening();
		uint8_t const channel_index = (millis() / channel_wait) % 3;
		if (radio.getChannel() != channels[channel_index]) {
			radio.setChannel(channels[channel_index]);
			Serial.print("\nChanging to channel ");
			Serial.print(channel_index+1);
			Serial.print(" (");
			Serial.print(channels[channel_index] + CH_TO_MHZ);
			Serial.println(" MHz).");
		}

		/* When a message is received, read it into memory. */
		if (!radio.available()) continue;
		uint8_t const read_length = radio.getDynamicPayloadSize();
		char read_buffer[33];
		memset(read_buffer, 0, 33 * sizeof(char));
		radio.read(read_buffer, read_length);
		Serial.print("Received: ");
		Serial.print(read_buffer);
		Serial.print(" (Bytes: ");
		Serial.print(read_length);
		Serial.println(")");

		/* Compare the received message against the expected message. */
		if (strcmp(read_buffer, init_msg)) continue;

		/* If the messages matched, send a reply to the controller. */
		bool reply_succeeded;
		radio.stopListening();
		unsigned long const reply_timeout = millis() + REPLY_TIMEOUT;
		while (millis() < reply_timeout) {
			reply_succeeded = radio.write(read_buffer, read_length);
			if (reply_succeeded) break;
			else delay(WRITE_DELAY); 
		}
		if (reply_succeeded) break;
		else Serial.println("Reply timed out.");
	}

	/* Map each motor object to its signal output pin. */
	Serial.println("\nAttaching motors to output pins.");
	left_motor.attach(DRIVE_LEFT);
	right_motor.attach(DRIVE_RIGHT);
	weapon_motor.attach(WEAPON);

	/* Set up the radio to send battery information to the controller. */
	Serial.println("\nEnabling acknowledgement payloads.");
	radio.flush_tx();
	radio.enableAckPayload();
	radio.startListening();

	Serial.println("\nStarting remote control.");
}

void loop(void) {
	/* If the TX FIFO is empty, load it with a battery reading. */
	uint16_t static battery_test = 0;
	if (radio.isFifo(true, true)) {
		battery_test = analogRead(BATTERY);
		radio.writeAckPayload(1, &battery_test, sizeof(uint16_t));
	}

	/* Wait for a message to arrive from the controller. */
	unsigned long const watchdog_timeout = millis() + WATCHDOG;
	while (!radio.available() && millis() < watchdog_timeout) {}

	/* If the watchdog timeout is reached, force the robot to reset. */
	if (!radio.available()) {
		left_motor.detach();
		right_motor.detach();
		weapon_motor.detach();
		Serial.println("\nWatchdog timer expired.");
		Serial.println("Resetting robot.");
		setup();
		return;
	}

	/* Read pulse width data from controller into memory. */
	uint16_t payload[3];
	memset(payload, 0, 3 * sizeof(uint16_t));
	radio.read(payload, 3 * sizeof(uint16_t));

	/* Write updated pulse width values to motor outputs. */
	uint16_t const left_pulse = payload[0];
	uint16_t const right_pulse = payload[1];
	uint16_t const weapon_pulse = payload[2];
	left_motor.writeMicroseconds(left_pulse);
	right_motor.writeMicroseconds(right_pulse);
	weapon_motor.writeMicroseconds(weapon_pulse);

#ifdef PRINT_EXTRA

	unsigned long static packet_echo_timer = 0;
	if (millis() - packet_echo_timer < PRINT_INTERVAL) return;
	else packet_echo_timer = millis();

	Serial.println();
	Serial.print("Payload: ");
	Serial.print(left_pulse, HEX);
	Serial.print(right_pulse, HEX);
	Serial.print(weapon_pulse, HEX);
	Serial.print(" (Bytes: ");
	Serial.print(3 * sizeof(uint16_t));
	Serial.println(")");

	Serial.print("Left Motor: ");
	Serial.print(left_pulse);
	Serial.print(", Right Motor: ");
	Serial.print(right_pulse);
	Serial.print(", Weapon Motor: ");
	Serial.println(weapon_pulse);

#endif
}