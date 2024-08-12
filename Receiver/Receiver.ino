#include <RF24.h>
#include <ESP32Servo.h>

#define ZERO_THR 1000
#define FULL_THR 2000

#define TTY 0xA5
#define LCD 0x5A
#define ALL 0x55

RF24 radio(16, 17);
Servo left_motor, right_motor;

void setup(void) {
	Serial.begin(115200);
	while (!Serial) {}
	
	Serial.println(F("Connecting to radio module."));
	if (!radio.begin()) {
		Serial.println(F("Could not connect to radio module."));
		while (true) {}
	}
	Serial.println(F("Radio module connected."));

	Serial.println(F("Configuring radio module."));

	Serial.println(F("Setting transmit power: PA_MAX"));
	radio.setPALevel(RF24_PA_MAX);

	Serial.println(F("Setting air data rate: 2MBPS"));
	radio.setDataRate(RF24_2MBPS);

	Serial.println(F("Setting auto-acks: true"));
	radio.setAutoAck(true);

	Serial.println(F("Setting number of retries: 255"));
	Serial.println(F("Setting delay between retries: 64 ms"));
	radio.setRetries(255, 255);

	Serial.print(F("Setting RX address: "));
	uint8_t const rx_address[] PROGMEM = "ROBOT";
	Serial.println((char*)rx_address);
	radio.openReadingPipe(1, rx_address);

	Serial.print(F("Setting TX address: "));
	uint8_t const tx_address[] PROGMEM = "CNTRL";
	Serial.println((char*)tx_address);
	radio.openWritingPipe(tx_address);

	Serial.print(F("Setting TX payload size: "));
	uint8_t const tx_size = 32;
	Serial.println(tx_size);
	radio.setPayloadSize(tx_size);

	Serial.println(F("Radio configuration complete."));

	Serial.println(F("\nConnecting to transmitter."));
	char * const init_msg PROGMEM = "CombatCraftCB3";
	uint8_t const channels[3] = {24, 66, 108};
	size_t const channel_wait = 1000;
	while (true) {
		radio.startListening();
		uint8_t const channel_index = (millis() / channel_wait) % 3;
		if (radio.getChannel() != channels[channel_index]) {
			radio.setChannel(channels[channel_index]);
			Serial.print(F("Changing to channel "));
			Serial.print(channel_index+1);
			Serial.print(F(" ("));
			Serial.print(2000 + channels[channel_index]);
			Serial.println(F(" MHz)."));
		}

		if (!radio.available()) continue;
		char read_buffer[33];
		memset(read_buffer, 0, 33);
		radio.read(read_buffer, 32);
		Serial.print(F("Received: "));
		Serial.println(read_buffer);

		radio.stopListening();
		if (strcmp(read_buffer, init_msg)) continue;
		else if (!radio.write(read_buffer, 32)) continue;
		else break;
	}
	Serial.println(F("Connected to transmitter."));

	delay(1000);

	Serial.println(F("\nStarting ESC calibration."));
	NRFSendText(radio, ALL, "Starting ESC calibration.\n");
	delay(1000);
	left_motor.attach(4);
	right_motor.attach(2);

	Serial.println(F("Full throttle forward. (5s)"));
	NRFSendText(radio, ALL, "Full throttle forward. (5s)\n");
	left_motor.writeMicroseconds(FULL_THR);
	right_motor.writeMicroseconds(FULL_THR);
	delay(5000);

	Serial.println(F("Decreasing throttle."));
	NRFSendText(radio, ALL, "Desreasing throttle.\n");
	for (int pulse_width = FULL_THR; pulse_width >= ZERO_THR; pulse_width--) {
		left_motor.writeMicroseconds(pulse_width);
		right_motor.writeMicroseconds(pulse_width);
		delay(1);
	}

	Serial.println(F("Full throttle reverse. (5s)"));
	NRFSendText(radio, ALL, "Full throttle reverse. (5s)\n");
	delay(5000);

	Serial.println(F("ESC calibration complete."));
	NRFSendText(radio, ALL, "ESC calibration complete.\n");
	delay(1000);

	Serial.println(F("\nRobot setup complete."));
	NRFSendText(radio, ALL, "\nRobot setup complete.\n");
	delay(1000);

	Serial.println(F("\nChecking transmitter status."));
	radio.startListening();
	while (true) {
		while (!radio.available()) {}
		char read_buffer[33];
		memset(read_buffer, 0, 33);
		radio.read(read_buffer, 32);
		if (!strcmp(read_buffer, "\nRobot setup complete.\n")) break;
	}
	delay(100);
	Serial.println(F("Transmitter is ready."));

	Serial.println(F("\nStarting remote control."));
}

void loop(void) {
	radio.startListening();
	unsigned long const watchdog_timeout = millis() + 1000;
	while (!radio.available() && millis() < watchdog_timeout) {}
	if (!radio.available()) {
    left_motor.detach();
    right_motor.detach();
		Serial.println(F("\nWatchdog timer expired."));
		Serial.println(F("Resetting robot."));
		setup();
		return;
	}

	uint8_t payload[4];
	radio.read(payload, 4);
	int const lpwm = (int)payload[0] | ((int)payload[1] << 8);
	int const rpwm = (int)payload[2] | ((int)payload[3] << 8);
	left_motor.writeMicroseconds(lpwm);
	right_motor.writeMicroseconds(rpwm);

	// Serial.println();
	// Serial.println(F("Packet received."));
	// Serial.print(F("Payload: "));
	// Serial.print(payload[0], HEX);
	// Serial.print(payload[1], HEX);
	// Serial.print(payload[2], HEX);
	// Serial.println(payload[3], HEX);

	// Serial.println();
	// Serial.print(F("Left Motor PWM: "));
	// Serial.println(lpwm);
	// Serial.print(F("Right Motor PWM: "));
	// Serial.println(rpwm);
}

void NRFSendText(RF24 & radio, uint8_t target, char const * text) {
	char const * msg_start = text;

	while (strlen(msg_start) > 0) {
		uint8_t const msg_len = (strlen(msg_start) < 31 ? strlen(msg_start) : 31);

		char write_buffer[32];
		write_buffer[0] = target;
		memset(&write_buffer[1], 0, 31);
		memcpy(&write_buffer[1], msg_start, msg_len);

		radio.stopListening();
		while (!radio.write(write_buffer, 32)) delay(100);
		msg_start += msg_len * sizeof(char);
	}
}