#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(2, 15); // CE, CSN pins
const byte address[6] = "00001";

struct JoystickData {
  int x;
  int y;
  bool button;
};

const int retryLimit = 10; // Number of retries before reinitializing
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5000; // Interval to check for data (in milliseconds)

void setup() {
  Serial.begin(9600); // Initialize serial communication

  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    JoystickData data;
    radio.read(&data, sizeof(data));

    // Print received data to the Serial Monitor
    Serial.print("X: ");
    Serial.print(data.x);
    Serial.print("\t");

    Serial.print("Y: ");
    Serial.print(data.y);
    Serial.print("\t");

    Serial.print("Btn: ");
    Serial.println(data.button ? "Pressed" : "Released");

    // Reset the check timer since data was received
    lastCheckTime = millis();
  } else {
    // Check if it's time to reinitialize the radio
    if (millis() - lastCheckTime > checkInterval) {
      Serial.println("No data received. Reinitializing...");
      radio.stopListening();
      delay(100); // Short delay to allow module to settle
      radio.begin();
      radio.openReadingPipe(1, address);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
      lastCheckTime = millis(); // Reset timer after reinitializing
    }
  }
}
