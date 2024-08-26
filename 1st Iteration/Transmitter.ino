#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

RF24 radio(2, 15); // CE, CSN pins
const byte address[6] = "00001";

const int joystick_x_pin = A0;
const int joystick_y_pin = A3;
const int swPin = 25;

struct JoystickData {
  int x;
  int y;
  bool button;
};

const unsigned long checkInterval = 5000; // Time to check for successful transmission (in milliseconds)
unsigned long lastTransmitTime = 0;
bool transmissionSuccessful = false;

void setup() {
  pinMode(swPin, INPUT_PULLUP); // Configure the switch pin with internal pull-up resistor

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  lcd.begin(20,4);
  lcd.init();
  lcd.backlight();
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Display "98%" on the top right of the LCD
  lcd.setCursor(17, 0);
  lcd.print("98%");

  JoystickData data;
  data.x = analogRead(joystick_x_pin);
  data.y = analogRead(joystick_y_pin);
  data.button = !digitalRead(swPin); // Assuming LOW when pressed, HIGH when released

  // Try to send data and check if transmission is successful
  transmissionSuccessful = radio.write(&data, sizeof(data));

  if (transmissionSuccessful) {
    lastTransmitTime = millis(); // Update last transmission time
    lcd.setCursor(0, 0);
    lcd.print("X: ");
    lcd.print(data.x);
    lcd.print("   "); // Clear remaining characters

    lcd.setCursor(0, 1);
    lcd.print("Y: ");
    lcd.print(data.y);
    lcd.print("   "); // Clear remaining characters

    lcd.setCursor(0, 2);
    lcd.print("Btn: ");
    lcd.print(data.button ? "Pressed  " : "Released "); // Add spaces to clear remaining characters

    lcd.setCursor(0, 3);
    lcd.print("Status: OK     "); // Clear remaining characters
  } else {
    // Handle unsuccessful transmission
    lcd.setCursor(0, 3);
    lcd.print("Status: Fail    "); // Clear remaining characters
  }

  // Check if the time since the last successful transmission is greater than the interval
  if (millis() - lastTransmitTime > checkInterval) {
    lcd.setCursor(0, 3);
    lcd.print("Reinitializing...");
    
    radio.stopListening();
    delay(100); // Short delay to allow module to settle
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    lastTransmitTime = millis(); // Reset the timer after reinitializing
    
    // Clear the status message after reinitialization
    lcd.setCursor(0, 3);
    lcd.print("Status: OK     ");
  }

  delay(100); // Adjust delay as needed
}
