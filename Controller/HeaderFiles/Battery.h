#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

class Battery {
private:
	uint8_t pinNumber;
	uint16_t minVoltage, maxVoltage;
	uint8_t divFactor;
	uint16_t minAnalog, maxAnalog;

public:
	Battery(uint8_t readPinNumber, uint16_t minVoltage, uint16_t maxVoltage, uint8_t divFactor);

	uint8_t getPinNumber(void);
	uint16_t getMinVoltage(void);
	uint16_t getMaxVoltage(void);

	uint16_t readVoltage(void);
	uint16_t readPercent(void);
};

#endif