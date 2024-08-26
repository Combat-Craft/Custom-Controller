#include <Arduino.h>
#include "HeaderFiles/Battery.h"

Battery::Battery(uint8_t pinNumber, uint16_t minVoltage, uint16_t maxVoltage, uint8_t divFactor)
: pinNumber(pinNumber), minVoltage(minVoltage), maxVoltage(maxVoltage), divFactor(divFactor) {
	minAnalog = minVoltage * 1024 / 3300 / divFactor;
	maxAnalog = maxVoltage * 1024 / 3300 / divFactor;
}

uint8_t Battery::getPinNumber(void) {return pinNumber;}
uint16_t Battery::getMinVoltage(void) {return minVoltage;}
uint16_t Battery::getMaxVoltage(void) {return maxVoltage;}

uint16_t Battery::readVoltage(void) {
	long const analog_value = constrain(analogRead(pinNumber), minAnalog, maxAnalog);
	return (uint16_t)map(analog_value, minAnalog, maxAnalog, minVoltage, maxVoltage);
}

uint16_t Battery::readPercent(void) {
	long const analog_value = constrain(analogRead(pinNumber), minAnalog, maxAnalog);
	return (uint16_t)map(analog_value, minAnalog, maxAnalog, 0, 100);
}