#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

class Joystick {
private:

	// Members of the Joystick class
	uint8_t xPinNumber, yPinNumber, btnPinNumber;
	uint16_t xOrigin, xMinimum, xMaximum;
	uint16_t yOrigin, yMinimum, yMaximum;
	bool xReversed, yReversed, btnReversed;

public:
	Joystick(uint8_t xPinNumber, uint8_t yPinNumber, uint8_t btnPinNumber, bool xReversed, bool yReversed, bool btnReversed);

	uint16_t getOriginX(void);
	uint16_t getOriginY(void);
	uint16_t getMinimumX(void);
	uint16_t getMinimumY(void);
	uint16_t getMaximumX(void);
	uint16_t getMaximumY(void);

	uint16_t readValueX(void);
	uint16_t readValueY(void);
	uint8_t readButton(void);

	void setOrigins(bool xAxisCal, bool yAxisCal, unsigned long readTime);
	void setExtrema(bool xAxisCal, bool yAxisCal, unsigned long readTime);
};

#endif