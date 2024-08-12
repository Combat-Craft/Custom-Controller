#include <Arduino.h>
#include "HeaderFiles/Joystick.h"

Joystick::Joystick(uint8_t xPinNumber, uint8_t yPinNumber, uint8_t btnPinNumber, bool xReversed, bool yReversed, bool btnReversed)
: xPinNumber(xPinNumber), yPinNumber(yPinNumber), btnPinNumber(btnPinNumber), xReversed(xReversed), yReversed(yReversed), btnReversed(btnReversed) {
	xMinimum = 0;
  xMaximum = 4095;
  yMinimum = 0;
  yMaximum = 4095;

  pinMode(xPinNumber, INPUT);
  pinMode(yPinNumber, INPUT);
  pinMode(btnPinNumber, INPUT);
}

uint16_t Joystick::getOriginX(void)  {return xOrigin;}
uint16_t Joystick::getOriginY(void)  {return yOrigin;}
uint16_t Joystick::getMinimumX(void) {return xMinimum;}
uint16_t Joystick::getMinimumY(void) {return yMinimum;}
uint16_t Joystick::getMaximumX(void) {return xMaximum;}
uint16_t Joystick::getMaximumY(void) {return yMaximum;}

uint16_t Joystick::readValueX(void) {
	pinMode(xPinNumber, INPUT);
	uint16_t const xaxis_test = analogRead(xPinNumber);
	if (xReversed && xaxis_test > xOrigin)
		return map(xaxis_test, xOrigin, xMaximum, xOrigin, xMinimum);
	if (xReversed && xaxis_test < xOrigin)
		return map(xaxis_test, xOrigin, xMinimum, xOrigin, xMaximum);
	return xaxis_test;
}

uint16_t Joystick::readValueY(void) {
	pinMode(yPinNumber, INPUT);
	uint16_t const yaxis_test = analogRead(yPinNumber);
	if (yReversed && yaxis_test > yOrigin)
		return map(yaxis_test, yOrigin, yMaximum, yOrigin, yMinimum);
	if (yReversed && yaxis_test < yOrigin)
		return map(yaxis_test, yOrigin, yMinimum, yOrigin, yMaximum);
	return yaxis_test;
}

uint8_t Joystick::readButton(void) {
	if (btnPinNumber == 0) return LOW;
	pinMode(btnPinNumber, INPUT);
	uint8_t const btn_test = digitalRead(btnPinNumber);
	if (btnReversed && (btn_test == HIGH || btn_test == LOW))
		return (btn_test == HIGH ? LOW : HIGH);
	return btn_test;
}

void Joystick::setOrigins(bool xAxisCal, bool yAxisCal, unsigned long readTime) {
	uintmax_t xaxis_accum = 0, yaxis_accum = 0, num_reads = 0;

	pinMode(xPinNumber, INPUT);
	pinMode(yPinNumber, INPUT);

	unsigned long const timeout = millis() + readTime;
	while (millis() < timeout) {
		uint16_t const xaxis_test = analogRead(xPinNumber);
		bool const xaxis_ovf = xaxis_accum + xaxis_test < xaxis_accum;
		if (!xaxis_ovf) xaxis_accum += xaxis_test;

		uint16_t const yaxis_test = analogRead(yPinNumber);
		bool const yaxis_ovf = yaxis_accum + yaxis_test < yaxis_accum;
		if (!yaxis_ovf) yaxis_accum += yaxis_test;

		if (xaxis_ovf && yaxis_ovf) break;
		if (++num_reads == UINTMAX_MAX) break;
	}

	if (xAxisCal) xOrigin = round(xaxis_accum / (double)num_reads);
	if (yAxisCal) yOrigin = round(yaxis_accum / (double)num_reads);
}

void Joystick::setExtrema(bool xAxisCal, bool yAxisCal, unsigned long readTime) {
	uint16_t xaxis_min, xaxis_max, yaxis_min, yaxis_max;
	xaxis_min = xaxis_max = analogRead(xPinNumber);
	yaxis_min = yaxis_max = analogRead(yPinNumber);

	pinMode(xPinNumber, INPUT);
	pinMode(yPinNumber, INPUT);

	unsigned long const timeout = millis() + readTime;
	while (millis() < timeout) {
		uint16_t const xaxis_test = analogRead(xPinNumber);
		if (xaxis_test < xaxis_min) xaxis_min = xaxis_test;
		if (xaxis_test > xaxis_max) xaxis_max = xaxis_test;

		uint16_t const yaxis_test = analogRead(yPinNumber);
		if (yaxis_test < yaxis_min) yaxis_min = yaxis_test;
		if (yaxis_test > yaxis_max) yaxis_max = yaxis_test;
	}

	if (xAxisCal) {xMinimum = xaxis_min; xMaximum = xaxis_max;}
	if (yAxisCal) {yMinimum = yaxis_min; yMaximum = yaxis_max;}
}