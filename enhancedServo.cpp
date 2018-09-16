#include "Arduino.h"
#include "enhancedServo.h"
#include "Servo.h"

enhancedServo::enhancedServo(char _name, Servo _servo, int _feed_pin, int _feed_min, int _feed_max, int _write_pin, int _write_min, int _write_max, double _Ki_neg, double _Ki_pos, double _alpha, int _errorMargin) {
	this->name = _name;
	this->servo = _servo;
	this->feed_pin = _feed_pin;
	this->feed_min = _feed_min;
	this->feed_max = _feed_max;
	this->write_pin = _write_pin;
	this->write_min = _write_min;
	this->write_max = _write_max;
	this->Ki_neg = _Ki_neg;
	this->Ki_pos = _Ki_pos;
	this->alpha = _alpha;
	this->errorMargin = _errorMargin;
}

void enhancedServo::write(int microseconds) {
	servo.writeMicroseconds(microseconds);
}

void enhancedServo::attach() {
	servo.attach(write_pin);
}

void enhancedServo::detach() {
	servo.detach();
}

void enhancedServo::preInitializer() {
	integral = rangeMap(feed_min, feed_max, write_min, write_max, getFeedback());
	expMvingAvg = integral;
}

void enhancedServo::loopInitializer() {
	timesCorrect = 0;
	arrived = false;
}

void enhancedServo::computePath(int position) {
	if (timesCorrect < 4) {
		if (position - getFeedback() < 0) {
			integral += error * Ki_neg;
		} else {
			integral += error * Ki_pos;
		}
		expMvingAvg = alpha * constrain(integral, write_min, write_max) + (1 - alpha) * expMvingAvg;
		servo.writeMicroseconds((int) expMvingAvg);
		error = position - getFeedback();
		//Serial.print("Servo "); Serial.print(name); Serial.print(" error: "); Serial.print(error); Serial.print("\n");
		if (abs(error) <= errorMargin) {
			timesCorrect++;
		} else {
			timesCorrect = 0;
		}
	} else {
		servo.writeMicroseconds(expMvingAvg);
		arrived = true;
		Serial.print("Servo "); Serial.print(name); Serial.print(" arrived!"); Serial.print("\n");
	}
}

int enhancedServo::getOutput() {
	return (int) expMvingAvg;
}

bool enhancedServo::getArrived() {
	if (arrived) {
		Serial.print("Servo "); Serial.print(name); Serial.print(" arrived!"); Serial.print("\n");
	} else {
		Serial.print("Servo "); Serial.print(name); Serial.print(" not arrived!"); Serial.print("\n");
	}
	return arrived;
}

int enhancedServo::getFeedback() {
	return constrain(analogRead(feed_pin), feed_min, feed_max);
}

double enhancedServo::rangeMap(float a, float b, float c, float d, float input) {
	// Takes an input between a and b and linearly maps the output between c and d
	float x = 0;
	if (input > b) {
		x = b;
	} else if (input < a) {
		x = a;
	} else {
		x = input;
	}
	int value = (c * (1 - ((x - a) / (b - a)))) + (d * ((x - a) / (b - a)));
	return value;
}
