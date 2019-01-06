#include "Arduino.h"
#include "enhancedServo.h"
#include "Servo.h"
#include <String.h>

#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

enhancedServo::enhancedServo(
	Adafruit_PWMServoDriver _pwm, 
	String _name, 
	uint8_t _feed_pin,	// middle pin of potentiometer
	uint8_t _pot_pin,	// top pin of potentiometer
	uint8_t _pot_reference_pin,	// bottom pin of potentiometer
	uint8_t _write_pin,	// write pin of servo
	int _write_min,
	int _write_max, 
	int _feed_min,
	int _feed_max,
	double _Ki,	// constant for tuning integral control loop
	double _alpha,	// contant for tuning control loop EWMA
	double _errorMargin,	// constant for tuning the accuracy of the servo motion
	bool _gripper) {

	this->gripper = _gripper;
	this->name = _name;
	this->pwm = _pwm;
	this->feed_pin = _feed_pin;
	this->pot_pin = _pot_pin;
	this->pot_reference_pin = _pot_reference_pin;
	this->write_pin = _write_pin;
	this->write_min = _write_min;
	this->write_max = _write_max;
	this->feed_min = _feed_min;
	this->feed_max = _feed_max;
	this->Ki = _Ki;
	this->alpha = _alpha;
	this->errorMargin = _errorMargin;
}

// Tell servo to move
void enhancedServo::write(double microseconds) {
	pwm.setPWM(write_pin, 0, microseconds);
}

// Setting these PWM values lets you move the servo freely by hand
void enhancedServo::detach() {
	pwm.setPWM(write_pin, 0, 4096);
}

// Returns arrival status (for use when the arm is moving through position sets)
bool enhancedServo::getArrived() {
	return arrived;
}

// Call this function before doing any arm movement to correctly initialize running averages for feedback
void enhancedServo::setupInitializer() {
pot_pin_V_avg = -1;
pot_reference_V_avg = -1;
feedback_V_avg = -1;
DEBUG = false;
}

// Call this function before doing any arm movement to correctly initialize the integral control loop
void enhancedServo::preInitializer() {
	integral = rangeMap(feed_min, feed_max, write_min, write_max, getFeedback());
	integralAvg = integral;
}

// Initializes the arrival conditions as false before each move
void enhancedServo::loopInitializer() {
	timesCorrect = 0;
	arrived = false;
}

// Initializing some variables for error EWMA
double errorAlpha = 0.75;
double errorAvg;

// Initializing some variables for feedback EWMA
double fAlpha = .2;	// change this value to adjust feedback EWMA
double fBeta = 1 - fAlpha;

// Takes relative feedback measurement and keeps running average of feedback
int enhancedServo::getFeedback() {
  // Converting analog input values to Voltage units
  pot_pin_V = (double) analogRead(pot_pin) / 1023 * 5;
  pot_reference_V = (double) analogRead(pot_reference_pin) / 1023 * 5;
  feedback_V = (double) analogRead(feed_pin) / 1023 * 5;
  
  // Setting up seperate variables for running voltage averages
  // Also sets initial values before running averages start
  if (pot_pin_V_avg == -1) pot_pin_V_avg = pot_pin_V;
  if (feedback_V_avg == -1) feedback_V_avg = feedback_V;
  if (pot_reference_V_avg == -1) pot_reference_V_avg = pot_reference_V;

  // Running averages (EWMA)
  pot_pin_V_avg = fAlpha * pot_pin_V + fBeta * pot_pin_V_avg;
  feedback_V_avg = fAlpha * feedback_V + fBeta * feedback_V_avg;
  pot_reference_V_avg = fAlpha * pot_reference_V + fBeta * pot_reference_V_avg;

  // DEBUG PRINT STATEMENTS
  if (DEBUG) {
	  Serial.print("alpha: "); Serial.print(fAlpha); Serial.print(" ");
	  Serial.print("beta: "); Serial.print(fBeta); Serial.print(" ");
	  Serial.print("  ||  ");
	  Serial.print("top V: "); Serial.print(pot_pin_V); Serial.print(" "); 
	  Serial.print("top avg V: "); Serial.print(pot_pin_V_avg); Serial.print(" ");
	  Serial.print("  ||  ");
	  Serial.print("middle V: "); Serial.print(feedback_V); Serial.print(" "); 
	  Serial.print("middle avg V: "); Serial.print(feedback_V_avg); Serial.print(" ");
	  Serial.print("  ||  ");
	  Serial.print("bottom V: "); Serial.print(pot_reference_V); Serial.print(" "); 
	  Serial.print("bottom avg V: "); Serial.print(pot_reference_V_avg); Serial.print(" ");
	  Serial.print("  ||  ");
	  Serial.print("feedback: ");
  }

  // I ran out of analog input pins, so I had to make this exception for the gripper's servo feedback
  if (gripper) {
  	return constrain(analogRead(A15), feed_min, feed_max);
  }

  // The feedback is equal to the following expression, using measurements from the potentiometer
  // (middle pin - bottom pin) / (top pin - bottom pin)
  return (int) constrain(((feedback_V_avg - pot_reference_V_avg) / 
  	(pot_pin_V_avg - pot_reference_V_avg) * 1000), feed_min, feed_max);
}

// Computes pwm to send to the servo; intended to run in a loop
void enhancedServo::computePath(double position) {
	// If the servo has not reported being at the correct positon for a certain number of cycles, continues computing output
	if (timesCorrect < 3) {
		// Integral control loop
		integral += error * Ki;

		// Running average for integral
		integralAvg = alpha * constrain(integral, write_min, write_max) + (1-alpha) * integralAvg;
		write((int) integralAvg);
		
		// Accuracy managment
		error = (position - getFeedback());
		errorAvg = errorAlpha * error + (1-errorAlpha) * errorAvg;
		if (abs(errorAvg) <= errorMargin) {
			timesCorrect++;
		} else {
			timesCorrect = 0;
		}
	} else {
		// After movement complete, holds position
		write(integralAvg);
		arrived = true;
	}
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
	double value = (c * (1 - ((x - a) / (b - a)))) + (d * ((x - a) / (b - a)));
	return value;
}
