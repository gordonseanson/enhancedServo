#ifndef enhancedServo_h
#define enhancedServo_h

#include "Arduino.h"
#include "Servo.h"
#include <String.h>

#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

class enhancedServo
{
  public:
    enhancedServo(
        Adafruit_PWMServoDriver _pwm, 
        String _name, 
        uint8_t _feed_pin, 
        uint8_t _pot_pin,
        uint8_t _pot_reference_pin, 
        uint8_t _write_pin, 
        int _write_min, 
        int _write_max,
        int _feed_min,
        int _feed_max,
        double _Ki, 
        double _alpha, 
        double _errorMargin,
        bool _gripper);

    bool gripper;
    String name;
    Adafruit_PWMServoDriver pwm;
    uint8_t feed_pin;
    uint8_t pot_pin;
    uint8_t pot_reference_pin;
    int feed_min;
    int feed_max;
    uint8_t write_pin;
    int write_min;
    int write_max;
    double Ki;
    double alpha;
    double errorMargin;
    void write(double microseconds);
    void detach();
    void preInitializer();
    void setupInitializer();
    void loopInitializer();
    void computePath(double position);
    bool getArrived();
    int getFeedback();
    double rangeMap(float a, float b, float c, float d, float input);

    double feedback[4];
    double error;
	double timesCorrect;
	bool arrived;
	double integral;
	double integralAvg;

    double pot_pin_V;
    double pot_reference_V;
    double feedback_V;

    double pot_pin_V_avg;
    double pot_reference_V_avg;
    double feedback_V_avg;

    bool DEBUG;
};

#endif
