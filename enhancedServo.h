#ifndef enhancedServo_h
#define enhancedServo_h

#include "Arduino.h"
#include "Servo.h"

class enhancedServo
{
  public:
    enhancedServo(char _name, Servo _servo, int _feed_pin, int _feed_min, int _feed_max, int _write_pin, int _write_min, int _write_max, double _Ki_neg, double _Ki_pos, double _alpha, int _errorMargin);
    char name;
    Servo servo;
    int feed_pin;
    int feed_min;
    int feed_max;
    int write_pin;
    int write_min;
    int write_max;
    double Ki_neg;
    double Ki_pos;
    double alpha;
    int errorMargin;
    void write(int microseconds);
    void attach();
    void detach();
    void preInitializer();
    void loopInitializer();
    void computePath(int position);
    int getOutput();
    bool getArrived();
    int getFeedback();
    double rangeMap(float a, float b, float c, float d, float input);

    int feedback[4];
    int error;
	int timesCorrect;
	bool arrived;
	double integral;
	double expMvingAvg;
};

#endif
