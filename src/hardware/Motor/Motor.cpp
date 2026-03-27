#include "Arduino.h"
#include "Motor.h"

#define PWM_FREQ 490
#define PWM_RESOLUTION 8

Motor::Motor(unsigned int pinA, unsigned int pinB, unsigned int pinPwm, unsigned int maxLimitPwm, unsigned int pwmDeadZone, unsigned int pwmChannel)
    : pinA(pinA), pinB(pinB), pinPwm(pinPwm), maxLimitPwm(maxLimitPwm), pwmDeadZone(pwmDeadZone), pwmChannel(pwmChannel)
{
  
}
void Motor::begin()
{
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinPwm, OUTPUT);

    ledcSetup(pwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinPwm, pwmChannel);  
    forward();
}
void Motor::forward()
{
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
}
void Motor::backward()
{
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
}

void Motor::setValue(int val)
{
    val = val < pwmDeadZone ? 0 : constrain(val, pwmDeadZone, maxLimitPwm);
    ledcWrite(pwmChannel, val);
}
unsigned int Motor::getPinA()
{
    return pinA;
}
unsigned int Motor::getPinB()
{
    return pinB;
}
unsigned int Motor::getPinPwm()
{
    return pinPwm;
}
