#include "Arduino.h"
#include "Motor.h"

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
static int nextChannel = 0; // garante canais únicos
int channel = 0;

Motor::Motor(unsigned int pinA, unsigned int pinB, unsigned int pinPwm, unsigned int maxLimitPwm, unsigned int pwmDeadZone)
    : pinA(pinA), pinB(pinB), pinPwm(pinPwm), maxLimitPwm(maxLimitPwm), pwmDeadZone(pwmDeadZone)
{
    channel = nextChannel++; // cada motor pega um canal diferente
}
void Motor::begin()
{
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinPwm, OUTPUT);

    ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinPwm, channel);

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
    ledcWrite(channel, val);
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
