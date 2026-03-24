#include "Arduino.h"
#include "SonarSensor.h"

SonarSensor::SonarSensor(unsigned int trigPin, unsigned int echoPin) : trigPin(trigPin),
                                                                       echoPin(echoPin)                                                                     
{
}
void SonarSensor::begin()
{
    pinMode(trigPin, OUTPUT);               // Define o Trigger como Saída
    pinMode(echoPin, INPUT);                // Define o Echo como Entrada  
}

int SonarSensor::read()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);

    if (duration == 0) return 999;

    return duration * 0.034 / 2;
}
