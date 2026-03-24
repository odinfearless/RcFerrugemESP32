#include <Wire.h>
#include "hardware/Motor/Motor.h"
#include "hardware/SonarSensor/SonarSensor.h"
#include <Arduino.h>
#include <WiFi.h>

// ================== MOTORES ==================
const unsigned int MOTOR_1_A = 4;
const unsigned int MOTOR_1_B = 5;

const unsigned int MOTOR_2_A = 6;
const unsigned int MOTOR_2_B = 7;

const unsigned int MOTOR_PWM_1 = 8;
const unsigned int MOTOR_PWM_2 = 9;

// ================== SENSORES ==================
const unsigned int SENSOR_TrigPin_FL = 10;
const unsigned int SENSOR_EchoPin_FL = 1;

const unsigned int SENSOR_TrigPin_FR = 12;
const unsigned int SENSOR_EchoPin_FR = 13;

const unsigned int SENSOR_TrigPin_BL = 14;
const unsigned int SENSOR_EchoPin_BL = 15;

const unsigned int SENSOR_TrigPin_BR = 16;
const unsigned int SENSOR_EchoPin_BR = 17;

const unsigned int deadZonePwm = 90;
const unsigned int maxLimitPwm = 255;

// ================== POTENCIÔMETROS ==================
const unsigned long TimeReleasePots = 1000;
const unsigned int AnalogMaxValue = 1023;

int distanceDetection = 50;
const unsigned int PotDistancePin = 1; // ADC estável
const unsigned int minLimitDistance = 5;
const unsigned int maxLimitDistance = 80;
unsigned long potLeftlastMillis = 0;

int currentMaxLimitPwm = 0;
const unsigned int PotPwmPin = 2; // ADC estável
unsigned long potRightlastMillis = 0;

Motor Motor_A(MOTOR_1_A, MOTOR_1_B, MOTOR_PWM_1, maxLimitPwm, deadZonePwm);
Motor Motor_B(MOTOR_2_A, MOTOR_2_B, MOTOR_PWM_2, maxLimitPwm, deadZonePwm);

SonarSensor Front_Sensor_FL(SENSOR_TrigPin_FL, SENSOR_EchoPin_FL);
SonarSensor Front_Sensor_FR(SENSOR_TrigPin_FR, SENSOR_EchoPin_FR);

SonarSensor Front_Sensor_BL(SENSOR_TrigPin_BL, SENSOR_EchoPin_BL);
SonarSensor Front_Sensor_BR(SENSOR_TrigPin_BR, SENSOR_EchoPin_BR);

bool isTurn = false;
bool isBackward = false;
bool isForward = false;

void setup()
{
  Serial0.begin(115200);
  WiFi.disconnect(true); // desconecta e limpa config
  WiFi.mode(WIFI_OFF);   // desliga rádio Wi-Fi
  btStop();

  Front_Sensor_FL.begin();
  Front_Sensor_FR.begin();

  Front_Sensor_BL.begin();
  Front_Sensor_BR.begin();

  Motor_A.begin();
  Motor_B.begin();

  pinMode(PotDistancePin, INPUT);
  pinMode(PotPwmPin, INPUT);
}

void readPots()
{
  unsigned long currentMillis = millis();

  if (currentMillis - potLeftlastMillis >= TimeReleasePots)
  {
    potLeftlastMillis = currentMillis;
    int potLeft = analogRead(PotDistancePin);
    Serial0.println(potLeft);
    distanceDetection = map(potLeft, 0, 1000, minLimitDistance, maxLimitDistance);
  }

  if (currentMillis - potRightlastMillis >= TimeReleasePots)
  {
    potRightlastMillis = currentMillis;
    int potRight = analogRead(PotPwmPin);
    currentMaxLimitPwm = map(potRight, 0, 1000, deadZonePwm, maxLimitPwm);
  }
}

void turnLeft()
{
  Motor_A.setValue(deadZonePwm);
  Motor_B.setValue(0);
}

void turnRight()
{
  Motor_A.setValue(0);
  Motor_B.setValue(deadZonePwm);
}
void forward()
{
  int distanceFL = Front_Sensor_FL.read();
  int distanceFR = Front_Sensor_FR.read();

  int motorDistanceSensor = min(distanceFL, distanceFR);

  float speedNorm = (float)currentMaxLimitPwm / maxLimitPwm;
  int dynamicMinDistance = 10 + (speedNorm * speedNorm) * 30;
  // distância dinâmica baseada na velocidade
  int dynamicDistance = map(currentMaxLimitPwm, deadZonePwm, maxLimitPwm, minLimitDistance, distanceDetection);

  // normaliza (0 a 1)
  float distNorm = (float)motorDistanceSensor / dynamicDistance;
  distNorm = constrain(distNorm, 0.0, 1.0);

  // curva (quanto maior o expoente, mais suave no começo e mais forte no final)
  float curva = distNorm * distNorm * distNorm;

  // PWM mínimo pra não travar
  int minPwm = deadZonePwm;

  // cálculo final
  int vlPwm = minPwm + curva * (currentMaxLimitPwm - minPwm);

  // parada seca bem perto

  if (motorDistanceSensor < dynamicMinDistance)
  {
    if (motorDistanceSensor < minLimitDistance)
    {
      isBackward = true;
    }
    if (!isTurn)
    {
      isTurn = true;

      if (distanceFR < distanceFL)
      {
        turnLeft();
      }
      else
      {
        turnRight();
      }
    }
  }
  else
  {
    isTurn = false;
    isBackward = false;
    Motor_A.forward();
    Motor_B.forward();
    Motor_A.setValue(vlPwm);
    Motor_B.setValue(vlPwm);
  }
}

void backward()
{
  if (isForward)
  {
    isBackward = false;
  }

  if (isBackward)
  {
    int distanceBL = Front_Sensor_BL.read();
    int distanceBR = Front_Sensor_BR.read();
    int motorDistanceSensor = min(distanceBL, distanceBR);

    Motor_A.backward();
    Motor_B.backward();

    if (motorDistanceSensor < 20)
    {

      if (!isTurn)
      {
        isTurn = true;
        if (distanceBR < distanceBL)
        {
          turnRight();
        }
        else
        {
          turnLeft();
        }
      }
    }
    else
    {
      isTurn = false;
      Motor_A.setValue(deadZonePwm);
      Motor_B.setValue(deadZonePwm);
    }
  }
}

void driver()
{
  forward();
  backward();
}

void loop()
{
  readPots();
  driver();
}
