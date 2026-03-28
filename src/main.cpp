#include <Wire.h>
#include "hardware/Motor/Motor.h"
#include "hardware/SonarSensor/SonarSensor.h"
#include <Arduino.h>
#include <WiFi.h>

// ================== CONFIG MOTORES ==================
const unsigned int MOTOR_1_A = 10;
const unsigned int MOTOR_1_B = 11;

const unsigned int MOTOR_2_A = 12;
const unsigned int MOTOR_2_B = 13;

const unsigned int MOTOR_PWM_1 = 8;
const unsigned int MOTOR_PWM_2 = 9;

// ================== CONFIG SENSORES ==================
const unsigned int SENSOR_TrigPin_FL = 4;
const unsigned int SENSOR_EchoPin_FL = 5;

const unsigned int SENSOR_TrigPin_FR = 6;
const unsigned int SENSOR_EchoPin_FR = 7;

const unsigned int SENSOR_TrigPin_BL = 15;
const unsigned int SENSOR_EchoPin_BL = 16;

const unsigned int SENSOR_TrigPin_BR = 17;
const unsigned int SENSOR_EchoPin_BR = 18;

const unsigned int deadZonePwm = 120;
const unsigned int maxLimitPwm = 255;

// ================== CONFIG POTENCIÔMETROS ==================
const unsigned long TimeReleasePots = 1000;
const unsigned int AnalogMaxValue = 4095;

// 1 - POT
int distanceDetection = 50;
const unsigned int PotDistancePin = 1; 
const unsigned int minLimitDistance = 5;
const unsigned int maxLimitDistance = 80;
unsigned long potLeftlastMillis = 0;

// 2 - POT
int currentMaxLimitPwm = 0;
const unsigned int PotPwmPin = 2; 
unsigned long potRightlastMillis = 0;

// ==================== Motores ========================

Motor Motor_A(MOTOR_1_A, MOTOR_1_B, MOTOR_PWM_1, maxLimitPwm, deadZonePwm, 0);
Motor Motor_B(MOTOR_2_A, MOTOR_2_B, MOTOR_PWM_2, maxLimitPwm, deadZonePwm, 1);

// ==================== Sensores =========================
SonarSensor Front_Sensor_FL(SENSOR_TrigPin_FL, SENSOR_EchoPin_FL);
SonarSensor Front_Sensor_FR(SENSOR_TrigPin_FR, SENSOR_EchoPin_FR);

SonarSensor Front_Sensor_BL(SENSOR_TrigPin_BL, SENSOR_EchoPin_BL);
SonarSensor Front_Sensor_BR(SENSOR_TrigPin_BR, SENSOR_EchoPin_BR);

// ====================== Variaveis Local =====================

volatile bool isBackward = false;
volatile bool isForward = false;
volatile int currentPwm = 0;

TaskHandle_t taskSensors;
TaskHandle_t taskBackward;
TaskHandle_t taskCommands;
TaskHandle_t taskPots;

enum Commands
{
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

volatile Commands currentCommand = STOP;

void taskSensorsLoop(void *pvParameters)
{
  while (true)
  {
    int distanceFL = Front_Sensor_FL.read();
    int distanceFR = Front_Sensor_FR.read();

    int motorDistanceSensor = min(distanceFL, distanceFR);

    float speedNorm = (float)currentMaxLimitPwm / maxLimitPwm;
    int dynamicMinDistance = 10 + (speedNorm * speedNorm) * 30;

    int dynamicDistance = map(currentMaxLimitPwm, deadZonePwm, maxLimitPwm, minLimitDistance, distanceDetection);

    float distNorm = (float)motorDistanceSensor / dynamicDistance;
    distNorm = constrain(distNorm, 0.0, 1.0);

    float curva = distNorm * distNorm * distNorm;

    int minPwm = deadZonePwm;
    int vlPwm = minPwm + curva * (currentMaxLimitPwm - minPwm);

    // decisão global (variáveis compartilhadas)
    if (motorDistanceSensor < dynamicMinDistance)
    {
      if (motorDistanceSensor < minLimitDistance)
      {
        isBackward = true;
      }

      if (distanceFR < distanceFL)
        currentCommand = LEFT;
      else
        currentCommand = RIGHT;
    }
    else
    {
      isBackward = false;
      currentPwm = vlPwm;
      currentCommand = FORWARD;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS); // 50Hz
  }
}

void taskBackwardLoop(void *pvParameters)
{
  while (true)
  {
    if (isBackward)
    {
      int distanceBL = Front_Sensor_BL.read();
      int distanceBR = Front_Sensor_BR.read();

      int motorDistanceSensor = min(distanceBL, distanceBR);

      currentCommand = BACKWARD;

      if (motorDistanceSensor < 20)
      {
        if (distanceBR < distanceBL)
          currentCommand = RIGHT;
        else
          currentCommand = LEFT;
      }
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

void taskPotsLoop(void *pvParameters)
{
  while (true)
  {
    int potLeft = analogRead(PotDistancePin);
    distanceDetection = map(potLeft, 0, AnalogMaxValue, minLimitDistance, maxLimitDistance);

    int potRight = analogRead(PotPwmPin);
    currentMaxLimitPwm = map(potRight, 0, AnalogMaxValue, deadZonePwm, maxLimitPwm);

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void taskCommandsLoop(void *pvParameters)
{
  while (true)
  {
    switch (currentCommand)
    {
    case LEFT:
      Motor_A.setValue(deadZonePwm);
      Motor_B.setValue(0);
      break;

    case RIGHT:
      Motor_A.setValue(0);
      Motor_B.setValue(deadZonePwm);
      break;

    case FORWARD:
      Motor_A.forward();
      Motor_B.forward();
      Motor_A.setValue(currentPwm);
      Motor_B.setValue(currentPwm);
      break;

    case BACKWARD:
      Motor_A.backward();
      Motor_B.backward();
      Motor_A.setValue(deadZonePwm);
      Motor_B.setValue(deadZonePwm);
      break;
    default:
      Motor_A.setValue(0);
      Motor_B.setValue(0);
      break;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  Front_Sensor_FL.begin();
  Front_Sensor_FR.begin();
  Front_Sensor_BL.begin();
  Front_Sensor_BR.begin();

  Motor_A.begin();
  Motor_B.begin();

  pinMode(PotDistancePin, INPUT);
  pinMode(PotPwmPin, INPUT);

  xTaskCreatePinnedToCore(taskSensorsLoop, "Sensors", 4096, NULL, 2, &taskSensors, 0);

  xTaskCreatePinnedToCore(taskBackwardLoop, "Backward", 4096, NULL, 1, &taskBackward, 1);

  xTaskCreatePinnedToCore(taskCommandsLoop, "Commands", 4096, NULL, 1, &taskCommands, 1);

  xTaskCreatePinnedToCore(taskPotsLoop, "Pots", 2048, NULL, 1, &taskPots, 1);
}
void loop()
{
}