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
const unsigned int SENSOR_TrigPin_FL = 35;
const unsigned int SENSOR_EchoPin_FL = 36;

const unsigned int SENSOR_TrigPin_FR = 6;
const unsigned int SENSOR_EchoPin_FR = 7;

const unsigned int SENSOR_TrigPin_BL = 15;
const unsigned int SENSOR_EchoPin_BL = 16;

const unsigned int SENSOR_TrigPin_BR = 17;
const unsigned int SENSOR_EchoPin_BR = 18;

const unsigned int SENSOR_TrigPin_SL = 35;
const unsigned int SENSOR_EchoPin_SL = 36;

const unsigned int SENSOR_TrigPin_SR = 37;
const unsigned int SENSOR_EchoPin_SR = 38;

const unsigned int deadZonePwm = 120;
const unsigned int maxLimitPwm = 220;

// ================== CONFIG POTs  ==================
const unsigned int PotDistancePin = 3;
const unsigned int PotPwmPin = 5;

// ================== CONFIG  ==================
const unsigned int releaseTimePots = 1000;
const unsigned int AnalogMaxValue = 4095;
const unsigned int minLimitDistance = 3;
const unsigned int maxLimitDistance = 40;

//==================== Starter ==================
const unsigned int Starter_Pin = 1;

// ==================== Motores ========================
Motor Motor_A(MOTOR_1_A, MOTOR_1_B, MOTOR_PWM_1, maxLimitPwm, deadZonePwm, 0);
Motor Motor_B(MOTOR_2_A, MOTOR_2_B, MOTOR_PWM_2, maxLimitPwm, deadZonePwm, 1);

// ==================== Sensores =========================
SonarSensor Front_Sensor_FL(SENSOR_TrigPin_FL, SENSOR_EchoPin_FL);
SonarSensor Front_Sensor_FR(SENSOR_TrigPin_FR, SENSOR_EchoPin_FR);

SonarSensor Front_Sensor_BL(SENSOR_TrigPin_BL, SENSOR_EchoPin_BL);
SonarSensor Front_Sensor_BR(SENSOR_TrigPin_BR, SENSOR_EchoPin_BR);

SonarSensor Side_Sensor_L(SENSOR_TrigPin_SL, SENSOR_EchoPin_SL);
SonarSensor Side_Sensor_R(SENSOR_TrigPin_SR, SENSOR_EchoPin_SR);

// ====================== Variáveis =====================
volatile bool isTurn = false;
volatile bool isBackward = false;
volatile float currentPwm = 0;
volatile unsigned long potsLastMillis = 0;
volatile int distanceDetection = maxLimitDistance;
volatile int currentMaxLimitPwm = maxLimitPwm;
volatile bool tasksRunning = false;
volatile bool lastState = HIGH;
static unsigned long lastTrigger = 0;

TaskHandle_t taskForward;
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

float lerp(float a, float b, float t)
{
  return a + t * (b - a);
}

void taskStarterSensorLoop(void *pvParameters)
{
  while (true)
  {
    bool currentState = digitalRead(Starter_Pin);

    if (lastState == HIGH && currentState == LOW &&
        millis() - lastTrigger > 300)
    {
      lastTrigger = millis();

      tasksRunning = !tasksRunning;

      if (tasksRunning)
      {
        Serial.println("Tasks START");
      }
      else
      {

        Serial.println("Tasks STOP");
      }
    }

    lastState = currentState;
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}
// ===================== SENSORES =======================
void taskForwardLoop(void *pvParameters)
{
  while (true)
  {
    if (!tasksRunning)
    {
      Motor_A.setValue(0);
      Motor_B.setValue(0);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    int distanceFL = Front_Sensor_FL.read();
    int distanceFR = Front_Sensor_FR.read();

    int motorDistanceSensor = min(distanceFL, distanceFR);

    if (motorDistanceSensor < distanceDetection)
    {
      if (motorDistanceSensor < minLimitDistance)
      {
        isBackward = true;
      }

      if (!isTurn)
      {
        isTurn = true;
        if (distanceFR < distanceFL)
          currentCommand = LEFT;
        else
          currentCommand = RIGHT;
      }
    }
    else
    {
      isTurn = false;
      isBackward = false;
      currentCommand = FORWARD;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ===================== BACKWARD =======================
void taskBackwardLoop(void *pvParameters)
{
  while (true)
  {
    if (!tasksRunning)
    {
      Motor_A.setValue(0);
      Motor_B.setValue(0);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (isBackward)
    {
      int distanceBL = Front_Sensor_BL.read();
      int distanceBR = Front_Sensor_BR.read();

      int motorDistanceSensor = min(distanceBL, distanceBR);

      if (motorDistanceSensor <= minLimitDistance)
      {
        currentCommand = FORWARD;
      }
      else
      {
        currentCommand = BACKWARD;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ===================== POTENCIÔMETROS =======================
void taskPotsLoop(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 100ms

  while (true)
  {
    if (!tasksRunning)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    int potLeft = analogRead(PotDistancePin);
    distanceDetection = map(potLeft, 0, AnalogMaxValue,
                            minLimitDistance, maxLimitDistance);

    int potRight = analogRead(PotPwmPin);
    currentMaxLimitPwm = map(potRight, 0, AnalogMaxValue,
                             deadZonePwm, maxLimitPwm);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ===================== COMANDOS =======================
void taskCommandsLoop(void *pvParameters)
{
  while (true)
  {
    if (!tasksRunning)
    {
      Motor_A.setValue(0);
      Motor_B.setValue(0);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    switch (currentCommand)
    {
    case LEFT:
      Motor_A.setValue(0);
      Motor_B.setValue(deadZonePwm);
      break;

    case RIGHT:
      Motor_A.setValue(deadZonePwm);
      Motor_B.setValue(0);
      break;

    case FORWARD:
      Motor_A.forward();
      Motor_B.forward();
      Motor_A.setValue(currentMaxLimitPwm);
      Motor_B.setValue(currentMaxLimitPwm);
      break;

    case BACKWARD:
      Motor_A.backward();
      Motor_B.backward();
      Motor_A.setValue(deadZonePwm);
      Motor_B.setValue(deadZonePwm);
      break;

    case STOP:
      Motor_A.setValue(0);
      Motor_B.setValue(0);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===================== SETUP =======================
void setup()
{
  Serial.begin(115200);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  /*
  pinMode(Starter_Pin, INPUT_PULLUP);

  Front_Sensor_FL.begin();
  Front_Sensor_FR.begin();
  Front_Sensor_BL.begin();
  Front_Sensor_BR.begin();
  Motor_A.begin();
  Motor_B.begin();

  xTaskCreatePinnedToCore(taskStarterSensorLoop, "StarterSensor", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskForwardLoop, "Forward", 4096, NULL, 2, &taskForward, 0);
  xTaskCreatePinnedToCore(taskBackwardLoop, "Backward", 4096, NULL, 1, &taskBackward, 1);
  xTaskCreatePinnedToCore(taskCommandsLoop, "Commands", 2048, NULL, 1, &taskCommands, 1);
  xTaskCreatePinnedToCore(taskPotsLoop, "Pots", 2048, NULL, 1, &taskPots, 1);
  */
}

void loop()
{

  
}
