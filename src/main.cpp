#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "RGB_LED.h"
#include "common/protocol.h"
#include "control.h"

// Comment this line to disable serial debug
// #define SERIAL_DEBUG

// TODO: move these defines later
#define SERVO_COUNT 7
#define STEPPER_COUNT 4
#define SENSOR_COUNT 8

RGB_LED led(PIN_LED_1_R, PIN_LED_1_G, PIN_LED_1_B);

AccelStepper steppers[STEPPER_COUNT] = {
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3},
    {AccelStepper::DRIVER, PIN_STEPPER_STEP_4, PIN_STEPPER_DIR_4, PIN_STEPPER_ENABLE_4},
};

Wheel wheelA(150, 0, steppers[0]);    // WheelA at 0°
Wheel wheelB(150, 120, steppers[1]);  // WheelB at 120°
Wheel wheelC(150, 240, steppers[2]);  // WheelC at 240°

position_t currentPosition, targetPosition;

uint8_t onReceiveData[BUFFERONRECEIVESIZE];
// int onReceiveDataSize = 0;
uint8_t ResponseData[BUFFERONRECEIVESIZE];
int ResponseDataSize = 0;

void receiveEvent(int numBytes);
void requestEvent();
void initStepper(AccelStepper &stepper, int maxSpeed, int Accel, int enablePin);
void initOutPin(int pin, bool low);
void initInPin(int pin);

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

  initOutPin(PIN_STEPPER_SLEEP, false);
  initOutPin(PIN_STEPPER_RESET, false);
  delay(1);
  initStepper(steppers[0], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_1);
  initStepper(steppers[1], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_2);
  initStepper(steppers[2], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_3);
  initStepper(steppers[3], DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCEL, PIN_STEPPER_ENABLE_4);


  currentPosition = { 0.0, 0.0, 0.0 };
  targetPosition = { 1000.0, 0.0, 90.0 };

  Wire.begin(I2C_ADDRESS);
  Wire.setTimeout(1000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop()
{
  for (int i = 0; i < STEPPER_COUNT; i++)
    steppers[i].run();
  led.run();

  updateWheels(currentPosition, targetPosition,
                wheelA, wheelB, wheelC);
}

void receiveEvent(int numBytes)
{
  if (!Wire.available())
    return;

  Wire.readBytes(onReceiveData, numBytes);

#ifdef SERIAL_DEBUG
  Serial.print("Received: 0x ");
  for (int i = 0; i < numBytes; i++)
  {
    Serial.print(onReceiveData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  uint8_t *ptr = onReceiveData;
  uint8_t command = ReadUInt8(&ptr);
  uint8_t number = ReadUInt8(&ptr);

#ifdef SERIAL_DEBUG
  Serial.print("Command: ");
  Serial.println(command, HEX);
  Serial.print("Number: ");
  Serial.println(number);
#endif

  uint8_t *resp_ptr = ResponseData; // + ResponseDataSize; // For requests
  switch (command)
  {
  case CMD_MOVE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].moveTo(ReadInt32(&ptr));
    break;
  case CMD_ENABLE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].enableOutputs();
    break;
  case CMD_DISABLE_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].disableOutputs();
    break;
  case CMD_RGB_LED:
    if (number != 1)
      break;
    led.recieveData(ptr);
    break;
  case CMD_SET_STEPPER: // Set the stepper position at the recieved posititon
    if (number > STEPPER_COUNT || number < 1)
      break;
    steppers[number - 1].setCurrentPosition(ReadInt32(&ptr));
    break;
  case CMD_SET_STEPPER_SPEED: // Set the stepper speed at the recieved posititon
  {
    if (number > STEPPER_COUNT || number < 1)
      break;
    int32_t speed = ReadInt32(&ptr);
    if (speed > 0)
      steppers[number - 1].setMaxSpeed(speed);
    else
      steppers[number - 1].setMaxSpeed(DEFAULT_MAX_SPEED);
  }
  // Request commands
  case CMD_GET_VERSION:
    WriteUInt8(&resp_ptr, API_VERSION);
  break;
  case CMD_GET_STEPPER:
    if (number > STEPPER_COUNT || number < 1)
      break;
    WriteInt32(&resp_ptr, steppers[number - 1].currentPosition());
#ifdef SERIAL_DEBUG
    // Serial.print("Stepper value is :");
    // Serial.println(steppers[number - 1].currentPosition());
#endif
    break;
  default:
    break;
  }
  // onReceiveDataSize -= ptr - onReceiveData;
  ResponseDataSize += resp_ptr - ResponseData;

  return;
}

void requestEvent()
{
#ifdef SERIAL_DEBUG
  Serial.print("Request ! Sending: 0x ");
  for (int i = 0; i < ResponseDataSize; i++)
  {
    Serial.print(ResponseData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (ResponseDataSize == 4)
  {
    Serial.print("In long is :");
    uint8_t *ptr = ResponseData;
    long val = ReadInt32(&ptr);
    Serial.println(val);
  }
#endif

  Wire.write(ResponseData, ResponseDataSize);
  ResponseDataSize = 0;
}

void initStepper(AccelStepper &stepper, int maxSpeed, int accel, int enablePin)
{
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();
  return;
}

void initOutPin(int pin, bool low)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, low ? LOW : HIGH);
  return;
}

void initInPin(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  return;
}