#include <Arduino.h>
#include <ContinuousStepper.h>
#include <ContinuousStepper/Tickers/KhoiH_PWM.hpp>
#include <AVR_PWM.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "RGB_LED.h"
#include "common/protocol.h"
#include "control.h"
#include "odometry/I2Cdevice.h"
#include "odometry/OTOS.h"

// Comment this line to disable serial debug
#define SERIAL_DEBUG

// TODO: move these defines later
#define SERVO_COUNT 7
#define STEPPER_COUNT 4
#define SENSOR_COUNT 8

RGB_LED led(PIN_LED_1_R, PIN_LED_1_G, PIN_LED_1_B);

ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> stepper1;
ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> stepper2;
ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> stepper3;
ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> stepper4;

Wheel wheelA(130, 180, 60, &stepper1);  // WheelA at 0°
Wheel wheelB(130, 60, 60, &stepper2);  // WheelB at 120°
Wheel wheelC(130,-60, 60, &stepper4);  // WheelC at 240°

position_t currentPosition, targetPosition, currentVelocity, currentAcceleration;

// SDA, SCL
I2CDevice i2cDevice(Wire, 0x17);
OTOS otos;


void receiveEvent(int numBytes);
void requestEvent();
void initStepper(ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> &stepper, int maxSpeed, int Accel, int enablePin);
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

  initStepper(stepper1, PIN_STEPPER_STEP_1, PIN_STEPPER_DIR_1, PIN_STEPPER_ENABLE_1);
  initStepper(stepper2, PIN_STEPPER_STEP_2, PIN_STEPPER_DIR_2, PIN_STEPPER_ENABLE_2);
  initStepper(stepper3, PIN_STEPPER_STEP_3, PIN_STEPPER_DIR_3, PIN_STEPPER_ENABLE_3);
  initStepper(stepper4, PIN_STEPPER_STEP_4, PIN_STEPPER_DIR_4, PIN_STEPPER_ENABLE_4);

  currentPosition = { 0.0, 0.0, 0.0 };
  targetPosition = { 0.0, 0.0, 0.0 };

  while (otos.begin(i2cDevice) == ret_FAIL){
    Serial.println("OTOS not connected !");
    delay(100);
  }
  Serial.println("OTOS connected !");
  otos.calibrateImu();
  otos.setPosition(currentPosition);
}

void loop()
{
  led.run();
  stepper1.loop();
  stepper2.loop();
  stepper3.loop();
  stepper4.loop();


  position_t measPos, measVel, measAcc;

  return_t ret = otos.getPosVelAcc(measPos, measVel, measAcc);
  if (ret == ret_FAIL)
  {
    Serial.println("OTOS not connected !");
    return;
  }
  if (ret == ret_OK)
  {
    /*
    Serial.print("Pos: x: ");
    Serial.print(measPos.x, 1);
    Serial.print("  y: ");
    Serial.print(measPos.y, 1);
    Serial.print("  a: ");
    Serial.println(measPos.a, 1);
    */
    currentPosition = measPos;
    currentVelocity = measVel;
    currentAcceleration = measAcc;
    updateWheels(currentPosition, targetPosition,
      wheelA, wheelB, wheelC);
  }
}
void initStepper(ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>> &stepper, int step_pin, int dir_pin, int enablePin)
{
  stepper.begin(step_pin, dir_pin); // step pin must support PWM
  stepper.spin(0);
  stepper.setAcceleration(1000000); // 1000 steps/s^2
  initOutPin(enablePin, false);
  //stepper.setEnablePin(enablePin);
  //stepper.setPinsInverted(false, false, true);
  //stepper.disableOutputs();
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