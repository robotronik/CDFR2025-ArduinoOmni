#pragma once

#include "Arduino.h"

#define BUFFERONRECEIVESIZE 32

//configuration Stepper
#define DEFAULT_MAX_SPEED 5000  // Vitesse maximale (en pas par seconde)
#define DEFAULT_MAX_ACCEL 8000 // Accélération maximale (en pas par seconde carré)

//Pin Moteur pas a pas VALEUR HAUTE 2200 avec Microstep 1/8
#define PIN_STEPPER_SLEEP 22
#define PIN_STEPPER_RESET 23
#define PIN_STEPPER_STEP_1 2
#define PIN_STEPPER_DIR_1 42
#define PIN_STEPPER_ENABLE_1 47

#define PIN_STEPPER_STEP_2 3
#define PIN_STEPPER_DIR_2 43
#define PIN_STEPPER_ENABLE_2 40

#define PIN_STEPPER_STEP_3 4
#define PIN_STEPPER_DIR_3 28
#define PIN_STEPPER_ENABLE_3 30

#define PIN_STEPPER_STEP_4 5
#define PIN_STEPPER_DIR_4 29
#define PIN_STEPPER_ENABLE_4 31

// LED Pins
#define PIN_LED_1_B 66 //A12
#define PIN_LED_1_G 65 //A11
#define PIN_LED_1_R 64 //A10
