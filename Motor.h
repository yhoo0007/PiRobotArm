#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "esp32-hal-timer.h"
#include "MotorController.h"

#define MOTOR_TIMER_PRESCALE 80
#define MOTOR_TIMER_FREQ 1000000

#define PIN_UNDEFINED -1
#define ENA_ENABLE LOW
#define ENA_DISABLE HIGH


class Motor {
    public:
    void init(struct MotorConfig motorConfig);
    void moveSteps(int steps, int timeMillis);
    int enable();
    int disable();

    int steps = 0;

    // When 'running' is set to true, the motor controller will periodically update the frequency
    // of the motor.
    bool running = false;
    int currentTimeSlice = 0;
    int timeSlices = 0;
    int counter = 0;  // stores the number of pulses sent
    int target = 0;  // stores the target steps to move
    uint16_t *freqs = NULL;  // stores frequency array
    hw_timer_t *timer = NULL;  // hardware timer of the motor
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    int stepPin;
    int dirPin;
    int enaPin;
};

struct MotorConfig {
    int stepPin = PIN_UNDEFINED;
    int dirPin = PIN_UNDEFINED;
    int enaPin = PIN_UNDEFINED;
    hw_timer_t* timer;
    void (*isr)();
};


#endif
