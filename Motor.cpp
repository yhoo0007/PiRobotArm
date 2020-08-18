#include <stdint.h>
#include "esp32-hal.h"  // timer functions, pinMode, etc
#include "math.h"
#include "Motor.h"
#include "Calc.h"

extern MotorController motorController;

/**
 * Takes a motor configuration and initializes the motor. Also initializes the timer associated
 * with the motor.
 */
void Motor::init(struct MotorConfig config) {
    stepPin = config.stepPin;
    dirPin = config.dirPin;
    enaPin = config.enaPin;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    if (enaPin != PIN_UNDEFINED) {
        pinMode(enaPin, OUTPUT);
    }
    
    freqs = (uint16_t*)malloc(sizeof(uint16_t) * 1);
    freqs[0] = 0;
    
    timer = config.timer;
    timerAttachInterrupt(timer, config.isr, true);

    // To ensure timer initializes properly
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    timerAlarmDisable(timer);
}


/**
 * Enables the motor if an enable pin is defined.
 */
int Motor::enable() {
    if (enaPin != PIN_UNDEFINED) {
        digitalWrite(enaPin, ENA_ENABLE);
        return 0;
    }
    return 1;
}


/**
 * Disables the motor if an enable pin is defined.
 */
int Motor::disable() {
    if (enaPin != PIN_UNDEFINED) {
        digitalWrite(enaPin, ENA_DISABLE);
        return 0;
    }
    return 1;
}


/**
 * Moves the motor the given number of steps in the given amount of time. Polarity of the steps
 * argument determines the direction of the movement.
 */
void Motor::moveSteps(int stepsToMove, int timeMillis) {
    if (stepsToMove == 0) return;
    steps += stepsToMove;
    digitalWrite(dirPin, stepsToMove < 0);

    // Calculate speed profile
    stepsToMove = abs(stepsToMove);
    timeSlices = timeMillis / MOTOR_CONTROLLER_UPDATE_INTERVAL_MILLIS;
    int fMax = stepsToMove / (float)timeMillis * 1000 * 2;
    int fDelta = fMax / timeSlices * 2;
    int correctiveSteps = stepsToMove * 100;

    // Form frequency array
    int midPoint = timeSlices / 2;
    freqs = (uint16_t*)malloc(sizeof(uint16_t) * timeSlices);
    for (int i = 0; i < timeSlices; i++) {
        freqs[i] = i < midPoint ?
                    fDelta * i :
                    fDelta * (timeSlices - i);
        correctiveSteps -= freqs[i];
    }

    // Pad corrective steps into the frequency array
    int i = 0;
    while (correctiveSteps > 0) {
        freqs[i]++;
        correctiveSteps--;
        i = (i + 1) % timeSlices;
    }

    // Set pulse target and reset variables
    target = stepsToMove;
    counter = 0;
    currentTimeSlice = 0;

    // Start movement
    running = true;
    motorController.timerEnable();
    timerAlarmEnable(timer);
}
