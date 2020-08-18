#include <stdint.h>
#include "MotorController.h"
#include "esp32-hal-timer.h"
#include "Debug.h"

#define getMotor(channel) motorController.motors[channel]

MotorController motorController;

/**
 * Initializes the motor controller for the given number of motors.
 */
int MotorController::init(int numMotors, struct MotorConfig motorConfigs[]) {
    DPRINTLN("MC: Setting up motor controller");
    if (isInit || numMotors > MAX_NUM_MOTORS) {
        DPRINTLN("MC: Invalid arguments provided!");
        return 1;
    }
    this->numMotors = numMotors;
    motors = new Motor[numMotors];

    // Setup motor controller timer
    DPRINTLN("MC: Setting up timer");
    timer = timerBegin(MOTOR_CONTROLLER_TIMER_NUM, MOTOR_CONTROLLER_PRESCALE, true);
    timerAlarmWrite(timer, MOTOR_CONTROLLER_CMV, true);
    timerAttachInterrupt(timer, &motorControllerIsr, true);

    // Setup motors
    DPRINTLN("MC: Setting up motors");
    for (int i = 0; i < numMotors; i++) {
        motors[i].init(motorConfigs[i]);
    }

    // Finish up
    isInit = true;
    DPRINTLN("MC: Setup complete");
    return 0;
}


/**
 * Enables the motor controller ISR.
 */
void MotorController::timerEnable() {
    DPRINTLN("MC: Timer enabled");
    timerAlarmEnable(timer);
}


/**
 * Enables the motor on the given channel.
 */
int MotorController::enable(int channel) {
    if (channel < 0 || channel >= numMotors) {
        DPRINTLN("MC: Invalid channel number provided");
        return 1;
    }
    return motors[channel].enable();
}


/**
 * Disables the motor on the given channel.
 */
int MotorController::disable(int channel) {
    if (channel < 0 || channel >= numMotors) {
        DPRINTLN("MC: Invalid channel number provided");
        return 1;
    }
    return motors[channel].disable();
}


/**
 * Sets the steps to move for the given channel.
 */
int MotorController::setSteps(int channel, int steps) {
    if (channel < 0 || channel >= numMotors) {
        DPRINTLN("MC: Invalid channel number provided");
        return 1;
    }
    this->steps[channel] = steps;
    return 0;
}


/**
 * Sets the time to move for the given channel.
 */
int MotorController::setTime(int channel, int timeMillis) {
    if (channel < 0 || channel >= numMotors) {
        DPRINTLN("MC: Invalid channel number provided");
        return 1;
    }
    times[channel] = timeMillis;
    return 0;
}


/**
 * Initiates the move of the motor on the given channel.
 */
void MotorController::move(int channel) {
    DPRINTLN("MC: Move called on channel: " + String(channel) + " steps: " + String(steps[channel]) + " time: " + String(times[channel]));
    motors[channel].moveSteps(steps[channel], times[channel]);
}


/**
 * Returns whether or not the motor controller is running a movement.
 */
bool MotorController::running() {
    bool running = false;
    for (int i = 0; i < numMotors; i++) {
        if (motors[i].running) {
            running = true;
        }
    }
    return running;
}


/**
 * Updates the frequency of all motors and increments its timeslice number.
 */
void IRAM_ATTR motorControllerIsr() {
    portENTER_CRITICAL_ISR(&motorController.timerMux);
    bool idle = true;
    for (int iterator = 0; iterator < motorController.numMotors; iterator++) {
        if (getMotor(iterator).currentTimeSlice < getMotor(iterator).timeSlices) {
            idle = false;
            setFrequency(iterator, getMotor(iterator).freqs[getMotor(iterator).currentTimeSlice]);
            getMotor(iterator).currentTimeSlice++;
        }
    }
    if (idle) {
        DPRINTLN("MC: Disabling timer");
        timerAlarmDisable(motorController.timer);
    }
    portEXIT_CRITICAL_ISR(&motorController.timerMux);
}


/**
 * Sets the frequency of the given channel to the specified frequency.
 */
void IRAM_ATTR setFrequency(int channel, uint16_t frequency) {
    if (frequency > 0) {
        int cmv = round((MOTOR_TIMER_FREQ / 2) / (double)frequency);
        timerAlarmWrite(getMotor(channel).timer, cmv, true);
    }
}


/**
 * The following ISRs correpond to the 3 available motor channels. The reason the individual
 * functions must be coded out explicitly is because we cannot attach ISRs to timers while passing
 * arguments. Hence it is not possible to have a single function that switches between the
 * different motor channels.
 */
void IRAM_ATTR motorIsr0() {
    portENTER_CRITICAL_ISR(&motorController.motors[0].timerMux);
    if (motorController.motors[0].counter < motorController.motors[0].target) {
        bool state = digitalRead(motorController.motors[0].stepPin);
        digitalWrite(motorController.motors[0].stepPin, !state);
        if (state) motorController.motors[0].counter++;
    } else {
        motorController.motors[0].running = false;
        free(motorController.motors[0].freqs);
        timerAlarmDisable(motorController.motors[0].timer);
    }
    portEXIT_CRITICAL_ISR(&motorController.motors[0].timerMux);
}


void IRAM_ATTR motorIsr1() {
    portENTER_CRITICAL_ISR(&motorController.motors[1].timerMux);
    if (motorController.motors[1].counter < motorController.motors[1].target) {
        bool state = digitalRead(motorController.motors[1].stepPin);
        digitalWrite(motorController.motors[1].stepPin, !state);
        if (state) motorController.motors[1].counter++;
    } else {
        motorController.motors[1].running = false;
        free(motorController.motors[1].freqs);
        timerAlarmDisable(motorController.motors[1].timer);
    }
    portEXIT_CRITICAL_ISR(&motorController.motors[1].timerMux);
}


void IRAM_ATTR motorIsr2() {
    portENTER_CRITICAL_ISR(&motorController.motors[2].timerMux);
    if (motorController.motors[2].counter < motorController.motors[2].target) {
        bool state = digitalRead(motorController.motors[2].stepPin);
        digitalWrite(motorController.motors[2].stepPin, !state);
        if (state) motorController.motors[2].counter++;
    } else {
        motorController.motors[2].running = false;
        free(motorController.motors[2].freqs);
        timerAlarmDisable(motorController.motors[2].timer);
    }
    portEXIT_CRITICAL_ISR(&motorController.motors[2].timerMux);
}
