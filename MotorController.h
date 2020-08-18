#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "esp32-hal-timer.h"
#include "Motor.h"

#define MAX_NUM_MOTORS 3
#define MOTOR_CONTROLLER_TIMER_NUM 3
#define MOTOR_CONTROLLER_PRESCALE 80
#define MOTOR_CONTROLLER_CMV 10000  // prescale & cmv causes ISR to be called at 100Hz
#define MOTOR_CONTROLLER_UPDATE_INTERVAL_MILLIS 10


class Motor;
class MotorController {
    public:
    int init(int numMotors, struct MotorConfig motorConfigs[]);
    void timerEnable();
    int enable(int channel);
    int disable(int channel);
    int setSteps(int channel, int steps);
    int setTime(int channel, int time);
    void move(int channel);
    bool running();

    int numMotors = 0;

    hw_timer_t* timer = NULL;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    Motor *motors;

    private:
    int steps[MAX_NUM_MOTORS];
    int times[MAX_NUM_MOTORS];
    bool isInit = false;
};

void motorControllerIsr();
void setFrequency(int channel, uint16_t frequency);
void motorIsr0();
void motorIsr1();
void motorIsr2();

extern MotorController motorController;

#endif
