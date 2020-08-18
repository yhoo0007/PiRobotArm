#ifndef CALC_H
#define CALC_H


#define DEGREE_PER_STEP 0.225
#define angleToSteps(angle, ratio) ((angle) / DEGREE_PER_STEP * (ratio))
#define stepsToAngle(steps, ratio) ((steps) / (ratio) * DEGREE_PER_STEP)


#endif