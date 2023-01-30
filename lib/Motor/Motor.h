#ifndef MOTOR
#define MOTOR

#include <Arduino.h>
#include "Math_functions.h"

#define DIR_PIN_1 31
#define DIR_PIN_2 30
#define PWM_PIN_1 29
#define PWM_PIN_2 28
#define PWM_MAX 16383.0

class Motor{
    public:
        void init();
        void drive(int16_t pwm_1, int16_t pwm_2);
        void drive_V(double V_1, double V_2, double V_bat=12.0);

    private:
        float pwm_freq = 9155.27; // 109.226707678 us pulse width
        uint32_t pwm_res = 14; // // 0 to 16383, or 16384 for high
        Math_functions math;
};

#endif
