#include "Motor.h"

void Motor::init(){
    pinMode(DIR_PIN_1,OUTPUT);
    pinMode(DIR_PIN_2,OUTPUT);

    digitalWrite(DIR_PIN_1,LOW);
    digitalWrite(DIR_PIN_2,LOW);

    pinMode(PWM_PIN_1,OUTPUT);
    pinMode(PWM_PIN_2,OUTPUT);
    
    analogWriteResolution(pwm_res); // 0 to 16383, or 16384 for high
    analogWriteResolution(pwm_res);

    analogWriteFrequency(PWM_PIN_1,pwm_freq);
    analogWriteFrequency(PWM_PIN_2,pwm_freq);
    
    analogWrite(PWM_PIN_1,0);
    analogWrite(PWM_PIN_2,0);
}

void Motor::drive(int16_t pwm_1, int16_t pwm_2){
    pwm_1 = math.saturate((float)pwm_1,-16383.0,16383.0);
    pwm_2 = math.saturate((float)pwm_2,-16383.0,16383.0);

    if(pwm_1<0){
        digitalWrite(DIR_PIN_1,LOW);
        analogWrite(PWM_PIN_1,-pwm_1);
    }
    else{
        digitalWrite(DIR_PIN_1,HIGH);
        analogWrite(PWM_PIN_1,pwm_1);
    }

    if(pwm_2<0){
        digitalWrite(DIR_PIN_2,LOW);
        analogWrite(PWM_PIN_2,-pwm_2);
    }
    else{
        digitalWrite(DIR_PIN_2,HIGH);
        analogWrite(PWM_PIN_2,pwm_2);
    }
}

void Motor::drive_V(double V_1, double V_2, double V_bat = 12.0){
    double pwm_1 = (PWM_MAX*V_1)/V_bat;
    double pwm_2 = (PWM_MAX*V_2)/V_bat;
    drive((int16_t)pwm_1, (int16_t)pwm_2);
}
