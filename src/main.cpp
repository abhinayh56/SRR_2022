#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_driver.h"
#include "Time_utils.h"
#include "QuadEncoder.h"
#include "Math_functions.h"
#include "Wheel_odom.h"
#include "Motor.h"
#include "Tilt.h"

#define LOOP_FREQ 250.0f

Time_utils time_utils(LOOP_FREQ);

I2Cdev i2c_master;

MPU6050_driver mpu6050(MPU6050_ADDR_DEFAULT);
struct IMU_data_raw{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t wx;
    int16_t wy;
    int16_t wz;
};

struct IMU_data_real{
    float ax;
    float ay;
    float az;
    float temp;
    float wx;
    float wy;
    float wz;
};

struct IMU_data_raw  imu_raw;
struct IMU_data_real imu_real;

Tilt tilt;

struct Quat{
    double q0 = 1.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
};

Quat get_tilt(double ax, double ay, double az){
    Quat q;
    double a = sqrt(ax*ax + ay*ay + az*az);
    ax = ax/a;
    ay = ay/a;
    az = az/a;
    if(az>=0){
        q.q0 = sqrt((az+1.0)/2.0);
        q.q1 = ay/sqrt(2.0*(az+1.0));
        q.q2 = -ax/sqrt(2.0*(az+1.0));
        q.q3 = 0.0;
    }
    else{
        q.q0 = -ay/sqrt(2.0*(1.0-az));
        q.q1 = -sqrt((1.0-az)/2.0);
        q.q2 = 0.0;
        q.q3 = -ax/sqrt(2.0*(1.0-az));
    }
    return q;
}

struct RPY{
    double phi = 0.0;
    double th  = 0.0;
    double psi = 0.0;
};

RPY quat2eul(Quat q){
    RPY rpy;
    rpy.phi = atan2(2.0*(q.q0*q.q1+q.q2*q.q3), 1.0-2.0*(q.q1*q.q1+q.q2*q.q2));
    rpy.th  = asin(2.0*(q.q0*q.q2-q.q3*q.q1));
    rpy.psi = atan2(2.0*(q.q0*q.q3+q.q1*q.q2),1.0-2.0*(q.q2*q.q2+q.q3*q.q3));
    return rpy;
}

QuadEncoder enc_left(1, 2, 3);
QuadEncoder enc_right(2, 4, 5);

Wheel_odom odom;
double x, y, th;
double v, w;

Motor motor;

void transform_imu();
void init_mpu6050();
void init_encoders();
void init_odom();

void setup(){
    Serial.begin(115200);

    motor.init();
    motor.drive(0,0);

    i2c_master.initialize();

    init_mpu6050();
    init_encoders();
    init_odom();

    time_utils.set_t_last_micros();
}

void loop(){
    mpu6050.get_MPU6050_OUT(&imu_raw.ax, &imu_raw.ay, &imu_raw.az, &imu_raw.temp, &imu_raw.wx, &imu_raw.wy, &imu_raw.wz);
    transform_imu();
    Quat q = get_tilt(imu_raw.ax, imu_raw.ay, imu_raw.az);
    RPY angle = quat2eul(q);

    double g_sen = sqrt(imu_raw.ax*imu_raw.ax + imu_raw.ay*imu_raw.ay + imu_raw.az*imu_raw.az);
    double phi_ac = atan2(imu_raw.ay,imu_raw.az);
    double th_ac = asin(-imu_raw.ax/g_sen);

    Serial.print(phi_ac*180.0/PI); Serial.print('\t');
    Serial.print(angle.phi*180.0/PI); Serial.print('\t');

    Serial.print(angle.th*180.0/PI); Serial.print('\t');
    Serial.print(th_ac*180.0/PI); Serial.print('\t');

    Serial.print(angle.psi*180.0/PI); Serial.print('\t');

    Serial.print((angle.phi-phi_ac)*180.0/PI); Serial.print('\t');
    Serial.println((angle.th-th_ac)*180.0/PI);

    odom.update_odom(-enc_left.read(),enc_right.read());
    odom.get_xyth(&x, &y, &th);
    odom.get_vw(&v, &w);
    
    double V_L = 0.0;
    double V_R = 0.0;
    motor.drive_V(V_R, V_L, 12.0); // 16383 (12)

    time_utils.wait();
}

void transform_imu(){
    int16_t tempAcX = imu_raw.ax;
    int16_t tempAcY = imu_raw.ay;
    int16_t tempAcZ = imu_raw.az;
    imu_raw.ax = tempAcZ;
    imu_raw.ay = -tempAcY;
    imu_raw.az = tempAcX;

    int16_t tempGyX = imu_raw.wx;
    int16_t tempGyY = imu_raw.wy;
    int16_t tempGyZ = imu_raw.wz;
    imu_raw.wx = tempGyZ;
    imu_raw.wy = -tempGyY;
    imu_raw.wz = tempGyX;
}

void init_mpu6050(){
    mpu6050.initialize(); //imu.set_SLEEP(SET_0);
    mpu6050.set_DLPF_CFG(DLPF_CFG_1);
    mpu6050.set_SMPLRT_DIV(SMPLRT_DIV);
    mpu6050.set_FS_SEL(FS_SEL_1);
    mpu6050.set_AFS_SEL(AFS_SEL_1);
}

void init_encoders(){
    enc_left.setInitConfig();
    enc_left.init();
    enc_right.setInitConfig();
    enc_right.init();
}

void init_odom(){
    odom.set_dt(1.0/LOOP_FREQ);
    odom.set_ddr_param(133600,0.0625,0.265);
}
