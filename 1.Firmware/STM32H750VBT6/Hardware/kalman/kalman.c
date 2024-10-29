#include "kalman.h"

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) 
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void EKF_init(EKF_t *ekf) {
    ekf->Q_angle = 0.001f;
    ekf->Q_bias = 0.003f;
    ekf->R_measure = 0.03f;

    ekf->pitch = 0.0f;
    ekf->roll = 0.0f;
    ekf->bias_pitch = 0.0f;
    ekf->bias_roll = 0.0f;

    ekf->bias_yaw = 0.0f;
    ekf->yaw = 0.0f;

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
}

Angle_Data EKF_update(EKF_t *ekf, icm42688_data_float acc, icm42688_data_float gyro, double dt) 
{
    double accel_x = acc.x;
    double accel_y = acc.y;
    double accel_z = acc.z;

    double gyro_x = gyro.x;
    double gyro_y = gyro.y;
    double gyro_z = gyro.z;

    float rate_pitch = gyro_y - ekf->bias_pitch;
    float rate_roll = gyro_x - ekf->bias_roll;
    float rate_yaw = gyro_z - ekf->bias_yaw;

    ekf->pitch += dt * rate_pitch;
    ekf->roll += dt * rate_roll;
    ekf->yaw += dt * rate_yaw;


    ekf->P[0][0] += dt * (dt * ekf->P[2][2] - ekf->P[0][2] - ekf->P[2][0] + ekf->Q_angle);
    ekf->P[1][1] += dt * (dt * ekf->P[3][3] - ekf->P[1][3] - ekf->P[3][1] + ekf->Q_angle);
    ekf->P[2][2] += ekf->Q_bias * dt;
    ekf->P[3][3] += ekf->Q_bias * dt;


    float pitch_measured = atan2f(accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * (180.0 / PI);
    float roll_measured = atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) * (180.0 / PI);

    float S_pitch = ekf->P[0][0] + ekf->R_measure;
    float S_roll = ekf->P[1][1] + ekf->R_measure;
    float K_pitch[2], K_roll[2];
    K_pitch[0] = ekf->P[0][0] / S_pitch;
    K_pitch[1] = ekf->P[2][0] / S_pitch;
    K_roll[0] = ekf->P[1][1] / S_roll;
    K_roll[1] = ekf->P[3][1] / S_roll;

   
    float pitch = pitch_measured - ekf->pitch;
    float roll = roll_measured - ekf->roll;
    ekf->pitch += K_pitch[0] * pitch;
    ekf->roll += K_roll[0] * roll;
    ekf->bias_pitch += K_pitch[1] * pitch;
    ekf->bias_roll += K_roll[1] * roll;


    ekf->P[0][0] -= K_pitch[0] * ekf->P[0][0];
    ekf->P[2][0] -= K_pitch[1] * ekf->P[2][0];
    ekf->P[1][1] -= K_roll[0] * ekf->P[1][1];
    ekf->P[3][1] -= K_roll[1] * ekf->P[3][1];

    Angle_Data angle;
    angle.Pitch = ekf->pitch;
    angle.Roll = ekf->roll;
    angle.Yaw = ekf->yaw;

    return angle;
}
