#ifndef __KALMAN_H
#define __KALMAN_H

#include "main.h"
#include "icm42688.h"

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


typedef struct EKF_t{
    float Q_angle;   
    float Q_bias;   
    float R_measure; 

    float pitch;     
    float roll;     
    float bias_pitch;
    float bias_roll; 

    float P[4][4];

    float bias_yaw;
    float yaw;
} EKF_t;

typedef struct icm42688_data_float icm42688_data_float;
typedef struct Angle_Data Angle_Data;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void EKF_init(EKF_t *ekf);
Angle_Data EKF_update(EKF_t *ekf, icm42688_data_float acc, icm42688_data_float gyro, double dt);

#endif
