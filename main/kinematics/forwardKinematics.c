/*******************************************************************************
 * @file        forwardKinematics.c
 * @author      Leonardo Acha Boiano
 * @date        6 Dec 2023
 * @brief       Main code, Micro Quadruped Robot.
 * 
 * @note        This code is written in C and is used on the ESP32 38PIN DEVboard.
 *
 *******************************************************************************/

#include "../kinematics/kinematics.h"

float_t gamma_fk(float_t theta_2) {
    return theta_2 - M_PI;
}

float_t w_fk(float_t theta_2) {
    return sqrt(D * D * (1 + ALPHA * ALPHA - 2 * ALPHA * cos(gamma_fk(theta_2))));
}

// Kinematic Equations Left Front Leg
float_t x_lf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * cos(theta_1) - D * ALPHA * cos(theta_1 + theta_2) + (P + R_X);
}

float_t y_lf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return (A1 + A2 + R_Y) + w_fk(theta_2) * cos(theta_3);
}

float_t z_lf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * sin(theta_1) - ALPHA * D * sin(theta_1 + theta_2) - w_fk(theta_2) * sin(theta_3);
}

// Kinematic Equations Right Front Leg
float_t x_rf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * cos(theta_1) - D * ALPHA * cos(theta_1 + theta_2) + (P + R_X);
}

float_t y_rf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -(A1 + A2 + R_Y) + w_fk(theta_2) * cos(M_PI - theta_3);
}

float_t z_rf(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * sin(theta_1) - ALPHA * D * sin(theta_1 + theta_2) - w_fk(theta_2) * sin(M_PI - theta_3);
}

// Kinematic Equations Right Back Leg
float_t x_rb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * cos(theta_1) - D * ALPHA * cos(theta_1 + theta_2) - (P + R_X);
}

float_t y_rb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -(A1 + A2 + R_Y) + w_fk(theta_2) * cos(M_PI - theta_3);
}

float_t z_rb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * sin(theta_1) - ALPHA * D * sin(theta_1 + theta_2) - w_fk(theta_2) * sin(M_PI - theta_3);
}

// Kinematic Equations Left Back Leg
float_t x_lb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * cos(theta_1) - D * ALPHA * cos(theta_1 + theta_2) - (P + R_X);
}

float_t y_lb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return (A1 + A2 + R_Y) + w_fk(theta_2) * cos(theta_3);
}

float_t z_lb(float_t theta_1, float_t theta_2, float_t theta_3) {
    return -D * sin(theta_1) - ALPHA * D * sin(theta_1 + theta_2) - w_fk(theta_2) * sin(theta_3);
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
