/*******************************************************************************
 * @file        inverseKinematics.c
 * @author      Leonardo Acha Boiano
 * @date        6 Dec 2023
 * @brief       Main code, Micro Quadruped Robot.
 * 
 * @note        This code is written in C and is used on the ESP32 38PIN DEVboard.
 *
 *******************************************************************************/

#include "../kinematics/kinematics.h"


// Function to calculate w_ik
float_t w_ik(float_t gamma) {
    return sqrt(pow(D, 2) * (1 + pow(ALPHA, 2) - 2 * ALPHA * cos(gamma)));
}

// Function to calculate theta_2
float_t calculate_theta_2(float_t gamma) {
    return M_PI + gamma;
}

// Function to calculate theta_3
float_t calculate_theta_3(float_t x, float_t y, float_t z, float_t gamma) {
    return atan2(sqrt(1 - pow((z - A1 - A2) / (w_ik(gamma) * gamma), 2)), 
    (z - A1 - A2) / (w_ik(gamma) * gamma));
}

// Function to calculate theta_1
float_t calculate_theta_1(float_t x, float_t y, float_t z, float_t gamma) {
    float_t theta3 = calculate_theta_3(x, y, z, gamma);

    return atan2(y - w_ik(gamma) * gamma * sin(theta3), x + P) - atan2(ALPHA * sin(calculate_theta_2(gamma)), 1 + ALPHA * cos(calculate_theta_2(gamma)));
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
