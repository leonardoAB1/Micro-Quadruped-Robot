/*******************************************************************************
 * @file        kinematics.h
 * @author      Leonardo Acha Boiano
 * @date        6 Dec 2023
 * @brief       Main code, Micro Quadruped Robot.
 * 
 * @note        This code is written in C and is used on the ESP32 38PIN DEVboard.
 *
 *******************************************************************************/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "math.h"
#include "stdio.h"

#define M_PI 3.14159

// Mechanism constants
#define A (3.0 / 100)
#define B (10.0 / 100)
#define C (3.0 / 100)
#define D (10.0 / 100)
#define ALPHA 0.707107

// Robot design constants
#define A1 (25.5 / 1000)
#define A2 (6.5 / 1000)
#define P (17.25 / 1000)
#define R_X (36.0 / 1000)
#define R_Y (62.5 / 1000)

#endif /* KINEMATICS_H */

/********************************* END OF FILE ********************************/
/******************************************************************************/
