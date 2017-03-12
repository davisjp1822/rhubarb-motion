/*
*	globals.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#ifndef GLOBALS_H
#define GLOBALS_H

#define NSEC_PER_SEC 1000000000
#define NSEC_PER_MSEC 1000000
#define TRUE 1
#define FALSE 0

#define PULSE_ERR_ESTOP -2
#define PULSE_ERR_FAIL -1

#include <stdint.h>
#include <linux/limits.h>

const int32_t MAX_FREQ;

int8_t WIRINGPI_PULSE_OUTPUT;
int8_t WIRINGPI_DIRECTION_OUTPUT;
int8_t WIRINGPI_ESTOP_INPUT;

int8_t VERBOSE;
int8_t NO_MOTOR;

char OUTPUT_FILE_PATH[PATH_MAX];

#endif /*GLOBALS_H*/
