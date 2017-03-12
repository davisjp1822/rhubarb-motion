/*
*	globals.c
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include "globals.h"

const int32_t MAX_FREQ = 30000;

int8_t WIRINGPI_PULSE_OUTPUT = 29;
int8_t WIRINGPI_DIRECTION_OUTPUT = 26;
int8_t WIRINGPI_ESTOP_INPUT = 0;

int8_t VERBOSE = FALSE;
int8_t NO_MOTOR = FALSE;

char OUTPUT_FILE_PATH[PATH_MAX] = {0};
