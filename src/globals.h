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
#define TRUE 1
#define FALSE 0

#include <stdint.h>

const int32_t MAX_FREQ;
int8_t WIRINGPI_PULSE_OUTPUT;
int8_t WIRINGPI_DIRECTION_OUTPUT;
int8_t VERBOSE;

#endif /*GLOBALS_H*/
