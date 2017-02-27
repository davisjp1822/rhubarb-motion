/*
*	pulse_train.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include "pulse_train.h"

extern int8_t WIRINGPI_PULSE_OUTPUT;

int pulse(const int32_t *freq)
{
	fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d...\nPress Ctrl-C to exit...\n\n", *freq, WIRINGPI_PULSE_OUTPUT);

	

	return 0;
}

