/*
*	pulse_train.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include "pulse_train.h"
#include "motion_control.h"

#include <time.h>
#include <stdlib.h>
#include <wiringpi.h>

extern int8_t WIRINGPI_PULSE_OUTPUT;

/**
 * This will send out a pulse of a certain frequency until the user exits with ctrl-c
**/

int pulse(const int32_t *freq)
{
	fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d...\nPress Ctrl-C to exit...\n\n", *freq, WIRINGPI_PULSE_OUTPUT);

	struct timespec t;

	/* init at 1 so that we start with a pulse */
	int8_t should_pulse = 1;

	/** calculate the pulse width
	 * Since we are not PWM, the duty cycle is 50%, which means, we have to reverse the f = 1/p equation and then divide by two to get the pulse on/off width
	**/
	double pulse_width = (1/(*freq))/2;

	/* setup the GPIO pin - wiringPiSetup() was called in main.c */
	pinMode(WIRINGPI_PULSE_OUTPUT, OUTPUT);

	/* get the time, load it into t, and increment t to 1s, which effectively delays execution for 1s. When clock_nanosleep is called, the TIMER_ABSTIME flag waits until the interval specified in t (the next arg). */
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	
	while(1)
	{
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

		if(should_pulse == 1)
		{
			digitalWrite(WIRINGPI_PULSE_OUTPUT, HIGH);
			should_pulse = 0;
		}
		else
		{
			digitalWrite(WIRINGPI_PULSE_OUTPUT, LOW);
			should_pulse = 1;
		}

		t.tv_nsec += pulse_width;
		tsnorm(&t);
	}

	return 0;
}

