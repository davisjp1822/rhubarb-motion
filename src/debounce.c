/*
*	debounce.c
*	ThreeML LLC
*	MIT License
*
*	Created by John Davis on 3/11/17. Copyright 2017 3ML LLC
*
*/

#include <wiringPi.h>

#include "globals.h"
#include "debounce.h"

/* DEBOUNCE_TIME is the amount of time in seconds that a switch should have been in a changed state before that state is recognized 
 * SAMPLING_FREQ is the input sampling time in Hz
 * */

static const int8_t DEBOUNCE_TIME = 0.3;
static const int8_t SAMPLING_RATE = 10;
#define MAXIMUM (SAMPLING_RATE / DEBOUNCE_TIME)

int16_t debounce_input_read(const int8_t wiringpi_input, int16_t *integrator, const struct timespec t)
{
	/**
	 * This function is typically going to be called from some type of control loop. Therefore, it is up to use to use t to 
	 * ensure that we are sampling at the correct sampling rate, which is defined by SAMPLING_RATE (Hz).
	 **/
	if((t.tv_nsec/NSEC_PER_MSEC) % SAMPLING_RATE == 0)
	{
		/* Step 1: Update the integrator based on the input signal.  Note that the 
		integrator follows the input, decreasing or increasing towards the limits as 
		determined by the input state (0 or 1). */

		if(digitalRead(wiringpi_input) == LOW)
		{
			if(*integrator > 0)
			{
				(*integrator)--;
			}
		}
		else if(*integrator < MAXIMUM)
		{
    		(*integrator)++;
		}

		/* Step 2: Update the output state based on the integrator.  Note that the
		output will only change states if the integrator has reached a limit, either
		0 or MAXIMUM. */

		if(*integrator == 0)
		{
			return 0;
		}
		else if(*integrator >= MAXIMUM)
		{
			*integrator = MAXIMUM;  /* defensive code if integrator got corrupted */
			return 1;
		}
	}

	/* should never reach here */
	return 0;
}

