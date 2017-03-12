/*
*	pulse_train.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include "pulse_train.h"
#include "motion_control.h"
#include "globals.h"
#include "debounce.h"

#include <wiringPi.h>
#include <time.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

extern _Bool VERBOSE;
extern _Bool NO_MOTOR;
extern int8_t WIRINGPI_PULSE_OUTPUT;
extern int8_t WIRINGPI_ESTOP_INPUT;

/**
 * This will send out a pulse of a certain frequency until the user exits with ctrl-c or stop_point is reached
**/

int8_t pulse_train(const int32_t freq, const int64_t stop_point, uint64_t *motor_pos)
{
	/* main.c sets stop_point to -1 if the -t option is used but steps are not constrained by -n*/
	if(stop_point > 0)
	{
		/**
	 	 * stop_point should always be positive, since direction is set by an output 
		 * this check and output set is done by the caller
		 **/

		int64_t abs_sp = abs(stop_point);

		fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d for %" PRId64 " steps...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT, abs_sp);
		return pulse_trap(freq, -1, -1, abs_sp, motor_pos);
	}

	/* if stop point is less than 0, then we are outputting an infinite pulse train */
	fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT);
	return pulse_trap(freq, -1, -1, abs_sp, motor_pos);
}

/** 
 * the actual output pulsing brains
 **/
int8_t pulse_trap(const int32_t freq, const int16_t a, const int32_t velocity, const int64_t stop_point, uint64_t *motor_pos)
{
	
	/**
	 * t is the time strcuture used to track the pulse times
	 * estop_int is the integrator value used to debounce the e-stop switch
	 **/
	struct timespec t;
	int16_t estop_int = 0;

	/* init at 1 so that we start with a pulse */
	int8_t should_pulse = 1;

	/** calculate the pulse width
	 * Since we are not PWM, the duty cycle is 50%, which means, we have to reverse the f = 1/p equation and then divide by two to get the pulse on/off width
	**/
	long double pulse_width = 0;
	pulse_width = ((1.0/((long double)freq))/2.0)*NSEC_PER_SEC;
	
	if(VERBOSE == true)
	{
		fprintf(stderr, "\nUsing Pulse Width of %Lfs\n", pulse_width/NSEC_PER_SEC);
	}

	/**
	 * Get the time, and load it into t. When clock_nanosleep is called, the TIMER_ABSTIME flag waits until the interval specified in t (the next arg). 
	 * normally, if this were a failure, we would return as such, but since this is kind of important, we bail from the program.
	 **/
	if(clock_gettime(CLOCK_MONOTONIC, &t) < 0)
	{
		perror("\n!!!ERROR: ");
		exit(EXIT_FAILURE);
	}
	
	/* adds a one second delay before starting the pulsing */
	/*t.tv_sec++;*/
	
	/* this case is for the basic pulse output - just execute an infinite loop that pulses on and off */
	if(a < 0 && velocity < 0)
	{
		/* if doing a simple pulse train and stop_point is specified, track motor_pos and stop when that limit is reached */
		int64_t pos = 0;
		
		while(1)
		{	
			/* check for e-stop condition */ 
			if(debounce_input_read(WIRINGPI_ESTOP_INPUT, &estop_int, t) == 1)
			{
				fprintf(stdout, "\n!!!ERROR: E-Stop detected!\n");
				return -1;
			}

			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

			if(should_pulse == 1)
			{	
				if(NO_MOTOR == false)
				{
					digitalWrite(WIRINGPI_PULSE_OUTPUT, HIGH);
				}
				should_pulse = 0;
				pos++;
			}
			else
			{
				if(NO_MOTOR == false)
				{
					digitalWrite(WIRINGPI_PULSE_OUTPUT, LOW);
				}
				should_pulse = 1;
			}
			
			/* after pulsing is done, check to see if we have hit the stop limit */
			if(stop_point > 0 && stop_point == *motor_pos)
			{
				fprintf(stderr, "\nMove Complete (moved %" PRId64 " steps)\n", *motor_pos);
				return 0;
			}

			t.tv_nsec += pulse_width;
			tsnorm(&t);
		}
	}
	/* okay, this is an actual profile. do some math and make it happen! */
	else
	{
		
	}

	return 0;
}

