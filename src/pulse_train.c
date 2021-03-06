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
#include <assert.h>

extern _Bool VERBOSE;
extern _Bool NO_MOTOR;
extern int8_t WIRINGPI_PULSE_OUTPUT;
extern int8_t WIRINGPI_ESTOP_INPUT;

static struct timespec t;

static int8_t _pulse(const long double freq, uint64_t *motor_pos, long double *a_rate, int64_t *stop_point);

/**
 * PULSE TRAIN OPERATION
 * This will send out a pulse of a certain frequency until the user exits with ctrl-c or stop_point is reached.
 * freq: pulse frequency in Hz.
 * stop_point: stopping point in steps. If zero, program assumes infinite move.
 * *motor_pos: current motor position (updated to the caller)
**/
int8_t pulse_train(const int32_t freq, const int64_t *stop_point, uint64_t *motor_pos)
{

	/* if stop_point is 0, then the move is infinite */
	if(stop_point != NULL)
	{
		int64_t abs_stop = abs(*stop_point);

		fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d for %" PRId64 " steps...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT, *stop_point);
		return _pulse(freq, motor_pos, NULL, &abs_stop);
	}

	/* if stop point is NULL, then we are outputting an infinite pulse train */
	fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT);
	return _pulse(freq, motor_pos, NULL, NULL);
}

/** 
 * ACC/DEC OPERATION
 * Calculates an acceleration or deceleration ramp for a Trapezoidal move and executes it.
 * move_params: the move parameters as specified by the user.
 * stop_point: stopping point in steps. effectively either the acceleration stop point or mp->num_steps (deceleration)
 * motor_pos: current motor position (updated to the caller)
 * times: array of the time in nanoseconds for each step that is size num_steps
 * positions: array of the positions for each step that is size num_steps
 **/
int8_t trap_acc_dec(const struct move_params mp, const int64_t stop_point, uint64_t *motor_pos, double *times, uint64_t *positions)
{
	/**
	 * There are several different scenarios to consider in this block:
	 * 1. If motor_pos == 0, we are accelerating. We accelerate until we achieve acc_stop_point, half of num_steps, or reach decel_start_point.
	 *
	 * 2. If stop_point == num_steps, we are decelerating. Keep going until this condition is true.
	 *
	 * For all instances, we should calculate:
	 * acc_stop_point
	 * dec_stop_point
	 * 
	 * Also create placeholder arrays, size mp->num_steps to hold our time and position values for the output file
	 * (assuming that option was given by the user)
	 *
	 **/

	int64_t acc_stop_point = stop_point;	

	/* decelerating */
	if(acc_stop_point == mp.num_steps)
	{
		int8_t retval = 0;
		long double dec = mp.dec * -1;

		printf("\nusing v: %F\n", mp.velocity);

		retval = _pulse(mp.velocity, motor_pos, &dec, &acc_stop_point);

		if( retval < 0)
		{
			return retval;
		}
	}
	/* accelerating */
	else
	{
		/* if motor_pos is indeed 0, we need to start our move with the starting speed and work up from there */
		if(*motor_pos == 0)
		{	
			int8_t retval = 0;
			long double acc = mp.acc;
			retval = _pulse(mp.starting_speed, motor_pos, &acc, &acc_stop_point);

			if( retval < 0)
			{
				return retval;
			}
		}
	}

	/* should never reach here */
	return 0;
}

/**
 * The main pulse driving function. 
 * freq: frequency in Hertz (really, steps/ second)
 * stop_point: the position in steps to stop. If 0, move continues infinitely.
 * *motor_pos: the current position of the motor, in steps
 **/ 
static int8_t _pulse(const long double freq, uint64_t *motor_pos, long double *a_rate, int64_t *stop_point)
{

	if(stop_point != NULL && *stop_point > 0)
	{
		/**
	 	* Get the time, and load it into t. When clock_nanosleep is called, the TIMER_ABSTIME flag waits until the interval specified in t (the next arg). 
	 	* normally, if this were a failure, we would return as such, but since this is kind of important, we bail from the program.
	 	**/
		if(clock_gettime(CLOCK_MONOTONIC, &t) < 0)
		{
			perror("\n!!!ERROR: ");
			exit(EXIT_FAILURE);
		}

		/**
		 * estop_int is the integrator value used to debounce the e-stop switch
		 **/
		int16_t estop_int = 0;

		/* init at 1 so that we start with a pulse */
		int8_t should_pulse = 1;

		/* assert that stop_point is positive */
		if(stop_point != NULL)
		{
			assert(*stop_point >= 0);
		}

		/** calculate the pulse width
		 * Since we are not PWM, the duty cycle is 50%, which means, we have to reverse the f = 1/p equation and then divide by two to get the pulse on/off width
		**/
		long double pulse_width = ((1.0/freq)/2.0)*NSEC_PER_SEC;
		long double cur_freq = freq;
		long double start_time = 0;
		long double stop_time = 0;
		
		if(VERBOSE == true && a_rate == NULL)
		{
			fprintf(stderr, "\nUsing Pulse Width of %Lfs\n", pulse_width/NSEC_PER_SEC);
		}
		
		/* example - adds a one second delay before starting the pulsing */
		/*t.tv_sec++;*/

		while(1)
		{	
			if(start_time == 0)
			{
				start_time = t.tv_nsec;
			}

			/* check for e-stop condition */ 
			if(debounce_input_read(WIRINGPI_ESTOP_INPUT, &estop_int, t) == 1)
			{
				fprintf(stdout, "\n!!!ERROR: E-Stop detected!\n");
				return -2;
			}

			if(should_pulse == 1)
			{	
				if(NO_MOTOR == false)
				{
					digitalWrite(WIRINGPI_PULSE_OUTPUT, HIGH);
				}
				should_pulse = 0;
				(*motor_pos)++;
			}
			else
			{
				if(NO_MOTOR == false)
				{
					digitalWrite(WIRINGPI_PULSE_OUTPUT, LOW);
				}
				should_pulse = 1;
			}

			/* if given an acceleration term, use it here */
			if(a_rate != NULL)
			{
				/* accelerating */
				if(*a_rate > 0)
				{
					cur_freq = cur_freq + ((*a_rate/NSEC_PER_SEC)*pulse_width);
					pulse_width = ((1.0/cur_freq)/2.0)*NSEC_PER_SEC;
				}
				else
				{	
					long double pos_rate = fabsl(*a_rate);
					cur_freq = cur_freq - ((pos_rate/NSEC_PER_SEC)*pulse_width);
					pulse_width = ((1.0/cur_freq)/2.0)*NSEC_PER_SEC;
				}
			}

			/* after pulsing is done, check to see if we have hit the stop limit */
			if(stop_point != NULL && *motor_pos == *stop_point)
			{
				/* reset output to low state */
				printf("\nMOTOR_POS: %"PRId64"\n", *motor_pos);
				printf("\nFINAL FREQ: %LFs\n", cur_freq);
				printf("\nMOVE TIME: %LFs\n", (stop_time/NSEC_PER_SEC));
				digitalWrite(WIRINGPI_PULSE_OUTPUT, LOW);
				return 0;
			}

			stop_time += pulse_width;
			t.tv_nsec += pulse_width;
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

			tsnorm(&t);
		}	
	}
	/* no move - just return success */
	else
	{
		return 0;
	}

	/* should not make it to here */
	return 0;
}
