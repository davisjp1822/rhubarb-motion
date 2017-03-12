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
extern char OUTPUT_FILE_PATH;

static struct timespec t;

/**
 * Get the time, and load it into t. When clock_nanosleep is called, the TIMER_ABSTIME flag waits until the interval specified in t (the next arg). 
 * normally, if this were a failure, we would return as such, but since this is kind of important, we bail from the program.
 **/
if(clock_gettime(CLOCK_MONOTONIC, &t) < 0)
{
	perror("\n!!!ERROR: ");
	exit(EXIT_FAILURE);
}

static int8_t _pulse(const int32_t freq, const int64_t stop_point, uint64_t *motor_pos);

/**
 * PULSE TRAIN OPERATION
 * This will send out a pulse of a certain frequency until the user exits with ctrl-c or stop_point is reached.
 * freq: pulse frequency in Hz.
 * stop_point: stopping point in steps. If zero, program assumes infinite move.
 * *motor_pos: current motor position (updated to the caller)
**/
int8_t pulse_train(const int32_t freq, const int64_t stop_point, uint64_t *motor_pos)
{
	/**
	 * stop_point should always be positive, since direction is set by an output 
	 * this check and output set is done by the caller
	**/

	int64_t abs_sp = abs(stop_point);

	/* if stop_point is 0, then the move is infinite */
	if(stop_point == 0)
	{
		fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d for %" PRId64 " steps...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT, abs_sp);
		return _pulse(freq, abs_sp, &motor_pos);
	}
	else
	{
		/* if stop point is greater than 0, then we are outputting an infinite pulse train */
		fprintf(stderr, "\nPulsing at %dHz on WiringPi output %d...\nPress Ctrl-C to exit...\n", freq, WIRINGPI_PULSE_OUTPUT);
		return _pulse(freq, abs_sp, &motor_pos);
	}
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
int8_t trap_acc_dec(const struct *move_params mp, const int64_t stop_point, uint64_t *motor_pos, double *times, uint64_t *positions)
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

	uint64_t acc_stop_point = pow(((mp->velocity * mp->steps_per_rev) - mp->starting_speed), 2)/(2 * mp->acc);
    uint64_t dec_start_point = mp->num_steps - acc_stop_point - (pow((mp->velocity * mp->steps_per_rev), 2)/(2 * mp->dec));	

    _Bool outfile = false;
	long double pulse_width = 0;
	int32_t last_freq = 0;

    if(OUTPUT_FILE_PATH[0] != 0)
    {	
    	double ltimes[mp->num_steps] = {0}; 
   		uint64_t lpos[mp->num_steps] = {0};
   		outfile = true;
    }

	while(1)
	{
		/**
		 * ACCELERATION
		 * Since this is a trapezoidal move and acceleration is linear, the algorithim to accelerate is trivial.
		 * 
		 * Start the motor at starting_speed for the first time step, and then call _pulse with freq and stop_point
		 * incremented with the acceleration rate
		 */
		/* calculate the pulse width (s) here - we add this to our array that eventually gets written to the output CSV */
		if(outfile)
		{
			pulse_width = ((1.0/((long double)freq))/2.0)*NSEC_PER_SEC;
		}	

		/* decelerating */
		if(stop_point == mp->num_steps)
		{
			return 0;
		}
		/* accelerating */
		else
		{
			/* if motor_pos is indeed 0, we need to start our move with the starting speed and work up from there */
			if(*motor_pos == 0)
			{	
				int8_t retval = 0;

				retval = _pulse(mp->starting_speed, mp->acc, *motor_pos);

				/* write initial values into output array */
				times[0] = 0;
				positions[0] = *motor_pos;
				last_freq = mp->starting_speed;

				if( retval < 0)
				{
					return retval;
				}
			}
			/* already moving, so keep adding until we hit our stop conditions */
			else
			{
				/* STOP CONDITION 1 - We have reached 1/2 of num_steps and are not yet at acc_stop_point */
				if(*motor_pos >= (mp->num_steps*0.5) && *motor_pos < acc_stop_point)
				{
					return 0;
				}
				
				/* STOP CONDITION 2 - We have reached acc_stop_point */
				if(*motor_pos >= acc_stop_point)
				{
					return 0;
				}
				
				/* STOP CONDITION 3 - We have reached dec_start_point */
				if(*motor_pos >= dec_start_point)
				{
					return 0;
				}

				/* made it to here? we accelerate. */
				int8_t retval = 0;
				int32_t acc_step_int = mp->acc * pulse_width;
				int32_t new_freq = last_freq + acc_step_int;

				retval = _pulse(new_freq, acc_step_int, *motor_pos);

				/*times[*motor_pos - 1] = times[*motor_pos - 2] + pulse_width;
				positions[*motor_pos] = *motor_pos;*/
				last_freq = new_freq;

				if( retval < 0)
				{
					return retval;
				}
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
static int8_t _pulse(const int32_t freq, const int64_t stop_point, uint64_t *motor_pos)
{
	/**
	 * estop_int is the integrator value used to debounce the e-stop switch
	 **/
	int16_t estop_int = 0;

	/* init at 1 so that we start with a pulse */
	int8_t should_pulse = 1;

	/* assert that stop_point is positive */
	assert(stop_point >= 0);

	/** calculate the pulse width
	 * Since we are not PWM, the duty cycle is 50%, which means, we have to reverse the f = 1/p equation and then divide by two to get the pulse on/off width
	**/
	long double pulse_width = 0;
	pulse_width = ((1.0/((long double)freq))/2.0)*NSEC_PER_SEC;
	
	if(VERBOSE == true)
	{
		fprintf(stderr, "\nUsing Pulse Width of %Lfs\n", pulse_width/NSEC_PER_SEC);
	}
	
	/* example - adds a one second delay before starting the pulsing */
	/*t.tv_sec++;*/

	while(1)
	{	
		/* check for e-stop condition */ 
		if(debounce_input_read(WIRINGPI_ESTOP_INPUT, &estop_int, t) == 1)
		{
			fprintf(stdout, "\n!!!ERROR: E-Stop detected!\n");
			return -2;
		}

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

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
		
		/* after pulsing is done, check to see if we have hit the stop limit */
		if(stop_point > 0 && *motor_pos == stop_point)
		{
			/* reset output to low state */
			digitalWrite(WIRINGPI_PULSE_OUTPUT, LOW);
			return 0;
		}

		t.tv_nsec += pulse_width;
		tsnorm(&t);
	}	

	/* should not make it to here */
	return 0;
}
