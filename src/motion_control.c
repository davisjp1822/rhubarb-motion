/*
*	motion_control.c
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <bsd/string.h>
#include <stdio.h>
#include <wiringPi.h>

#include "motion_control.h"
#include "globals.h"
#include "pulse_train.h"

extern _Bool VERBOSE;
extern int8_t WIRINGPI_DIRECTION_OUTPUT;

static uint64_t motor_pos = 0;
static int64_t acc_stop_point = 0;
static int64_t dec_start_point = 0;
static struct move_params *this_move;

/* STATE MACHINE SETUP - STEP 1
 * The next several blocks will setup the state machine.
 *
 * First, we define our states. The states are the actual functions that will be called by the control loop.
 * They take no arguments and return a return code based on whether they fail, pass, or repeat.
 *
 * The state functions defined below define the actual actions in the state (move logic, etc).
 * The do NOT define transition logic - they just return values as defined by state_ret_codes.
 *
 * The transition logic is defined by lookup_transitions.
 */

static int state_start(void);
static int state_accel(void);
static int state_run(void);
static int state_decel(void);
static int state_estop(void);
static int state_exit_success(void);
static int state_exit_fail(void);

/* STATE MACHINE SETUP - STEP 2
 * Next, we setup a function pointer and some variables. 
 * A quick note on function pointers. They are like placholders for actual functions. They define what the function looks like, not what the actual function is.
 * This will make more sense in the command loop in execute_move(). Instead of having a giant if/then/else or switch statement, function pointers allow
 * for a very simple state machine loop as the function pointer can be swapped to represent the different state_* functions
 */

/* Here we define an array of function pointers, along with an enum which gives a name to each index of the array.
 * The defintion arguments NEED to match the names of the state functions declared above
 */
int (*state[])(void) = {state_start, state_accel, state_run, state_decel, state_estop, state_exit_success, state_exit_fail};

/* this enum allows us to refer to the array members above by name, rather than index. For example, state_start == 0, etc
 * These values HAVE to be in sync with the array of function pointers above
 */
enum state_codes {start, accel, run, decel, estop, exit_success, exit_fail};

/* set another enum to give us more legible state return codes */
enum state_ret_codes {done, fail, repeat, stop};

/* STATE MACHINE SETUP - STEP 3
 * This table is very important. This is where all of the possible state transitions are mapped. The template is the "transition" struct -
 * which defines a src state (could be current state), the return code from that state, and then the next state based on the return code
 * 
 * Transitions from the exit and estop states don't need to be defined, because we, well, end.
 *
 * The transition map comes from your state diagram. In the case of this program, the diagram (plotted in GNU Dia), may be found in this Github repository.
 */

struct transition
{
	enum state_codes src_state;
	enum state_ret_codes ret_code;
	enum state_codes dst_state;
};

static struct transition state_transitions[] =
{
	{start, done, accel},
	{start, fail, exit_fail},
	{start, repeat, start},
	{start, stop, estop},

	{accel, done, run},
	{accel, fail, exit_fail},
	{accel, repeat, accel},
	{accel, stop, estop},

	{run, done, decel},
	{run, fail, exit_fail},
	{run, repeat, run},
	{run, stop, estop},

	{decel, done, exit_success},
	{decel, fail, exit_fail},
	{decel, repeat, decel},
	{decel, stop, estop}
};

/* Transition lookup routine */
static enum state_codes lookup_transitions(enum state_codes cs, enum state_ret_codes rc);

int execute_move(struct move_params *mp)
{
	/*
	 * when first entering the move, set current state to state_start and init all variables
	 *
	 * current_state = the state the machine is currently in
	 * rc = return code variables
	 * m_state = function pointer that can be assigned to any of the state_* functions
	 *
	 */
	
	this_move = mp;
	acc_stop_point = pow(this_move->velocity, 2)/(2 * this_move->acc);
	dec_start_point = this_move->num_steps - (pow(this_move->velocity, 2)/(2 * this_move->dec));

	if(VERBOSE == true)
	{
		printf("\nMOVE STATISTICS - Trapezoidal Move:\n");
		printf("Total number of steps:\t\t\t%" PRId64 "\n", this_move->num_steps);
		printf("Acceleration stop point (steps):\t%" PRId64 "\n", acc_stop_point);
		printf("Deceleration start point (steps):\t%" PRId64 "\n", dec_start_point);
	}

	/**
	 * SPECIAL CASE - Acceleration stop point is past halfway point of move
	 * Create a triangle move in this case, with an equal acc and dec
	 * 
	 * To make this move work, we have to figure out our original time to complete the move
	 * (Vo/a = To), and then use the original time divided by two (To/2) (since this is a triangle move)
	 * to calculate our new velocity:
	 * Vnew = a * (To/2)
	 **/
	if(acc_stop_point >= this_move->num_steps * 0.5)
	{
		acc_stop_point = this_move->num_steps * 0.5;
		dec_start_point = (this_move->num_steps * 0.5);

		double new_v = sqrt(2*this_move->acc * (this_move->num_steps/2));
		mp->velocity = new_v;

		printf("\nHalf Way Rule!\n");
		printf("acc stop: %" PRId64 "\n", acc_stop_point);
		printf("dec start: %" PRId64 "\n", dec_start_point);
		printf("new velocity: %F\n", mp->velocity);
	}

	enum state_codes current_state = start;
	enum state_ret_codes rc;
	int (*m_state)(void);

	/* this is the main control loop for the state machine 
	 *	
	 *	Thanks to using function pointers, the state machine is quite straightforward. 
	 *	Essentially, m_state is set to point to the current state's state_* function.
	 *	Then, the return code generated by the execution of the appropriate state function is stored in rc.
	 *	Based on the code stored in rc (done, fail, estop, etc), move forward, or don't if it is an exit state or estop state
	 *
	 *	All the while, the called functions (state_start, state_accel, state_decel) are setting the pulse interval,
	 *	which is being executed here.
	 */
	for(;;)
	{
		m_state = state[current_state];
		rc = m_state();
		
		/* setup some conditionals. if in an exit state or estop state, break the loop and exit.
		 * otherwise, call the lookup function to find the next state
		 */
		if(current_state == exit_success)
		{
			break;
		}

		if(current_state == exit_fail)
		{
			/* the calling function should print an error message */
			return EXIT_FAILURE;
		}

		if(current_state == estop)
		{
			/* the calling function should print an error message */
			return EXIT_FAILURE;
		}

		/* if we made it past the conditionals, call the state transition lookup and proceed! */
		current_state = lookup_transitions(current_state, rc);
	}

	/* get here only if we break the for loop due to getting to state_exit_success */
	return EXIT_SUCCESS;
}

static enum state_codes lookup_transitions(enum state_codes cs, enum state_ret_codes rc)
{
	/*
	 * loop through the array state_transitions and find our current state, cs, and the return code.
	 * fetch the next value in the sub array and return the next state code
	 */

	enum state_codes ret_state;
	int8_t i = 0;
	int8_t found = 0;

	for(i=0; i < (sizeof(state_transitions) / sizeof(state_transitions[0])); i++)
	{
		struct transition *t = &state_transitions[i];
		
		if(t->src_state == cs && t->ret_code == rc)
		{
			ret_state = t->dst_state;
			found = 1;
		}
	}

	/* assert that we found a valid state. if not, something is very wrong and we should abort */
	assert(found == 1);
	
	/* also throw in a debug message about a state change */
	if(VERBOSE == true)
	{
		if(ret_state != cs)
		{
			char state_name[100] = {0};

			switch(ret_state)
			{
				case start:
					strlcpy(state_name, "start", sizeof(state_name));
					break;

				case accel:
					strlcpy(state_name, "accel", sizeof(state_name));
					break;

				case run:
					strlcpy(state_name, "run", sizeof(state_name));
					break;

				case decel:
					strlcpy(state_name, "decel", sizeof(state_name));
					break;

				case estop:
					strlcpy(state_name, "e-stop", sizeof(state_name));
					break;

				case exit_success:
					strlcpy(state_name, "exit with success", sizeof(state_name));
					break;

				case exit_fail:
					strlcpy(state_name, "exit with fail", sizeof(state_name));
					break;
				
				default:
					strlcpy(state_name, "unknown", sizeof(state_name));
					break;
			}

			fprintf(stderr, "\nTransitioning to state: %s\n", state_name);
		}
	}

	return ret_state;
}

static int state_start(void)
{
	/* default state is success since this is mostly a setup routine*/
	enum state_ret_codes rc = done;
	
	/*
	 * the starting logic is simple. init the following variables
	 * motor_pos - tracks motor position in steps (always starts at 0)
	 */

	motor_pos = 0;
	
	return rc;
}

static int state_accel(void)
{
	enum state_ret_codes rc;
	
	 /** 
	  * Calling this function fires a loop in pulse_train.c.
	  * The loop returns when the acceleration profile is finished.
	  * If successful, the return will be 0
	  * If not, the return will be < 0
	  * E-Stop will return -2, all other errors -1
	  *
	  * If you wanted to extend the program to do other move types, you would add the math in this function!
	  *
	  * The third parameter, stop_point, is ignored for this move profile!
	  **/
	
	double times[this_move->num_steps];
	memset(times, 0, this_move->num_steps*sizeof(double));

	uint64_t positions[this_move->num_steps];
	memset(positions, 0, this_move->num_steps*sizeof(uint64_t));

	int8_t ret = trap_acc_dec(*this_move, acc_stop_point, &motor_pos, times, positions);
	
	if(ret == 0)
	{
		rc=done;
	}
	
	if(ret == -1)
	{
		rc=fail;
	}

	if(ret == -2)
	{
		rc=stop;
	}

	return rc;
}

static int state_run(void)
{
	enum state_ret_codes rc;

	int64_t run_dist = dec_start_point - acc_stop_point;
	int8_t ret = pulse_train(this_move->velocity, &run_dist, &motor_pos);

	if(ret == 0)
	{
		rc=done;
	}
	
	if(ret == -1)
	{
		rc=fail;
	}

	if(ret == -2)
	{
		rc=stop;
	}

	return rc;
}

static int state_decel(void)
{
	enum state_ret_codes rc;

	/** 
	  * Calling this function fires a loop in pulse_train.c.
	  * The loop returns when the acceleration profile is finished.
	  * If successful, the return will be 0
	  * If not, the return will be < 0
	  * E-Stop will return -2, all other errors -1
	  *
	  * If you wanted to extend the program to do other move types, you would add the math in this function!
	  *
	  * The third parameter, stop_point, is ignored for this move profile!
	  **/
	
	double times[this_move->num_steps];
	memset(times, 0, this_move->num_steps*sizeof(double));

	uint64_t positions[this_move->num_steps];
	memset(positions, 0, this_move->num_steps*sizeof(uint64_t));

	int8_t ret = trap_acc_dec(*this_move, this_move->num_steps, &motor_pos, times, positions);
	
	if(ret == 0)
	{
		rc=done;
	}
	
	if(ret == -1)
	{
		rc=fail;
	}

	if(ret == -2)
	{
		rc=stop;
	}
	
	return rc;
}

static int state_estop(void)
{
	enum state_ret_codes rc;
	
	printf("!!! E-STOP - Stopping Execution!\n");
	rc = fail;
	return rc;
}

static int state_exit_success(void)
{
	enum state_ret_codes rc;
	
	rc = done;
	return rc;
}

/* exact error message should be printed by the caller */
static int state_exit_fail(void)
{
	enum state_ret_codes rc;
	
	rc = fail;
	return rc;
}

struct move_params init_move_params()
{
	struct move_params m;
	
	m.CW=0;
	m.CCW=0;
	m.acc=-1;
	m.dec=-1;
	m.starting_speed = -1;
	m.velocity=-1;
	m.num_steps=-1;
	m.steps_per_rev=2000;
	
	return m;
}

inline void tsnorm(struct timespec *ts)
{
	while(ts->tv_nsec >= NSEC_PER_SEC)
	{
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
		
}
