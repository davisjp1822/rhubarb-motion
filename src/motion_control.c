/*
*	motion_control.c
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#include <stdint.h>
#include "motion_control.h"

/* returns -1 if unsuccessful */
int execute_move(struct timespec t, struct move_params mp)
{


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
