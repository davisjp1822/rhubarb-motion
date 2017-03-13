/*
*	motion_control.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <time.h>
#include <stdint.h>

struct move_params
{
	int8_t CW;
	int8_t CCW;
	int16_t starting_speed;
	int32_t acc;
	int32_t dec;
	int32_t velocity;
	int64_t num_steps;
	int32_t steps_per_rev;
};

int execute_move(struct move_params *mp);
struct move_params init_move_params();
inline void tsnorm(struct timespec *ts);

#endif /*MOTION_CONTROL_H*/
