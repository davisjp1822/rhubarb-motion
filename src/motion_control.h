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

/* returns -1 if unsuccessful */
extern int execute_move(struct timespec t, int8_t move_direction, int16_t starting_speed, int16_t acc, int16_t dec, int32_t velocity, int32_t num_steps);

#endif /*MOTION_CONTROL_H*/
