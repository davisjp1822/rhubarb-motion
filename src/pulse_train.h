/*
*	pulse_train.h
*	rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*
*/

#ifndef PULSE_TRAIN_H
#define PULSE_TRAIN_H

#include <stdint.h>
#include <stdio.h>

#include "motion_control.h"

/**
 * PULSE TRAIN OPERATION
 * This will send out a pulse of a certain frequency until the user exits with ctrl-c or stop_point is reached.
 * freq: pulse frequency in Hz. If a_rate is specified, this is the starting_speed for the acceleration ramp!!
 * *a_rate: acceleration rate in steps/s/s. NULL if using constant velocity profile
 * *stop_point: stopping point in steps. If NULL, program assumes infinite move.
 * *motor_pos: current motor position (updated to the caller)
**/
int8_t pulse_train(const int32_t freq, const int64_t *stop_point, uint64_t *motor_pos);

/** 
 * ACC/DEC OPERATION
 * Calculates an acceleration or deceleration ramp for a Trapezoidal move and executes it.
 * move_params: the move parameters as specified by the user.
 * stop_point: stopping point in steps. effectively either the acceleration stop point or mp->num_steps (deceleration)
 * motor_pos: current motor position (updated to the caller)
 **/
int8_t trap_acc_dec(const struct move_params mp, const int64_t stop_point, uint64_t *motor_pos, double *times, uint64_t *positions);

#endif /*PULSE_TRAIN_H*/
