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

/**
 * just a straight pulse train at frequency freq. Just keeps going until the user does a ctrl-c or stop_point is reached
 * if an infinite move is desired, just set stop_point < 0.
 **/
int8_t pulse(const int32_t freq, const int64_t stop_point);

/* this variation of the pulse function pulses at a set freq, accelerating or decelerating with a (steps/s), until velocity (steps/s) OR stop_point (steps) is reached 
 * motor_pos is a pointer to a value that is constantly updated to the caller as the function is running 
 * 
 * IMPORTANT NOTE: in this case, freq is actually the speed at which the move should start!
 *
 */
int8_t pulse(const int32_t freq, const int16_t a, const int32_t velocity, const int64_t stop_point, uint64_t *motor_pos);

#endif /*PULSE_TRAIN_H*/
