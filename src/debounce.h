/******************************************************************************
debounce.c
written by Kenneth A. Kuhn
version 1.00

Modified by John Davis
jd@pauldavisautomation.com

The original code may be found at:
http://www.kennethkuhn.com/electronics/debounce.c

Modified to turn this code into a simple function that can be called from a control loop.



This is an algorithm that debounces or removes random or spurious
transistions of a digital signal read as an input by a computer.  This is
particularly applicable when the input is from a mechanical contact.  An
integrator is used to perform a time hysterisis so that the signal must
persistantly be in a logical state (0 or 1) in order for the output to change
to that state.  Random transitions of the input will not affect the output
except in the rare case where statistical clustering is longer than the
specified integration time.

The following example illustrates how this algorithm works.  The sequence 
labeled, real signal, represents the real intended signal with no noise.  The 
sequence labeled, corrupted, has significant random transitions added to the 
real signal.  The sequence labled, integrator, represents the algorithm 
integrator which is constrained to be between 0 and 3.  The sequence labeled, 
output, only makes a transition when the integrator reaches either 0 or 3.  
Note that the output signal lags the input signal by the integration time but 
is free of spurious transitions.
 
real signal 0000111111110000000111111100000000011111111110000000000111111100000
corrupted   0100111011011001000011011010001001011100101111000100010111011100010
integrator  0100123233233212100012123232101001012321212333210100010123233321010
output      0000001111111111100000001111100000000111111111110000000001111111000

I have been using this algorithm for years and I show it here as a code
fragment in C.  The algorithm has been around for many years but does not seem
to be widely known.  Once in a rare while it is published in a tech note.  It 
is notable that the algorithm uses integration as opposed to edge logic 
(differentiation).  It is the integration that makes this algorithm so robust 
in the presence of noise.
******************************************************************************/

#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <time.h>
#include <stdint.h>

int16_t debounce_input_read(const int8_t wiringpi_input, int16_t *integrator, const struct *timespec t);

#endif /* DEBOUNCE_H */ 
