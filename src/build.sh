#!/bin/bash

clear
gcc -Wall -lrt -lwiringPi -lm -lbsd globals.c motion_control.c pulse_train.c debounce.c main.c -g
