/*
*  main.c
*  rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*	Compile with:
*	
*
*/

#define _GNU_SOURCE
#define MAX_SAFE_STACK (8*1024)

#include "globals.h"

#include <sys/stat.h>
#include <getopt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/utsname.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

extern int16_t STARTING_SPEED;
extern int16_t STEPS_PER_REV;
extern int8_t WIRINGPI_OUTPUT;
extern int16_t ACC;
extern int16_t DEC;
extern int32_t VELOCITY;
extern int32_t NUM_STEPS;

extern struct timespec t;
struct sched_param param;

extern void	show_usage();
extern void	check_rt();
extern void	check_root();
extern void	parse_args();
extern void rt_setup();

int main(int argc, char *argv[])
{
	
	/* pre checks - make sure user is root and that we are running a PREEMPT kernel */
	check_root();
	check_rt();

	/* parse command line arguments and also check that the inputs are within range */
	parse_args(argc, argv);

	/* if we pass the checks, setup a PREEMPT environment */
	rt_setup();

	return 0;
}

void rt_setup()
{
	/* declare ourselves as a realtime task and set the scheduler */
	param.sched_priority = 49;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1)
	{
		perror("Could not set scheduler");
		exit(-1);
	}

	/* lock memory to prevent page faults - mlockall forces the executing program to lock all memory to RAM, not swap, which is slow */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
	{
		perror("mlockall failed");
		exit(-1);
	}
	
	/* prefault the stack. set aside memory so that there are no interrupts when the system loads the memory pages into cache */
	unsigned char dummy[MAX_SAFE_STACK];
	memset(dummy, 0, MAX_SAFE_STACK);

}

void parse_args(int argc, char **argv)
{	
	int opt = 0;
	
	int16_t STARTING_SPEED = -1;
	int16_t STEPS_PER_REV = 2000;
	int8_t WIRINGPI_OUTPUT = 29;
	int16_t ACC = -1;
	int16_t DEC = -1;
	int32_t VELOCITY = -1;
	int32_t NUM_STEPS = -1;

	/* by default, if no options are given, just show the usage */
	if(argc == 1)
	{
		show_usage();
	}

	while ((opt = getopt(argc, argv, "hs:r:g:a:d:v:n:")) != -1)
	{
		
		switch (opt) {
			
			case 's':
				STARTING_SPEED = atoi(optarg);

				if(STARTING_SPEED > 500 || STARTING_SPEED <= 0)
				{
					printf("Staring Speed cannot be less than 0 or greater than 500 steps/rev\n");
					exit(-1);
				}
				break;

			case 'r':
				STEPS_PER_REV = atoi(optarg);

				if(STEPS_PER_REV <= 0)
				{
					printf("Drive steps per rev cannot be less than or equal to zero\n");
					exit(-1);
				}
				break;

			case 'g':
				WIRINGPI_OUTPUT = atoi(optarg);
			break;			

			case 'a':
				ACC = atoi(optarg);

				if(ACC <= 0 || ACC > 1000)
				{
					printf("Acceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(-1);
				}
				break;

			case 'd':
				DEC = atoi(optarg);

				if(DEC <= 0 || DEC > 1000)
				{
					printf("Deceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(-1);
				}
				break;

			case 'v':
			{
				VELOCITY = atoi(optarg);

				/*velocity cannot be negative or zero*/
				if(VELOCITY <=0)
				{
					printf("\nERROR: Velocity cannot be less than or equal to 0\n\n");
					exit(-1);
				}

				/* calculate velocity, turn it into a frequency, and test to see if it is above 20kHz */
				int32_t freq = 1/((double)1/(VELOCITY*STEPS_PER_REV));

				if(freq > MAX_FREQ)
				{
					fprintf(stderr, "\nERROR: Pulse frequency cannot be greater than %dkHz. This limit is derived by the following formula:\n\n1/(1/(velocity * steps_per_rev)).\n", MAX_FREQ);
					printf("Where velocity is set with option -v (revolutions per second) and steps_per_rev is set via -r (steps per revolution). The latter is usually set in the stepper drive itself.\n\n");
					exit(0);
				}

				else
				{
					fprintf(stderr, "\nOutput freq is: %d\n", freq);
				}
				break;
			}

			case 'n':
				NUM_STEPS = atoi(optarg);

				if(NUM_STEPS > INT8_MAX)
				{
					printf("Number of steps/ move value too large\n");
					exit(-1);
				}
				break;

			case 'h' :
			case '?' :
				show_usage();
				break;
				
			default:
				show_usage();
				exit(0);
		}
	}

	/* if any of these values are uninitialized, dump and show user the help menu */
	if(STARTING_SPEED == -1 || STEPS_PER_REV == -1 || ACC == -1 || DEC == -1 || VELOCITY == -1 || NUM_STEPS == -1)
	{
		printf("Missing argument!\n");
		show_usage();
	}
}

void check_root()
{
	if(geteuid() != 0)
	{
		printf("\n\n***You must be root (try using sudo) to run this program!***\n");
		show_usage();
		exit(-1);
	}
}

void check_rt()
{

	struct utsname u;
    int crit2 = 0;
    FILE *fd;
	char *crit1;

    uname(&u);
    crit1 = strcasestr(u.version, "PREEMPT RT");

    if((fd = fopen("/sys/kernel/realtime", "r")) != NULL)
    {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }

    if(crit1 == NULL && !crit2)
    {
		printf("\n\n***This is NOT a PREEMPT kernel - please install RTLinux***\n");
    	show_usage();
    	exit(-1);
    }
}

void show_usage()
{

	printf("\n");
	printf("Rhubarb Motion - a Pulse Train Motion Controller\n");
	printf("Developed for the Rhubarb Industrial Interface Board for the Raspberry Pi\n");
	printf("Copyright 2017 3ML LLC\n");
	printf("\n");
	printf("\n");
	printf("Using a given acceleration, deceleration, starting speed, and velocity, this program will construct and execute a trapezoidal move profile over the given distance.\n");
	printf("\n");
	printf("\n");
	printf("Usage:\n");
	printf("-h: display this message\n");
	printf("-s: starting speed in steps/s (1-500)\n");
	printf("-r: drive steps per revolution (default 2000)\n");
	printf("-g: wiringpi output number (default 29)\n");
	printf("-a: acceleration in steps/s^2 (1-1000)\n");
	printf("-d: deceleration in steps/s^2 (1-1000)\n");
	printf("-v: velocity in revolutions per second (rps) (not to exceed 20kHz pulse frequency)\n");
	printf("-n: move distance in steps\n");
	exit(0);
}
