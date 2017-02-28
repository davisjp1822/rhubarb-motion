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
#include "motion_control.h"
#include "pulse_train.h"

#include <sys/stat.h>
#include <getopt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/utsname.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sched.h>
#include <sys/mman.h>
#include <wiringPi.h>

extern int8_t WIRINGPI_PULSE_OUTPUT;
extern int8_t WIRINGPI_DIRECTION_OUTPUT;

struct move_params mp;

void	show_usage();
void	check_rt();
void	check_root();
void	parse_args();
void 	rt_setup();

int main(int argc, char *argv[])
{

	mp = init_move_params();
	wiringPiSetup();

	/* pre checks - make sure user is root and that we are running a PREEMPT kernel */
	check_root();
	check_rt();

	/* parse command line arguments and also check that the inputs are within range */
	parse_args(argc, argv);

	/* if we pass the checks, setup a PREEMPT environment */
	rt_setup();

	/* rt setup complete, now pass control to the stepper function */
	execute_move(mp);

	return 0;
}

void rt_setup()
{
	
	/* declare ourselves as a realtime task and set the scheduler */
	struct sched_param param;
	param.sched_priority = 85;
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
	int32_t freq = 0;
	int8_t pulse_flag = 0;
	/* by default, if no options are given, just show the usage */
	if(argc == 1)
	{
		show_usage();
	}

	while ((opt = getopt(argc, argv, "hs:r:g:a:d:v:n:z:t:")) != -1)
	{
		switch (opt) {
			
			case 's':
				mp.starting_speed = atoi(optarg);

				if(mp.starting_speed > 500 || mp.starting_speed <= 0)
				{
					printf("Staring Speed cannot be less than 0 or greater than 500 steps/rev\n");
					exit(-1);
				}
				break;

			case 'r':
				mp.steps_per_rev = atoi(optarg);

				if(mp.steps_per_rev <= 0)
				{
					printf("Drive steps per rev cannot be less than or equal to zero\n");
					exit(-1);
				}
				break;

			case 'g':
				WIRINGPI_PULSE_OUTPUT = atoi(optarg);

				if(WIRINGPI_PULSE_OUTPUT != 28 && WIRINGPI_PULSE_OUTPUT != 29)
				{
					printf("\nERROR: The Rhubarb can only output pulse train signals on WiringPi outputs 28 and 29\n");
					exit(-1);
				}
				break;			
			
			case 'z':
				WIRINGPI_DIRECTION_OUTPUT = atoi(optarg);
				
				if(WIRINGPI_DIRECTION_OUTPUT != 26 && WIRINGPI_DIRECTION_OUTPUT != 27 && WIRINGPI_DIRECTION_OUTPUT != 28 && WIRINGPI_DIRECTION_OUTPUT != 29)
				{
					printf("\nERROR: You must specify a valid output for the Rhubarb using WiringPi outputs 26, 27, 28, or 29.\n");
				}
				break;

			case 'a':
				mp.acc = atoi(optarg);

				if(mp.acc <= 0 || mp.acc > 1000)
				{
					printf("\nERROR: Acceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(-1);
				}
				break;

			case 'd':
				mp.dec = atoi(optarg);

				if(mp.dec <= 0 || mp.dec > 1000)
				{
					printf("\nERROR: Deceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(-1);
				}
				break;

			case 'v':
			{
				mp.velocity = atoi(optarg);

				/*velocity cannot be negative or zero*/
				if(mp.velocity <=0)
				{
					printf("\nERROR: Velocity cannot be less than or equal to 0\n\n");
					exit(-1);
				}

				/* calculate velocity, turn it into a frequency, and test to see if it is above 20kHz */
				freq = 1/((double)1/(mp.velocity*mp.steps_per_rev));

				if(freq > MAX_FREQ)
				{
					fprintf(stderr, "\nERROR: Pulse frequency cannot be greater than %dHz. This limit is derived by the following formula:\n\n1/(1/(velocity * steps_per_rev)).\n", MAX_FREQ);
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
				mp.num_steps = atoi(optarg);

				if(abs(mp.num_steps > INT8_MAX))
				{
					printf("Number of steps/ move value too large\n");
					exit(-1);
				}

				if(mp.num_steps < 0)
				{
					mp.CCW = 1;
				}
				else
				{
					mp.CW = 1;
				}
				break;
			
			case 't':
			{
				freq = atoi(optarg);

				if(freq > MAX_FREQ || freq <= 0)
				{
					fprintf(stderr, "\nERROR: Pulse frequency cannot be greater than %dHz or less than or equal to 0\n", MAX_FREQ);
					exit(-1);
				}
				
				pulse_flag = 1;
				break;
			}
			case 'h' :
			case '?' :
				show_usage();
				break;
				
			default:
				show_usage();
				exit(0);
		}
	}
	
	if(pulse_flag == 1)
	{
		if(pulse(&freq) != 0)
		{
			printf("\nERROR: Error in pulse train execution, exiting...\n");
			exit(-1);
		}
		else
		{
			exit(0);
		}
	}
	else
	{
		if(mp.starting_speed == -1 || mp.steps_per_rev == -1 || mp.acc == -1 || mp.dec == -1 || mp.velocity == -1 || mp.num_steps == -1)
		{
			printf("Missing argument!\n");
			show_usage();
		}
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
	printf("John Davis\n");
	printf("\n");
	printf("\n");
	printf("Using a given acceleration, deceleration, starting speed, and velocity, this program will construct and execute a trapezoidal move profile over the given distance.\n");
	printf("\n");
	printf("\n");
	printf("Usage:\n");
	printf("-h: display this message\n");
	printf("-g: wiringpi pulse train output number (default 29)\n");
	printf("-z: wiringpi step direction output number (default 26)\n");
	printf("-s: starting speed in steps/s (1-500)\n");
	printf("-r: drive steps per revolution (default 2000)\n");
	printf("-a: acceleration in steps/s^2 (1-1000)\n");
	printf("-d: deceleration in steps/s^2 (1-1000)\n");
	printf("-v: velocity in revolutions per second (rps) (not to exceed 20kHz pulse frequency)\n");
	printf("-n: move distance in steps (negative values for CCW rotation, positive values for CW rotation)\n");
	printf("\n");
	printf("\n");
	printf("Special Functions:\n");
	printf("-t: Create pulse train at specified frequency in Hz\n");
	printf("\n");
	printf("\n");
	printf("Example: ./rhubarb_motion -s 100 -a 250 -d 250 -v 10 -n 10000\n");
	printf("This would move the stepper - with a starting speed of 100 steps/s, an acceleration of 250 steps/s/s, a deceleration of 250 steps/s/s, a velocity of 10 RPS (600RPM) - move 10,000 steps CW\n");
	printf("In this example, we assume the drive is set to 2000 steps/rev, so the stepper would move 5 revolutions (10,000/2,000). If the lead on your actuator is 1\" per revolution, your actuator would love 5\"\n");
	exit(0);
}
