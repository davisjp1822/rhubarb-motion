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
#define MAX_SAFE_STACK (100*1024)

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
#include <bsd/string.h>
#include <linux/limits.h>

extern int8_t WIRINGPI_PULSE_OUTPUT;
extern int8_t WIRINGPI_DIRECTION_OUTPUT;
extern int8_t WIRINGPI_ESTOP_INPUT;
extern _Bool VERBOSE;
extern _Bool NO_MOTOR;
extern char OUTPUT_FILE_NAME[PATH_MAX];

struct move_params mp;

void	show_usage(void);
void	check_rt(void);
void	check_root(void);
void	parse_args();
void 	rt_setup(void);

int main(int argc, char *argv[])
{

	mp = init_move_params();
	wiringPiSetup();

	/* pre checks - make sure user is root and that we are running a PREEMPT kernel */
	check_root();
	check_rt();

	/* if we pass the checks, setup a PREEMPT environment */
	rt_setup();

	/* parse command line arguments and also check that the inputs are within range
	 * If the command line arguments pass muster, then parse_args handles either making a move happen or doing the pulse train output
	 */
	parse_args(argc, argv);

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
		exit(EXIT_FAILURE);
	}

	/* lock memory to prevent page faults - mlockall forces the executing program to lock all memory to RAM, not swap, which is slow */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
	{
		perror("mlockall failed");
		exit(EXIT_FAILURE);
	}
	
	/* prefault the stack. set aside memory so that there are no interrupts when the system loads the memory pages into cache */
	unsigned char dummy[MAX_SAFE_STACK];
	memset(dummy, 0, MAX_SAFE_STACK);

}

void parse_args(int argc, char **argv)
{	
	int8_t opt = 0;
	int32_t freq = 0;
	int8_t pulse_flag = 0;

	/* by default, if no options are given, just show the usage */
	if(argc == 1)
	{
		show_usage();
	}

	while ((opt = getopt(argc, argv, "yhsq:r:g:a:d:v:n:z:t:x:o:")) != -1)
	{
		switch (opt) {
			
			case 's':
				mp.starting_speed = atoi(optarg);

				if(mp.starting_speed > 500 || mp.starting_speed <= 0)
				{
					printf("Staring Speed cannot be less than 0 or greater than 500 steps/rev\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'r':
				mp.steps_per_rev = atoi(optarg);

				if(mp.steps_per_rev <= 0)
				{
					printf("Drive steps per rev cannot be less than or equal to zero\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'g':
				WIRINGPI_PULSE_OUTPUT = atoi(optarg);

				if(WIRINGPI_PULSE_OUTPUT != 28 && WIRINGPI_PULSE_OUTPUT != 29)
				{
					printf("\nERROR: The Rhubarb can only output pulse train signals on WiringPi outputs 28 and 29\n");
					exit(EXIT_FAILURE);
				}
				break;			
			
			case 'z':
				WIRINGPI_DIRECTION_OUTPUT = atoi(optarg);
				
				if(WIRINGPI_DIRECTION_OUTPUT != 26 && WIRINGPI_DIRECTION_OUTPUT != 27 && WIRINGPI_DIRECTION_OUTPUT != 28 && WIRINGPI_DIRECTION_OUTPUT != 29)
				{
					printf("\nERROR: You must specify a valid output for the Rhubarb using WiringPi outputs 26, 27, 28, or 29.\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'x':
				WIRINGPI_ESTOP_INPUT = atoi(optarg);

				if(WIRINGPI_ESTOP_INPUT > 31 || WIRINGPI_ESTOP_INPUT < 0 || WIRINGPI_ESTOP_INPUT >= 26)
				{
					printf("\nERROR: You must specify a valid WiringPi input for use with the E-Stop\n");
					exit(EXIT_FAILURE);
				}

			case 'a':
				mp.acc = atoi(optarg);

				if(mp.acc <= 0 || mp.acc > 1000)
				{
					printf("\nERROR: Acceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'd':
				mp.dec = atoi(optarg);

				if(mp.dec <= 0 || mp.dec > 1000)
				{
					printf("\nERROR: Deceleration cannot be less than or equal to 0 or greater than 1000\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'v':
			{
				mp.velocity = atoi(optarg);

				/*velocity cannot be negative or zero*/
				if(mp.velocity <=0)
				{
					printf("\nERROR: Velocity cannot be less than or equal to 0\n\n");
					exit(EXIT_FAILURE);
				}

				/* calculate velocity, turn it into a frequency, and test to see if it is above MAX_FREQ in kHz */
				freq = 1/((double)1/(mp.velocity*mp.steps_per_rev));

				if(freq > MAX_FREQ)
				{
					fprintf(stderr, "\nERROR: Pulse frequency cannot be greater than %dHz. This limit is derived by the following formula:\n\n1/(1/(velocity * steps_per_rev)).\n", MAX_FREQ);
					printf("Where velocity is set with option -v (revolutions per second) and steps_per_rev is set via -r (steps per revolution). The latter is usually set in the stepper drive itself.\n\n");
					exit(EXIT_SUCCESS);
				}

				else
				{
					fprintf(stderr, "\nOutput freq is: %d\n", freq);
				}
				break;
			}

			case 'n':
				mp.num_steps = atoi(optarg);

				if(abs(mp.num_steps > INT64_MAX))
				{
					printf("\nNumber of steps/ move value too large\n");
					exit(EXIT_FAILURE);
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
					exit(EXIT_FAILURE);
				}

				if(abs(mp.num_steps > INT64_MAX))
				{
					printf("\nNumber of steps/ move value too large\n");
					exit(EXIT_FAILURE);
				}
				
				pulse_flag = 1;
				break;
			}

			case 'o':
			{
				/* test to see if we can open the file. if not, tell the user and bail */
				FILE *fp;
				char filePath[PATH_MAX] = {0};

				strlcpy(filePath, optarg, sizeof(filePath));

				if((fp=fopen(filePath, "w")) == NULL)
				{
					perror("\nERROR: ");
					exit(EXIT_FAILURE);
				}
				else
				{
					fclose(fp);
					strlcpy(OUTPUT_FILE_PATH, filePath, sizeof(OUTPUT_FILE_PATH));
				}

				break;
			}

			case 'q':
			{
				NO_MOTOR = true;
				break;
			}

			case 'y':
				VERBOSE = true;
				break;

			case 'h' :
			case '?' :
				show_usage();
				break;
				
			default:
				show_usage();
				exit(EXIT_SUCCESS);
		}
	}
	
	/* prior to jumping into an execution routine, finish set up for the WiringPi I/O */
	pinMode(WIRINGPI_PULSE_OUTPUT, OUTPUT);
	pinMode(WIRINGPI_DIRECTION_OUTPUT, OUTPUT);
	pinMode(WIRINGPI_ESTOP_INPUT, INPUT);

	pullUpDnControl(WIRINGPI_PULSE_OUTPUT, PUD_DOWN);
	pullUpDnControl(WIRINGPI_DIRECTION_OUTPUT, PUD_DOWN);
	pullUpDnControl(WIRINGPI_ESTOP_INPUT, PUD_DOWN);

	/**
	 * direction logic is set here - go ahead and turn on the output
	 * for the AMCI SD7540, a HIGH output is CW
	 **/
 
 	if(mp.CW == 1)
	{
		digitalWrite(WIRINGPI_DIRECTION_OUTPUT, 1);
	}
 
 	if(mp.CCW == 1)
 	{
		digitalWrite(WIRINGPI_DIRECTION_OUTPUT, 0);
	}

	/**
	 * check which mode we are using - just a pulse train output, or an actual move profile. act accordingly
	 **/

	if(pulse_flag == 1)
	{
		if(pulse_train(freq, mp.num_steps) != 0)
		{
			printf("\nERROR: Error in pulse train execution, exiting...\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			exit(EXIT_SUCCESS);
		}
	}
	else
	{
		if(mp.starting_speed == -1 || mp.steps_per_rev == -1 || mp.acc == -1 || mp.dec == -1 || mp.velocity == -1 || mp.num_steps == -1)
		{
			printf("Missing argument!\n");
			show_usage();
		}
		else
		{
			/* error messages are printed by execute_move() */
			if(execute_move(&mp) != 0)
			{
				exit(EXIT_FAILURE);
			}
			else
			{
				printf("Motion Complete!\n");
				exit(EXIT_SUCCESS);
			}
		}
	}
}

void check_root()
{
	if(geteuid() != 0)
	{
		printf("\n***You must be root (try using sudo) to run this program!***\n\n");
		exit(EXIT_FAILURE);
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
		printf("\n***This is NOT a PREEMPT kernel - please install RTLinux***\n\n");
    	exit(EXIT_FAILURE);
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
	printf("-x: wiringpi E-Stop input number (default 0)\n");
	printf("-y: turns on verbose output\n");
	printf("-o: outputs motion profile to <filename>\n");
	printf("-q: does NOT actually run the motor, just simulates the run. Useful to use with -o if you want to graph the motion profile.\n");
	printf("\n");
	printf("-s: starting speed in steps/s (1-500)\n");
	printf("-r: drive steps per revolution (default 2000)\n");
	printf("-a: acceleration in steps/s^2 (1-1000)\n");
	printf("-d: deceleration in steps/s^2 (1-1000)\n");
	printf("-v: velocity in revolutions per second (rps) (not to exceed 20kHz pulse frequency)\n");
	printf("-n: move distance in steps (negative values for CCW rotation, positive values for CW rotation)\n");
	printf("\n");
	printf("\n");
	printf("Special Functions:\n");
	printf("-t: Create pulse train at specified frequency in Hz. Can be used with -n to specify number of steps to pulse\n");
	printf("\n");
	printf("\n");
	printf("Example: ./rhubarb_motion -s 100 -a 250 -d 250 -v 10 -n 10000 -o profile.csv\n");
	printf("This would move the stepper - with a starting speed of 100 steps/s, an acceleration of 250 steps/s/s, a deceleration of 250 steps/s/s, a velocity of 10 RPS (600RPM) - move 10,000 steps CW\n");
	printf("In this example, we assume the drive is set to 2000 steps/rev, so the stepper would move 5 revolutions (10,000/2,000). If the lead on your actuator is 1\" per revolution, your actuator would love 5\"\n");
	exit(EXIT_SUCCESS);
}
