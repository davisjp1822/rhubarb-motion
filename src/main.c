/*
*  main.c
*  rhubarb_motion
*
*	Created by John Davis on 2/20/17. Copyright 2017 3ML LLC
*	Compile with:
*	
*
*/

#include <sys/stat.h>
#include <getopt.h>
#include <string.h>
#include <stdio.h>
#include <sys/utsname.h>
#include <wiringPi.h>

extern int8_t STARTING_SPEED;
extern int8_t STEPS_PER_REV;
extern int8_t WIRINGPI_OUTPUT;
extern int8_t ACC;
extern int8_t DEC;
extern int8_t VELOCITY;
extern int8_t NUM_STEPS;

extern void	show_usage();
extern void	check_rt();
extern void	check_root();
extern void	parse_args();

int main(int argc, char *argv[])
{
	
	/* pre checks - make sure user is root and that we are running a PREEMPT kernel */
	check_root();
	check_rt();

	/* parse command line arguments and also check that the inputs are within range */
	parse_args(argc, argv);

	/* if we pass the checks, setup a PREEMPT environment */


	return 0;
}

void parse_args(int argc, char **argv)
{	
	int opt = 0;
	
	int8_t STARTING_SPEED = -1;
	int8_t STEPS_PER_REV = 2000;
	int8_t WIRINGPI_OUTPUT = 29;
	int8_t ACC = -1;
	int8_t DEC = -1;
	int8_t VELOCITY = -1;
	int8_t NUM_STEPS = -1;

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
				/* calculate velocity, turn it into a frequency, and test to see if it is above 20kHz */

				break;

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
		fprintf(stderr, "You must be root (try using sudo) to run this program!\n");
		show_usage();
		exit(-1);
	}
}

void check_rt()
{

	struct utsname u;
    int crit1, crit2 = 0;
    FILE *fd;

    uname(&u);
    crit1 = strcasestr(u.version, "PREEMPT RT");

    if((fd = fopen("/sys/kernel/realtime", "r")) != NULL)
    {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }

    if(!crit1 && !crit2)
    {
    	fprintf(stderr, "This is NOT a PREEMPT kernel - please install RTLinux\n");
    	show_usage();
    	exit(-1);
    }
}

void show_usage()
{

	printf(stderr, "\n");
	printf(stderr, "Rhubarb Motion - a Pulse Train Motion Controller\n");
	printf(stderr, "Developed for the Rhubarb Industrial Interface Board for the Raspberry Pi\n");
	printf(stderr, "Copyright 2017 3ML LLC\n");
	printf(stderr, "\n");
	printf(stderr, "\n");
	printf(stderr, "Using a given acceleration, deceleration, starting speed, and velocity, this program will construct and execute a trapezoidal move profile over the given distance.\n");
	printf(stderr, "\n");
	printf(stderr, "\n");
	printf(stderr, "Usage:\n");
	printf(stderr, "-h: display this message\n");
	printf(stderr, "-s: starting speed in steps/s (1-500)\n");
	printf(stderr, "-r: drive steps per revolution (default 2000)\n");
	printf(stderr, "-g: wiringpi output number (default 29)\n");
	printf(stderr, "-a: acceleration in steps/s^2 (1-1000)\n");
	printf(stderr, "-d: deceleration in steps/s^2 (1-1000)\n");
	printf(stderr, "-v: velocity in steps/s (not to exceed 20kHz)\n");
	printf(stderr, "-n: move distance in steps\n");
	exit(0);
}
