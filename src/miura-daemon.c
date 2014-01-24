/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2013,2014, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <sys/types.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for miura
#include "miura.h"

// for raspi debug
#include <bcm2835.h>
#include <string.h>

// Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)
#define PIN RPI_GPIO_P1_07


#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
			    as the priority of kernel tasklets
			    and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000




/* for serial */
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
//                error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

//        if (tcsetattr (fd, TCSANOW, &tty) != 0)
//                error_message ("error %d setting term attributes", errno);
}


/* end for serial */

typedef struct timeb {
	time_t   time;
	unsigned short millitm;
	short    timezone;
	short    dstflag;
} timeb_t;



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void miuraLoop();
int ftime(timeb_t *tp);






int jnt = 1;



// ach message type

// ach channels
ach_channel_t chan_miura_ref;      // miura-ref
ach_channel_t chan_miura_state;    // miura-state

int debug = 0;
int flipFlag = 0;


void flipBit(){
  char command[50];

  if( flipFlag == 1) {
    strcpy( command, "echo 1 > /sys/class/gpio/gpio11/value" );
    system(command);
//      printf("1\n");
//      bcm2835_gpio_set(RPI_V2_GPIO_P1_07);
//      printf("111\n");
      flipFlag = 0;
		}
  else {
    strcpy( command, "echo 0 > /sys/class/gpio/gpio11/value" );
    system(command);
//      printf("0\n");
//      bcm2835_gpio_clr(RPI_V2_GPIO_P1_07);
//      printf("00\n");
      flipFlag = 1;
		}

}



void miuraLoop() {

	/* for serial debug */
	char *portname = "/dev/ttyUSB0";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
//	        error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
	        return;
	}

//	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_interface_attribs (fd, B230400, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

//	write (fd, "hello!\n", 7);           // send 7 character greeting
//	char buf [100];
//	int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read


	// get initial values for hubo
	miura_ref_t M_ref;
	miura_state_t M_state;
	memset( &M_ref,   0, sizeof(M_ref));
	memset( &M_state, 0, sizeof(M_state));

	size_t fs;
	//int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//assert( sizeof(H) == fs );
	int r = ach_get( &chan_miura_ref, &M_ref, sizeof(M_ref), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(debug) {
			printf("Ref ini r = %s\n",ach_result_to_string(r));}
		}
	else{   assert( sizeof(M_ref) == fs ); }

	r = ach_get( &chan_miura_state, &M_state, sizeof(M_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(debug) {
			printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{
		assert( sizeof(M_state) == fs );
	 }



	// time info
	struct timespec t;
	//int interval = 500000000; // 2hz (0.5 sec)
//	int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	//int interval = 2000000; // 500 hz (0.002 sec)
	  int interval = 100000; // 1000 hz (0.001 sec)


	/* Sampling Period */
	double T = (double)interval/(double)NSEC_PER_SEC; // (sec)

	// get current time
	//clock_gettime( CLOCK_MONOTONIC,&t);
	clock_gettime( 0,&t);

	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 790000;
	while(1) {
		// wait until next shot
//		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		if(nanosleep(&tim , &tim2) < 0 )   
		{
			printf("Nano sleep system call failed \n");
///	return -1;
		}

		/* RASPI pin debug */
//		flipBit();
		
		/* test for serial */
		write (fd, "\n", 1);           // send 7 character greeting

		/* Get latest ACH message */
		r = ach_get( &chan_miura_state, &M_state, sizeof(M_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(debug) {
				printf("State r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(M_state) == fs ); }

// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------


			M_ref.joint[jnt].ref = 1.23456;


// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT BELOW THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
		ach_put( &chan_miura_ref, &M_ref, sizeof(M_ref));
		t.tv_nsec+=interval;
		tsnorm(&t);
	}


}






void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];
	memset( dummy, 0, MAX_SAFE_STACK );
}



static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		//usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}

int main(int argc, char **argv) {

	/* for FASPI debug */
	if (!bcm2835_init())
        	return 1;

     char command[50];
     strcpy( command, "echo 11 > /sys/class/gpio/export" );
     system(command);
     char command2[50];
     strcpy( command2, "echo out > /sys/class/gpio/gpio11/direction" );
     system(command2);
//      printf("1\n");
//      bcm2835_gpio_set(RPI_V2_GPIO_P1_07);


	int vflag = 0;
	int c;

	int i = 1;
	while(argc > i) {
		if(strcmp(argv[i], "-d") == 0) {
			debug = 1;
		}
		i++;
	}

	/* RT */
	struct sched_param param;
	/* Declare ourself as a real time task */
	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	/* Pre-fault our stack */
	stack_prefault();


	/* open ach channel */
	int r = ach_open(&chan_miura_ref, MIURA_CHAN_REF_NAME , NULL);
	assert( ACH_OK == r );

	r = ach_open(&chan_miura_state, MIURA_CHAN_STATE_NAME , NULL);
	assert( ACH_OK == r );

	miuraLoop();
	pause();
	return 0;

}
