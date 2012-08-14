/* Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <amino.h>
#include <unistd.h>
#include <reflex.h>

#include "config.h"
#include "hubo_arm.h"

#include <stdio.h>


// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include <../hubo-ACH/hubo.h>

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"


// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000

// ach message type
typedef struct hubo hubo[1];

// ach channels
ach_channel_t chan_num;



static inline void tsnorm(struct timespec *ts){

//      clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
        // calculates the next shot
        while (ts->tv_nsec >= NSEC_PER_SEC) {
                //usleep(100);  // sleep for 100us (1us = 1/1,000,000 sec)
                ts->tv_nsec -= NSEC_PER_SEC;
                ts->tv_sec++;
        }
}


void hubo_arm_kin( rfx_ctrl_ws_t *G ) {
    double T[12];
    hubo_arm_kin_(G->q, T, T+9, G->J);
    memcpy( G->x, T+9, sizeof(G->x) );
    aa_tf_rotmat2quat( T, G->r );
}

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset( dummy, 0, MAX_SAFE_STACK );
}


int main( int argc, char **argv ) {
    (void) argc; (void) argv;
	printf("1\n");
// RT
int r = ach_open(&chan_num, "hubo", NULL); 
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


    rfx_ctrl_ws_t G;
    rfx_ctrl_ws_lin_k_t K;
    rfx_ctrl_ws_init( &G, 6 );
    rfx_ctrl_ws_lin_k_init( &K, 6 );

    // init position
    G.q[0] = -M_PI_2;
    G.q[3] = -M_PI_2;
    hubo_arm_kin( &G );
    memcpy(G.r_r, G.r, sizeof(G.r));

    K.dls = .005;
    for( size_t i = 0; i < 6; i ++ ) {
        G.q_min[i] = -6*M_PI;
        G.q_max[i] = 6*M_PI;
        K.p[i] = 10; // FIXME: this gain may break a real robot
        K.q[i] = 0.0;
        K.f[i] = 0.0;
    }
    for( size_t i = 0; i < 3; i ++ ) {
        G.x_min[i] = -10;
        G.x_max[i] = 10;
    }

    double x0[3];
    memcpy( x0, G.x, sizeof(x0) );

//    FILE *f_ref = fopen("ref.dat", "w");
//    FILE *f_act = fopen("act.dat", "w");

 
	printf("2\n");

/* Main Loop */

// For timing
// time info
        struct timespec t;
        //int interval = 500000000; // 2hz (0.5 sec)
        int interval = 10000000; // 100 hz (0.01 sec)
        double T = 0.2;        // period

    	const double delta_t = T;
        double f = 0.1;
	// get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);

	printf("3\n");
// get initial values for hubo
        hubo H;
        size_t fs;
	printf("4\n");
        r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
	double TT = 10.0;
   for( double tt = 0; tt < TT; tt += delta_t ) {
	r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
        // compute reference position
        // FIXME: would work better with reference velocity too
        memcpy( G.x_r, x0, sizeof(x0) );
        G.x_r[2] += .05 * sin(2.0*pi*f*tt); // do a fist pump, but sinusoidally

        // compute reference velocity
        double dq[6];
        {
            int i = rfx_ctrl_ws_lin_vfwd( &G, &K, dq );
            assert( 0 == i );
        }

        // bozo simulate/integrate
        for( size_t i = 0; i < 6; i ++ ) {
            G.q[i] += delta_t * dq[i];
        }

        // update kinematics
        hubo_arm_kin( &G );
	
	// Write to ach
	H->joint[RSP].ref = G.q[0];
	H->joint[RSR].ref = G.q[1];
	H->joint[RSY].ref = G.q[2];
	H->joint[REB].ref = G.q[3];
	H->joint[RWY].ref = G.q[4];
	H->joint[RWP].ref = G.q[5];
	ach_put(&chan_num, H, sizeof(H));

	t.tv_nsec+=interval;
	tsnorm(&t);

	if( tt > (TT - 2*delta_t) ) {
		tt = 0.0;
	}

/* Kill the printing	
        printf("%f r:\t", t);
        aa_dump_vec(stdout, G.x_r, 3 );
        printf("%f a:\t", t);
        aa_dump_vec(stdout, G.x, 3 );

        // print reference and actual for plotting
        fprintf(f_ref, "%f %f\n", t, G.x_r[2]);
        fprintf(f_act, "%f %f\n", t, G.x[2]);
*/
    }

//    fclose(f_ref);
//    fclose(f_act);

    /* hubo_arm_kin( (double[]){0,0,0,0,0,0}, T, J ); */

    /* // print some test positions to check the FK */
    /* printf("down\n"); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */

    /* printf("forward\n"); */
    /* hubo_arm_kin( (double[]){-M_PI_2,0,0,0,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */

    /* printf("right\n"); */
    /* hubo_arm_kin( (double[]){0,-M_PI_2,0,0,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */


    /* printf("forward-elbow\n"); */
    /* hubo_arm_kin( (double[]){0,0,0,-M_PI_2,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */


    /* printf("out-elbow\n"); */
    /* hubo_arm_kin( (double[]){0,0,-M_PI_2,-M_PI_2,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */

    /* printf("out\n"); */
    /* hubo_arm_kin( (double[]){0,0,-M_PI_2,M_PI_2,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */

    rfx_ctrl_ws_destroy( &G );
    rfx_ctrl_ws_lin_k_destroy( &K );
}
