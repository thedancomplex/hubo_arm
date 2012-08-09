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

void hubo_arm_kin( rfx_ctrl_ws_t *G ) {
    double T[12];
    hubo_arm_kin_(G->q, T, T+9, G->J);
    memcpy( G->x, T+9, sizeof(G->x) );
    aa_tf_rotmat2quat( T, G->r );
}



int main( int argc, char **argv ) {
    (void) argc; (void) argv;

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

    FILE *f_ref = fopen("ref.dat", "w");
    FILE *f_act = fopen("act.dat", "w");

    const double delta_t = 0.1;
    for( double t = 0; t < 10.0; t += delta_t ) {

        // compute reference position
        // FIXME: would work better with reference velocity too
        memcpy( G.x_r, x0, sizeof(x0) );
        G.x_r[2] += .05 * sin(t); // do a fist pump, but sinusoidally

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

        printf("%f r:\t", t);
        aa_dump_vec(stdout, G.x_r, 3 );
        printf("%f a:\t", t);
        aa_dump_vec(stdout, G.x, 3 );


        // print reference and actual for plotting
        fprintf(f_ref, "%f %f\n", t, G.x_r[2]);
        fprintf(f_act, "%f %f\n", t, G.x[2]);

    }

    fclose(f_ref);
    fclose(f_act);

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
