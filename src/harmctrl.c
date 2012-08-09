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

#include "config.h"
#include "hubo_arm.h"

void hubo_arm_kin( const double *q, double *T12, double *J ) {
    hubo_arm_kin_(q, T12, T12+9, J);
}

int main( int argc, char **argv ) {
    (void) argc; (void) argv;
    double T[12], J[6*6];

    hubo_arm_kin( (double[]){0,0,0,0,0,0}, T, J );

    // print some test positions to check the FK
    printf("down\n");
    aa_dump_mat(stdout, T, 3, 4 );

    printf("forward\n");
    hubo_arm_kin( (double[]){-M_PI_2,0,0,0,0,0}, T, J );
    aa_dump_mat(stdout, T, 3, 4 );

    printf("right\n");
    hubo_arm_kin( (double[]){0,-M_PI_2,0,0,0,0}, T, J );
    aa_dump_mat(stdout, T, 3, 4 );


    printf("forward-elbow\n");
    hubo_arm_kin( (double[]){0,0,0,-M_PI_2,0,0}, T, J );
    aa_dump_mat(stdout, T, 3, 4 );


    printf("out-elbow\n");
    hubo_arm_kin( (double[]){0,0,-M_PI_2,-M_PI_2,0,0}, T, J );
    aa_dump_mat(stdout, T, 3, 4 );

    /* printf("out\n"); */
    /* hubo_arm_kin( (double[]){0,0,-M_PI_2,M_PI_2,0,0}, T, J ); */
    /* aa_dump_mat(stdout, T, 3, 4 ); */

}
