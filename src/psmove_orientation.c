
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "psmove.h"
#include "psmove_private.h"
#include "psmove_orientation.h"

#ifdef CMAKE_BUILD
#include <unistd.h>
#endif

struct _PSMoveOrientation {
    PSMove *move;

    /* 9 values = 3x accelerometer + 3x gyroscope + 3x magnetometer */
    float output[9];

    /* Current sampling frequency */
    float sample_freq;

    /* Sample frequency measurements */
    long sample_freq_measure_start;
    long sample_freq_measure_count;

    /* Output value as quaternion */
    float quaternion[4];

    /* Quaternion measured when controller points towards camera */
    float reset_quaternion[4];
};


PSMoveOrientation *
psmove_orientation_new(PSMove *move)
{
    psmove_return_val_if_fail(move != NULL, NULL);

    if (!psmove_has_calibration(move)) {
        psmove_DEBUG("Can't create orientation - no calibration!\n");
        return NULL;
    }

    PSMoveOrientation *orientation = calloc(1, sizeof(PSMoveOrientation));

    orientation->move = move;

    /* Initial sampling frequency */
    orientation->sample_freq = 120.0f;

    /* Measurement */
    orientation->sample_freq_measure_start = psmove_util_get_ticks();
    orientation->sample_freq_measure_count = 0;

    /* Initial quaternion */
    orientation->quaternion[0] = 1.f;
    orientation->quaternion[1] = 0.f;
    orientation->quaternion[2] = 0.f;
    orientation->quaternion[3] = 0.f;

    return orientation;
}

// This algorithm comes directly from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
// There exists a version of this algorithm online, but the maintainers GPL'ed the code.
// Since this is too restrictive a license for many developers I pulled the code directly
// from the paper (which has no such license restriction).

// TODO: Redo this method in terms of vector/matrix ops. 
// The heart of this is just a matrix multiplication of a 4x6 matrix (jacobian) by a 6x1 vector(objective function)

// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979f * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrtf(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrtf(3.0f / 4.0f) * gyroMeasDrift // compute zeta

// Global system variables
//float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

// Function to compute one filter iteration
static void psmove_orientation_filter_update(
	float *quaternion,
	float deltat,
	float w_x, float w_y, float w_z,
	float a_x, float a_y, float a_z,
	float m_x, float m_y, float m_z)
{
	float SEq_1 = quaternion[0];
	float SEq_2 = quaternion[1];
	float SEq_3 = quaternion[2];
	float SEq_4 = quaternion[3];

	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
		J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
	//
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; // computed flux in the earth frame

	// auxiliary variables to avoid repeated calculations
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = SEq_1 * SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = SEq_2 * SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;

	// normalize the accelerometer measurement
	norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// normalize the magnetometer measurement
	norm = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;

	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3; // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;

	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

	// normalize the gradient to estimate direction of the gyroscope error
	norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;

	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

	// compute and remove the gyroscope baises
	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;

	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// compute then integrate the estimated quaternion rate
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

	// normalize quaternion
	norm = sqrtf(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;

	// compute flux in the earth frame
	SEq_1SEq_2 = SEq_1 * SEq_2; // recompute auxiliary variables
	SEq_1SEq_3 = SEq_1 * SEq_3;
	SEq_1SEq_4 = SEq_1 * SEq_4;
	SEq_3SEq_4 = SEq_3 * SEq_4;
	SEq_2SEq_3 = SEq_2 * SEq_3;
	SEq_2SEq_4 = SEq_2 * SEq_4;
	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);

	// normalize the flux vector to have only components in the x and z
	b_x = sqrtf((h_x * h_x) + (h_y * h_y));
	b_z = h_z;

	quaternion[0] = SEq_1;
	quaternion[1] = SEq_2;
	quaternion[2] = SEq_3;
	quaternion[3] = SEq_4;
}

void
psmove_orientation_update(PSMoveOrientation *orientation)
{
    psmove_return_if_fail(orientation != NULL);

    int frame;

    long now = psmove_util_get_ticks();
    if (now - orientation->sample_freq_measure_start >= 1000) {
        float measured = ((float)orientation->sample_freq_measure_count) /
            ((float)(now-orientation->sample_freq_measure_start))*1000.f;
        psmove_DEBUG("Measured sample_freq: %f\n", measured);

        orientation->sample_freq = measured;
        orientation->sample_freq_measure_start = now;
        orientation->sample_freq_measure_count = 0;
    }

    /* We get 2 measurements per call to psmove_poll() */
    orientation->sample_freq_measure_count += 2;

    psmove_get_magnetometer_vector(orientation->move,
            &orientation->output[6],
            &orientation->output[7],
            &orientation->output[8]);

    float q0 = orientation->quaternion[0];
    float q1 = orientation->quaternion[1];
    float q2 = orientation->quaternion[2];
    float q3 = orientation->quaternion[3];

    for (frame=0; frame<2; frame++) {
        psmove_get_accelerometer_frame(orientation->move, (enum PSMove_Frame)(frame),
                &orientation->output[0],
                &orientation->output[1],
                &orientation->output[2]);

		psmove_get_gyroscope_frame(orientation->move, (enum PSMove_Frame)(frame),
                &orientation->output[3],
                &orientation->output[4],
                &orientation->output[5]);

		psmove_orientation_filter_update(
			orientation->quaternion,
			1.f / orientation->sample_freq, // deltaT = 1/frequency

			/* Gyroscope */
			orientation->output[3],
			orientation->output[5],
			-orientation->output[4],

			/* Accelerometer */
			orientation->output[0],
			orientation->output[2],
			-orientation->output[1],

			/* Magnetometer */
			orientation->output[6],
			orientation->output[8],
			orientation->output[7]);

        if (isnan(orientation->quaternion[0]) ||
            isnan(orientation->quaternion[1]) ||
            isnan(orientation->quaternion[2]) ||
            isnan(orientation->quaternion[3])) {
            psmove_DEBUG("Orientation is NaN!");
            orientation->quaternion[0] = q0;
            orientation->quaternion[1] = q1;
            orientation->quaternion[2] = q2;
            orientation->quaternion[3] = q3;
        }
    }
}

void
psmove_orientation_get_quaternion(PSMoveOrientation *orientation,
        float *q0, float *q1, float *q2, float *q3)
{
    psmove_return_if_fail(orientation != NULL);

    /* first factor (reset quaternion) */
    float a_s = orientation->reset_quaternion[0];
    float a_x = orientation->reset_quaternion[1];
    float a_y = orientation->reset_quaternion[2];
    float a_z = orientation->reset_quaternion[3];

    /* second factor (quaternion) */
    float b_s = orientation->quaternion[0];
    float b_x = orientation->quaternion[1];
    float b_y = orientation->quaternion[2];
    float b_z = orientation->quaternion[3];

    /**
     * Quaternion multiplication:
     * http://lxr.kde.org/source/qt/src/gui/math3d/qquaternion.h#198
     **/
    float ww = (a_z + a_x) * (b_x + b_y);
    float yy = (a_s - a_y) * (b_s + b_z);
    float zz = (a_s + a_y) * (b_s - b_z);
    float xx = ww + yy + zz;
    float qq = .5f * (xx + (a_z - a_x) * (b_x - b_y));

    /* Result */
    float r_s = qq - ww + (a_z - a_y) * (b_y - b_z);
    float r_x = qq - xx + (a_x + a_s) * (b_x + b_s);
    float r_y = qq - yy + (a_s - a_x) * (b_y + b_z);
    float r_z = qq - zz + (a_z + a_y) * (b_s - b_x);

    if (q0) {
        *q0 = r_s;
    }

    if (q1) {
        *q1 = r_x;
    }

    if (q2) {
        *q2 = r_y;
    }

    if (q3) {
        *q3 = r_z;
    }
}

void
psmove_orientation_reset_quaternion(PSMoveOrientation *orientation)
{
    psmove_return_if_fail(orientation != NULL);
    float q0 = orientation->quaternion[0];
    float q1 = orientation->quaternion[1];
    float q2 = orientation->quaternion[2];
    float q3 = orientation->quaternion[3];

    /**
     * Normalize and conjugate in one step:
     *  - Normalize via the length
     *  - Conjugate using (scalar, x, y, z) -> (scalar, -x, -y, -z)
     **/
    double length = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    orientation->reset_quaternion[0] = (float)(q0 / length);
	orientation->reset_quaternion[1] = (float)(-q1 / length);
	orientation->reset_quaternion[2] = (float)(-q2 / length);
	orientation->reset_quaternion[3] = (float)(-q3 / length);
}

void
psmove_orientation_free(PSMoveOrientation *orientation)
{
    psmove_return_if_fail(orientation != NULL);

    free(orientation);
}

