// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is the file that contains the fusion routines.  It is STRONGLY RECOMMENDED
// that the casual developer NOT TOUCH THIS FILE.  The mathematics behind this file
// is extremely complex, and it will be very easy (almost inevitable) that you screw
// it up.
//
#include "kalman.hpp"

#include <cmath>

#include "build.hpp"
#include "matrix.hpp"
#include "orientation.hpp"
#include "tasks.hpp"

namespace filter::kalman {
    using filter::matrix::f3x3matrixAeqI;
    using filter::matrix::fmatrixAeqInvA;
    using filter::orientation::f3DOFTiltAndroid;
    using filter::orientation::f3DOFTiltNED;
    using filter::orientation::f3DOFTiltWin8;
    using filter::orientation::fAndroidAnglesDegFromRotationMatrix;
    using filter::orientation::fNEDAnglesDegFromRotationMatrix;
    using filter::orientation::fqAeq1;
    using filter::orientation::fqAeqNormqA;
    using filter::orientation::fQuaternionFromRotationMatrix;
    using filter::orientation::fQuaternionFromRotationVectorDeg;
    using filter::orientation::fRotationMatrixFromQuaternion;
    using filter::orientation::fRotationVectorDegFromQuaternion;
    using filter::orientation::fWin8AnglesDegFromRotationMatrix;
    using filter::orientation::qAeqBxC;


    // function initalizes the 6DOF accel + gyro Kalman filter algorithm
    void fInit_6DOF_GY_KALMAN(struct ::filter::tasks::SV_6DOF_GY_KALMAN& pthisSV) {
        int i = 0;
        int j = 0;  // loop counters

        // reset the flag denoting that a first 6DOpthisSVtion lock has been achieved
        pthisSV.iFirstOrientationLock = false;

        // initialize the fixed entries in the measurement matrix C
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 9; j++) {
                pthisSV.C[i][j] = 0.0F;
            }
        }
        pthisSV.C[0][6] = pthisSV.C[1][7] = pthisSV.C[2][8] = 1.0F;

        // zero a posteriori orientation, error vector xe+ (thetae+, be+, ae+) and b+
        f3x3matrixAeqI(pthisSV.posterior_rot_matrix);
        fqAeq1(pthisSV.posterior_orientation_quat);
        for (i = 0; i <= 2; i++) {
            pthisSV.orientation_error_deg[i] = pthisSV.gyro_offset_error[i] = pthisSV.linear_accel_error_g1[i] =
                pthisSV.gyro_offset[i]                                      = 0.0F;
        }

        // initialize noise variance for Qv and Qw matrix updates
        pthisSV.accel_term_from_Qv =
            FQVA_6DOF_GY_KALMAN + FQWA_6DOF_GY_KALMAN
            + FDEGTORAD * FDEGTORAD * pthisSV.DELTA_T_SQUARED * (FQWB_6DOF_GY_KALMAN + FQVG_6DOF_GY_KALMAN);

        // initialize the 6x6 noise covariance matrix Qw of the a priori error vector xe-
        // Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K * C) * Qw  and Qw updated from P+
        // zero the matrix Qw
        for (i = 0; i < 9; i++) {
            for (j = 0; j < 9; j++) {
                pthisSV.Qw[i][j] = 0.0F;
            }
        }
        // loop over non-zero values
        for (i = 0; i < 3; i++) {
            // theta_e * theta_e terms
            pthisSV.Qw[i][i] = FQWINITTHTH_6DOF_GY_KALMAN;
            // b_e * b_e terms
            pthisSV.Qw[i + 3][i + 3] = FQWINITBB_6DOF_GY_KALMAN;
            // th_e * b_e terms
            pthisSV.Qw[i][i + 3] = pthisSV.Qw[i + 3][i] = FQWINITTHB_6DOF_GY_KALMAN;
            // a_e * a_e terms
            pthisSV.Qw[i + 6][i + 6] = FQWINITAA_6DOF_GY_KALMAN;
        }

        // clear the reset flag
        pthisSV.resetflag = false;
    }  // end fInit_6DOF_GY_KALMAN

    // 6DOF accel + gyro Kalman filter algorithm
    Eigen::Quaternion<double> fRun_6DOF_GY_KALMAN(struct ::filter::tasks::SV_6DOF_GY_KALMAN& pthisSV,
                                                  const Eigen::Matrix<double, 3, 1>& accel_reading,
                                                  const Eigen::Matrix<double, 3, 1>& gyro_reading,
                                                  const int& ithisCoordSystem,
                                                  const bool& resetflag) {
        double accel_reading_arr[3] = {accel_reading.x(), accel_reading.y(), accel_reading.z()};
        double gyro_reading_arr[3]  = {gyro_reading.x(), gyro_reading.y(), gyro_reading.z()};

        // local arrays and scalars
        double rvec[3];         // rotation vector
        double ftmpA9x3[9][3];  // scratch array

        // assorted array pointers
        double* pP_pluskj   = nullptr;
        double* pP_plusij   = nullptr;
        double* pKij        = nullptr;
        double* pKik        = nullptr;
        double* pftmpA9x3ik = nullptr;
        double* pftmpA9x3ij = nullptr;
        double* pftmpA9x3kj = nullptr;
        double* pQwij       = nullptr;
        double* pQwik       = nullptr;
        double* pQwkj       = nullptr;
        double* pCik        = nullptr;
        double* pCjk        = nullptr;

        int i = 0;
        int j = 0;
        int k = 0;  // loop counters

        // working arrays for 3x3 matrix inversion
        double* pfRows[3];
        int iColInd[3];
        int iRowInd[3];
        int iPivot[3];

        // do a reset and return if requested
        if (resetflag) {
            fInit_6DOF_GY_KALMAN(pthisSV);
            return fRun_6DOF_GY_KALMAN(pthisSV, accel_reading, gyro_reading, ithisCoordSystem, false);
        }

        // do a once-only orientation lock to accelerometer tilt
        if (!pthisSV.iFirstOrientationLock) {
            // get the 3DOF orientation matrix and initial inclination angle
            if (ithisCoordSystem == NED) {
                // call NED tilt function
                // f3DOFTiltNED(pthisSV.posterior_rot_matrix, pthisAccel.fGpFast);
                f3DOFTiltNED(pthisSV.posterior_rot_matrix, accel_reading_arr);
            }
            else if (ithisCoordSystem == ANDROID) {
                // call Android tilt function
                // f3DOFTiltAndroid(pthisSV.posterior_rot_matrix, pthisAccel.fGpFast);
                f3DOFTiltAndroid(pthisSV.posterior_rot_matrix, accel_reading_arr);
            }
            else {
                // call Windows 8 tilt function
                // f3DOFTiltWin8(pthisSV.posterior_rot_matrix, pthisAccel.fGpFast);
                f3DOFTiltWin8(pthisSV.posterior_rot_matrix, accel_reading_arr);
            }

            // get the orientation quaternion from the orientation matrix
            fQuaternionFromRotationMatrix(pthisSV.posterior_rot_matrix, pthisSV.posterior_orientation_quat);

            // set the orientation lock flag so this initial alignment is only performed once
            pthisSV.iFirstOrientationLock = true;
        }

        // *********************************************************************************
        // calculate a priori rotation matrix
        // *********************************************************************************

        // compute the angular velocity from the average of the high frequency gyro readings.
        // omega[k] = yG[k] - b-[k] = yG[k] - b+[k-1] (deg/s)
        // this involves a small angle approximation but the resulting angular velocity is
        // only computed for transmission over bluetooth and not used for orientation determination.
        for (i = 0; i <= 2; i++) {
            // pthisSV.angular_velocity_vec[i] = pthisGyro.fYp[i] - pthisSV.gyro_offset[i];
            pthisSV.angular_velocity_vec[i] = gyro_reading_arr[i] - pthisSV.gyro_offset[i];
        }

        // initialize the a priori orientation quaternion to the a posteriori orientation estimate
        pthisSV.prior_rotation_quat = pthisSV.posterior_orientation_quat;
        // get the a priori rotation matrix from the a priori quaternion
        fRotationMatrixFromQuaternion(pthisSV.prior_rotation_matrix, pthisSV.prior_rotation_quat);

        // *********************************************************************************
        // calculate a priori gyro and accelerometer estimates of the gravity vector
        // and the error between the two
        // *********************************************************************************

        // compute the a priori **gyro** estimate of the gravitational vector (g, sensor frame)
        // using an absolute rotation of the global frame gravity vector (with magnitude 1g)
        for (i = 0; i <= 2; i++) {
            if (ithisCoordSystem == NED) {
                // NED gravity is along positive z axis
                pthisSV.gyro_gravity_g[i] = pthisSV.prior_rotation_matrix[i][2];
            }
            else {
                // Android and Win8 gravity are along negative z axis
                pthisSV.gyro_gravity_g[i] = -pthisSV.prior_rotation_matrix[i][2];
            }

            // compute a priori acceleration (a-) (g, sensor frame) from a posteriori estimate (g, sensor frame)
            pthisSV.linear_accel_g2[i] = FCA_6DOF_GY_KALMAN * pthisSV.linear_accel_g1[i];

            // compute the a priori gravity error vector (accelerometer minus gyro estimates) (g, sensor frame)
            if ((ithisCoordSystem == NED) || (ithisCoordSystem == WIN8)) {
                // NED and Windows 8 have positive sign for gravity: y = g - a and g = y + a
                // pthisSV.gravity_accel_minus_gravity_gyro[i] = pthisAccel.fGpFast[i] + pthisSV.linear_accel_g2[i] -
                // pthisSV.gyro_gravity_g[i];
                pthisSV.gravity_accel_minus_gravity_gyro[i] =
                    accel_reading_arr[i] + pthisSV.linear_accel_g2[i] - pthisSV.gyro_gravity_g[i];
            }
            else {
                // Android has negative sign for gravity: y = a - g, g = -y + a
                // pthisSV.gravity_accel_minus_gravity_gyro[i] = -pthisAccel.fGpFast[i] + pthisSV.linear_accel_g2[i]
                // - pthisSV.gyro_gravity_g[i];
                pthisSV.gravity_accel_minus_gravity_gyro[i] =
                    -accel_reading_arr[i] + pthisSV.linear_accel_g2[i] - pthisSV.gyro_gravity_g[i];
            }
        }

        // *********************************************************************************
        // update variable elements of measurement matrix C
        // *********************************************************************************

        // update measurement matrix C (3x9) with -alpha(g-)x from gyro (g, sensor frame)
        pthisSV.C[0][1] = FDEGTORAD * pthisSV.gyro_gravity_g[2];
        pthisSV.C[0][2] = -FDEGTORAD * pthisSV.gyro_gravity_g[1];
        pthisSV.C[1][2] = FDEGTORAD * pthisSV.gyro_gravity_g[0];
        pthisSV.C[1][0] = -pthisSV.C[0][1];
        pthisSV.C[2][0] = -pthisSV.C[0][2];
        pthisSV.C[2][1] = -pthisSV.C[1][2];
        pthisSV.C[0][4] = -pthisSV.DELTA_T * pthisSV.C[0][1];
        pthisSV.C[0][5] = -pthisSV.DELTA_T * pthisSV.C[0][2];
        pthisSV.C[1][5] = -pthisSV.DELTA_T * pthisSV.C[1][2];
        pthisSV.C[1][3] = -pthisSV.C[0][4];
        pthisSV.C[2][3] = -pthisSV.C[0][5];
        pthisSV.C[2][4] = -pthisSV.C[1][5];

        // *********************************************************************************
        // calculate the Kalman gain matrix K (9x3)
        // K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
        // Qw is used as a proxy for P- throughout the code
        // P+ is used here as a working array to reduce RAM usage and is re-computed later
        // *********************************************************************************

        // set ftmpA9x3 = P- * C^T = Qw * C^T where Qw and C are both sparse
        // C also has a significant number of +1 and -1 entries
        // ftmpA9x3 is also sparse but not symmetric
        for (i = 0; i < 9; i++)  // loop over rows of ftmpA9x3
        {
            // initialize pftmpA9x3ij for current i, j=0
            pftmpA9x3ij = ftmpA9x3[i];

            for (j = 0; j < 3; j++)  // loop over columns of ftmpA9x3
            {
                // zero ftmpA9x3[i][j]
                *pftmpA9x3ij = 0.0F;

                // initialize pCjk for current j, k=0
                pCjk = pthisSV.C[j];

                // initialize pQwik for current i, k=0
                pQwik = pthisSV.Qw[i];

                // sum matrix products over inner loop over k
                for (k = 0; k < 9; k++) {
                    if ((*pQwik != 0.0F) && (*pCjk != 0.0F)) {
                        if (*pCjk == 1.0F) {
                            *pftmpA9x3ij += *pQwik;
                        }
                        else if (*pCjk == -1.0F) {
                            *pftmpA9x3ij -= *pQwik;
                        }
                        else {
                            *pftmpA9x3ij += *pQwik * *pCjk;
                        }
                    }

                    // increment pCjk and pQwik for next iteration of k
                    pCjk++;
                    pQwik++;

                }  // end of loop over k

                // increment pftmpA9x3ij for next iteration of j
                pftmpA9x3ij++;

            }  // end of loop over j
        }      // end of loop over i

        // set symmetric P+ (3x3 scratch sub-matrix) to C * P- * C^T + Qv
        // = C * (Qw * C^T) + Qv = C * ftmpA9x3 + Qv
        // both C and ftmpA9x3 are sparse but not symmetric
        for (i = 0; i < 3; i++)  // loop over rows of P+
        {
            // initialize pP_plusij for current i, j=i
            pP_plusij = pthisSV.P_plus[i] + i;

            for (j = i; j < 3; j++)  // loop over above diagonal columns of P+
            {
                // zero P+[i][j]
                *pP_plusij = 0.0F;

                // initialize pCik for current i, k=0
                pCik = pthisSV.C[i];

                // initialize pftmpA9x3kj for current j, k=0
                pftmpA9x3kj = *ftmpA9x3 + j;

                // sum matrix products over inner loop over k
                for (k = 0; k < 9; k++) {
                    if ((*pCik != 0.0F) && (*pftmpA9x3kj != 0.0F)) {
                        if (*pCik == 1.0F) {
                            *pP_plusij += *pftmpA9x3kj;
                        }
                        else if (*pCik == -1.0F) {
                            *pP_plusij -= *pftmpA9x3kj;
                        }
                        else {
                            *pP_plusij += *pCik * *pftmpA9x3kj;
                        }
                    }

                    // update pCik and pftmpA9x3kj for next iteration of k
                    pCik++;
                    pftmpA9x3kj += 3;

                }  // end of loop over k

                // increment pP_plusij for next iteration of j
                pP_plusij++;

            }  // end of loop over j
        }      // end of loop over i

        // add in noise covariance terms to the diagonal
        pthisSV.P_plus[0][0] += pthisSV.accel_term_from_Qv;
        pthisSV.P_plus[1][1] += pthisSV.accel_term_from_Qv;
        pthisSV.P_plus[2][2] += pthisSV.accel_term_from_Qv;

        // copy above diagonal terms of P+ (3x3 scratch sub-matrix) to below diagonal terms
        pthisSV.P_plus[1][0] = pthisSV.P_plus[0][1];
        pthisSV.P_plus[2][0] = pthisSV.P_plus[0][2];
        pthisSV.P_plus[2][1] = pthisSV.P_plus[1][2];

        // calculate inverse of P+ (3x3 scratch sub-matrix) = inv(C * P- * C^T + Qv) = inv(C * Qw * C^T + Qv)
        for (i = 0; i < 3; i++) {
            pfRows[i] = pthisSV.P_plus[i];
        }
        fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

        // set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
        // = ftmpA9x3 * P+ (3x3 sub-matrix)
        // ftmpA9x3 = Qw * C^T is sparse but P+ (3x3 sub-matrix) is not
        // K is not symmetric because C is not symmetric
        for (i = 0; i < 9; i++)  // loop over rows of K9x3
        {
            // initialize pKij for i, j=0
            pKij = pthisSV.K[i];

            for (j = 0; j < 3; j++)  // loop over columns of K9x3
            {
                // zero the matrix element K[i][j]
                *pKij = 0.0F;

                // initialize pftmpA9x3ik for i, k=0
                pftmpA9x3ik = ftmpA9x3[i];

                // initialize pP_pluskj for j, k=0
                pP_pluskj = *pthisSV.P_plus + j;

                // sum matrix products over inner loop over k
                for (k = 0; k < 3; k++) {
                    if (*pftmpA9x3ik != 0.0F) {
                        *pKij += *pftmpA9x3ik * *pP_pluskj;
                    }

                    // increment pftmpA9x3ik and pP_pluskj for next iteration of k
                    pftmpA9x3ik++;
                    pP_pluskj += 9;

                }  // end of loop over k

                // increment pKij for the next iteration of j
                pKij++;

            }  // end of loop over j
        }      // end of loop over i

        // *********************************************************************************
        // calculate a posteriori error estimate: xe+ = K * ze-
        // *********************************************************************************

        // update the a posteriori state vector
        for (i = 0; i <= 2; i++) {
            // zero a posteriori error terms
            pthisSV.orientation_error_deg[i] = pthisSV.gyro_offset_error[i] = pthisSV.linear_accel_error_g1[i] = 0.0F;

            // accumulate the error vector terms K * ze-
            for (k = 0; k < 3; k++) {
                pthisSV.orientation_error_deg[i] += pthisSV.K[i][k] * pthisSV.gravity_accel_minus_gravity_gyro[k];
                pthisSV.gyro_offset_error[i] += pthisSV.K[i + 3][k] * pthisSV.gravity_accel_minus_gravity_gyro[k];
                pthisSV.linear_accel_error_g1[i] += pthisSV.K[i + 6][k] * pthisSV.gravity_accel_minus_gravity_gyro[k];
            }
        }

        // *********************************************************************************
        // apply the a posteriori error corrections to the a posteriori state vector
        // *********************************************************************************

        // get the a posteriori delta quaternion
        fQuaternionFromRotationVectorDeg(pthisSV.delta_quaternion, pthisSV.orientation_error_deg, -1.0);

        // compute the a posteriori orientation quaternion posterior_orientation_quat = prior_rotation_quat *
        // Deltaq(-thetae+) the resulting quaternion may have negative scalar component q0
        qAeqBxC(pthisSV.posterior_orientation_quat, pthisSV.prior_rotation_quat, pthisSV.delta_quaternion);

        // normalize the a posteriori orientation quaternion to stop error propagation
        // the renormalization function ensures that the scalar component q0 is non-negative
        fqAeqNormqA(pthisSV.posterior_orientation_quat);

        // compute the a posteriori rotation matrix from the a posteriori quaternion
        fRotationMatrixFromQuaternion(pthisSV.posterior_rot_matrix, pthisSV.posterior_orientation_quat);

        // compute the rotation vector from the a posteriori quaternion
        fRotationVectorDegFromQuaternion(pthisSV.posterior_orientation_quat, pthisSV.rot_vec);

        // update the a posteriori gyro offset vector b+ and linear acceleration vector a+ (sensor frame)
        for (i = 0; i <= 2; i++) {
            // b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
            pthisSV.gyro_offset[i] -= pthisSV.gyro_offset_error[i];
            // a+ = a- - ae+ (g, sensor frame)
            pthisSV.linear_accel_g1[i] = pthisSV.linear_accel_g2[i] - pthisSV.linear_accel_error_g1[i];
        }

        // *********************************************************************************
        // compute the a posteriori Euler angles from the orientation matrix
        // *********************************************************************************

        if (ithisCoordSystem == NED) {
            // calculate the NED Euler angles
            fNEDAnglesDegFromRotationMatrix(pthisSV.posterior_rot_matrix,
                                            pthisSV.roll_deg,
                                            pthisSV.pitch_deg,
                                            pthisSV.yaw_deg,
                                            pthisSV.compass_deg,
                                            pthisSV.tilt_deg);
        }
        else if (ithisCoordSystem == ANDROID) {
            // calculate the Android Euler angles
            fAndroidAnglesDegFromRotationMatrix(pthisSV.posterior_rot_matrix,
                                                pthisSV.roll_deg,
                                                pthisSV.pitch_deg,
                                                pthisSV.yaw_deg,
                                                pthisSV.compass_deg,
                                                pthisSV.tilt_deg);
        }
        else {
            // calculate Win8 Euler angles
            fWin8AnglesDegFromRotationMatrix(pthisSV.posterior_rot_matrix,
                                             pthisSV.roll_deg,
                                             pthisSV.pitch_deg,
                                             pthisSV.yaw_deg,
                                             pthisSV.compass_deg,
                                             pthisSV.tilt_deg);
        }

        // ***********************************************************************************
        // calculate (symmetric) a posteriori error covariance matrix P+
        // P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
        // both Qw and P+ are used as working arrays in this section
        // at the end of this section, P+ is valid but Qw is over-written
        // ***********************************************************************************

        // set P+ (3x9 scratch sub-matrix) to the product C (3x9) * Qw (9x9)
        // where both C and Qw are sparse and C has a significant number of +1 entries
        // the resulting matrix is sparse but not symmetric
        for (i = 0; i < 3; i++)  // loop over the rows of P+
        {
            // initialize pP_plusij for current i, j=0
            pP_plusij = pthisSV.P_plus[i];

            for (j = 0; j < 9; j++)  // loop over the columns of P+
            {
                // zero P+[i][j]
                *pP_plusij = 0.0F;

                // initialize pCik for current i, k=0
                pCik = pthisSV.C[i];

                // initialize pQwkj for current j, k=0
                pQwkj = &pthisSV.Qw[0][j];

                // sum matrix products over inner loop over k
                for (k = 0; k < 9; k++) {
                    if ((*pCik != 0.0F) && (*pQwkj != 0.0F)) {
                        if (*pCik == 1.0F) {
                            *pP_plusij += *pQwkj;
                        }
                        else if (*pCik == -1.0F) {
                            *pP_plusij -= *pQwkj;
                        }
                        else {
                            *pP_plusij += *pCik * *pQwkj;
                        }
                    }

                    // update pCik and pQwkj for next iteration of k
                    pCik++;
                    pQwkj += 9;

                }  // end of loop over k

                // increment pP_plusij for next iteration of j
                pP_plusij++;

            }  // end of loop over j
        }      // end of loop over i

        // compute P+ = (I9 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (3x9 sub-matrix)
        // storing result P+ in Qw and over-writing Qw which is OK since Qw is later computed from P+
        // where working array P+ (6x12 sub-matrix) is sparse but K is not sparse
        // only on and above diagonal terms of P+ are computed since P+ is symmetric
        for (i = 0; i < 9; i++) {
            // initialize pQwij for i, j=i
            pQwij = pthisSV.Qw[i] + i;

            for (j = i; j < 9; j++) {
                // initialize pKik for i, k=0
                pKik = pthisSV.K[i];

                // initialize pP_pluskj for j, k=0
                pP_pluskj = *pthisSV.P_plus + j;

                // compute on and above diagonal matrix entry
                for (k = 0; k < 3; k++) {
                    // check for non-zero values since P+ (3x9 scratch sub-matrix) is sparse
                    if (*pP_pluskj != 0.0F) {
                        *pQwij -= *pKik * *pP_pluskj;
                    }

                    // increment pKik and pP_pluskj for next iteration of k
                    pKik++;
                    pP_pluskj += 9;

                }  // end of loop over k

                // increment pQwij for next iteration of j
                pQwij++;

            }  // end of loop over j
        }      // end of loop over i

        // Qw now holds the on and above diagonal elements of P+ (9x9)
        // so perform a simple copy to the all elements of P+
        // after execution of this code P+ is valid but Qw remains invalid
        for (i = 0; i < 9; i++) {
            // initialize pP_plusij and pQwij for i, j=i
            pP_plusij = pthisSV.P_plus[i] + i;
            pQwij     = pthisSV.Qw[i] + i;

            // copy the on-diagonal elements and increment pointers to enter loop at j=i+1
            *(pP_plusij++) = *(pQwij++);

            // loop over above diagonal columns j copying to below-diagonal elements
            for (j = i + 1; j < 9; j++) {
                *(pP_plusij++) = pthisSV.P_plus[j][i] = *(pQwij++);
            }
        }

        // *********************************************************************************
        // re-create the noise covariance matrix Qw=fn(P+) for the next iteration
        // using the elements of P+ which are now valid
        // Qw was over-written earlier but is here recomputed (all elements)
        // *********************************************************************************

        // zero the matrix Qw (9x9)
        for (i = 0; i < 9; i++) {
            for (j = 0; j < 9; j++) {
                pthisSV.Qw[i][j] = 0.0F;
            }
        }

        // update the covariance matrix components
        for (i = 0; i < 3; i++) {
            // Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 * (Q[b+b+] + (Qwb + QvG) * I)
            pthisSV.Qw[i][i] = pthisSV.P_plus[i][i]
                               + pthisSV.DELTA_T_SQUARED * (pthisSV.P_plus[i + 3][i + 3] + pthisSV.FQWB_plus_FQVG);

            // Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
            pthisSV.Qw[i + 3][i + 3] = pthisSV.P_plus[i + 3][i + 3] + FQWB_6DOF_GY_KALMAN;

            // Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) = -deltat * Qw[b-b-]
            pthisSV.Qw[i][i + 3] = pthisSV.Qw[i + 3][i] = -pthisSV.DELTA_T * pthisSV.Qw[i + 3][i + 3];

            // Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
            pthisSV.Qw[i + 6][i + 6] = pthisSV.FCA_squared * pthisSV.P_plus[i + 6][i + 6] + FQWA_6DOF_GY_KALMAN;
        }
        return pthisSV.posterior_orientation_quat;
    }  // end fRun_6DOF_GY_KALMAN
}  // namespace filter::kalman