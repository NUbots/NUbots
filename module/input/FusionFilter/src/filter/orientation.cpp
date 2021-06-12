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
// This file contains functions designed to operate on, or compute, orientations.
// These may be in rotation matrix form, quaternion form, or Euler angles.
// It also includes functions designed to operate with specify reference frames
// (Android, Windows 8, NED).
//
#include "orientation.hpp"

#include <cmath>

#include "build.hpp"
#include "matrix.hpp"
#include "utilities.hpp"
// #include "stdio.h"
// #include "stdlib.h"
// #include "string.h"
// #include "time.h"

// compile time constants that are private to this file
#define SMALLQ0      0.01F   // limit of quaternion scalar component requiring special algorithm
#define CORRUPTQUAT  0.001F  // threshold for deciding rotation quaternion is corrupt
#define SMALLMODULUS 0.01F   // limit where rounding errors may appear

namespace filter::orientation {

    using filter::matrix::f3x3matrixAeqI;
    using filter::matrix::f3x3matrixAeqScalar;
    using filter::utilities::acos_deg;
    using filter::utilities::asin_deg;
    using filter::utilities::atan2_deg;
    using filter::utilities::atan_deg;

    // Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
    void f3DOFTiltNED(double fR[][3], double fGp[]) {
        // the NED self-consistency twist occurs at 90 deg pitch

        // local variables
        int i                = 0;    // counter
        double fmodGxyz      = 0.0;  // modulus of the x, y, z accelerometer readings
        double fmodGyz       = 0.0;  // modulus of the y, z accelerometer readings
        double frecipmodGxyz = 0.0;  // reciprocal of modulus
        double ftmp          = 0.0;  // scratch variable

        // compute the accelerometer squared magnitudes
        fmodGyz  = fGp[1] * fGp[1] + fGp[2] * fGp[2];
        fmodGxyz = fmodGyz + fGp[0] * fGp[0];

        // check for freefall special case where no solution is possible
        if (fmodGxyz == 0.0F) {
            f3x3matrixAeqI(fR);
            return;
        }

        // check for vertical up or down gimbal lock case
        if (fmodGyz == 0.0F) {
            f3x3matrixAeqScalar(fR, 0.0F);
            fR[1][1] = 1.0F;
            if (fGp[0] >= 0.0F) {
                fR[0][2] = 1.0F;
                fR[2][0] = -1.0F;
            }
            else {
                fR[0][2] = -1.0F;
                fR[2][0] = 1.0F;
            }
            return;
        }

        // compute moduli for the general case
        fmodGyz       = std::sqrt(fmodGyz);
        fmodGxyz      = std::sqrt(fmodGxyz);
        frecipmodGxyz = 1.0F / fmodGxyz;
        ftmp          = fmodGxyz / fmodGyz;

        // normalize the accelerometer reading into the z column
        for (i = 0; i <= 2; i++) {
            fR[i][2] = fGp[i] * frecipmodGxyz;
        }

        // construct x column of orientation matrix
        fR[0][0] = fmodGyz * frecipmodGxyz;
        fR[1][0] = -fR[0][2] * fR[1][2] * ftmp;
        fR[2][0] = -fR[0][2] * fR[2][2] * ftmp;

        // // construct y column of orientation matrix
        fR[0][1] = 0.0F;
        fR[1][1] = fR[2][2] * ftmp;
        fR[2][1] = -fR[1][2] * ftmp;
    }

    // Android accelerometer 3DOF tilt function computing rotation matrix fR
    void f3DOFTiltAndroid(double fR[][3], double fGp[]) {
        // the Android tilt matrix is mathematically identical to the NED tilt matrix
        // the Android self-consistency twist occurs at 90 deg roll
        f3DOFTiltNED(fR, fGp);
    }

    // Windows 8 accelerometer 3DOF tilt function computing rotation matrix fR
    void f3DOFTiltWin8(double fR[][3], double fGp[]) {
        // the Win8 self-consistency twist occurs at 90 deg roll

        // local variables
        double fmodGxyz      = 0.0;  // modulus of the x, y, z accelerometer readings
        double fmodGxz       = 0.0;  // modulus of the x, z accelerometer readings
        double frecipmodGxyz = 0.0;  // reciprocal of modulus
        double ftmp          = 0.0;  // scratch variable
        int i                = 0;    // counter

        // compute the accelerometer squared magnitudes
        fmodGxz  = fGp[0] * fGp[0] + fGp[2] * fGp[2];
        fmodGxyz = fmodGxz + fGp[1] * fGp[1];

        // check for freefall special case where no solution is possible
        if (fmodGxyz == 0.0F) {
            f3x3matrixAeqI(fR);
            return;
        }

        // check for vertical up or down gimbal lock case
        if (fmodGxz == 0.0F) {
            f3x3matrixAeqScalar(fR, 0.0F);
            fR[0][0] = 1.0F;
            if (fGp[1] >= 0.0F) {
                fR[1][2] = -1.0F;
                fR[2][1] = 1.0F;
            }
            else {
                fR[1][2] = 1.0F;
                fR[2][1] = -1.0F;
            }
            return;
        }

        // compute moduli for the general case
        fmodGxz       = std::sqrt(fmodGxz);
        fmodGxyz      = std::sqrt(fmodGxyz);
        frecipmodGxyz = 1.0 / fmodGxyz;
        ftmp          = fmodGxyz / fmodGxz;
        if (fGp[2] < 0.0) {
            ftmp = -ftmp;
        }

        // normalize the negated accelerometer reading into the z column
        for (i = 0; i <= 2; i++) {
            fR[i][2] = -fGp[i] * frecipmodGxyz;
        }

        // construct x column of orientation matrix
        fR[0][0] = -fR[2][2] * ftmp;
        fR[1][0] = 0.0;
        fR[2][0] = fR[0][2] * ftmp;

        // // construct y column of orientation matrix
        fR[0][1] = fR[0][2] * fR[1][2] * ftmp;
        fR[1][1] = -fmodGxz * frecipmodGxyz;
        if (fGp[2] < 0.0F) {
            fR[1][1] = -fR[1][1];
        }
        fR[2][1] = fR[1][2] * fR[2][2] * ftmp;
    }

    // Aerospace NED magnetometer 3DOF flat eCompass function computing rotation matrix fR
    void f3DOFMagnetometerMatrixNED(double fR[][3], double fBc[]) {
        // local variables
        double fmodBxy = 0.0;  // modulus of the x, y magnetometer readings

        // compute the magnitude of the horizontal (x and y) magnetometer reading
        fmodBxy = std::sqrt(fBc[0] * fBc[0] + fBc[1] * fBc[1]);

        // check for zero field special case where no solution is possible
        if (fmodBxy == 0.0F) {
            f3x3matrixAeqI(fR);
            return;
        }

        // define the fixed entries in the z row and column
        fR[2][0] = fR[2][1] = fR[0][2] = fR[1][2] = 0.0F;
        fR[2][2]                                  = 1.0F;

        // define the remaining entries
        fR[0][0] = fR[1][1] = fBc[0] / fmodBxy;
        fR[1][0]            = fBc[1] / fmodBxy;
        fR[0][1]            = -fR[1][0];
    }

    // Android magnetometer 3DOF flat eCompass function computing rotation matrix fR
    void f3DOFMagnetometerMatrixAndroid(double fR[][3], double fBc[]) {
        // local variables
        double fmodBxy = 0.0;  // modulus of the x, y magnetometer readings

        // compute the magnitude of the horizontal (x and y) magnetometer reading
        fmodBxy = std::sqrt(fBc[0] * fBc[0] + fBc[1] * fBc[1]);

        // check for zero field special case where no solution is possible
        if (fmodBxy == 0.0F) {
            f3x3matrixAeqI(fR);
            return;
        }

        // define the fixed entries in the z row and column
        fR[2][0] = fR[2][1] = fR[0][2] = fR[1][2] = 0.0F;
        fR[2][2]                                  = 1.0F;

        // define the remaining entries
        fR[0][0] = fR[1][1] = fBc[1] / fmodBxy;
        fR[0][1]            = fBc[0] / fmodBxy;
        fR[1][0]            = -fR[0][1];
    }

    // Windows 8 magnetometer 3DOF flat eCompass function computing rotation matrix fR
    void f3DOFMagnetometerMatrixWin8(double fR[][3], double fBc[]) {
        // call the Android function since it is identical to the Windows 8 matrix
        f3DOFMagnetometerMatrixAndroid(fR, fBc);
    }

    // extract the NED angles in degrees from the NED rotation matrix
    void fNEDAnglesDegFromRotationMatrix(double R[][3],
                                         double& pfPhiDeg,
                                         double& pfTheDeg,
                                         double& pfPsiDeg,
                                         double& pfRhoDeg,
                                         double& pfChiDeg) {
        // calculate the pitch angle -90.0 <= Theta <= 90.0 deg
        pfTheDeg = asin_deg(-R[0][2]);

        // calculate the roll angle range -180.0 <= Phi < 180.0 deg
        pfPhiDeg = atan2_deg(R[1][2], R[2][2]);

        // map +180 roll onto the functionally equivalent -180 deg roll
        if (pfPhiDeg == 180.0) {
            pfPhiDeg = -180.0;
        }

        // calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
        if (pfTheDeg == 90.0F) {
            // vertical upwards gimbal lock case
            pfPsiDeg = atan2_deg(R[2][1], R[1][1]) + pfPhiDeg;
        }
        else if (pfTheDeg == -90.0F) {
            // vertical downwards gimbal lock case
            pfPsiDeg = atan2_deg(-R[2][1], R[1][1]) - pfPhiDeg;
        }
        else {
            // general case
            pfPsiDeg = atan2_deg(R[0][1], R[0][0]);
        }

        // map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
        if (pfPsiDeg < 0.0F) {
            pfPsiDeg += 360.0F;
        }

        // check for rounding errors mapping small negative angle to 360 deg
        if (pfPsiDeg >= 360.0F) {
            pfPsiDeg = 0.0F;
        }

        // for NED, the compass heading Rho equals the yaw angle Psi
        pfRhoDeg = pfPsiDeg;

        // calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
        pfChiDeg = acos_deg(R[2][2]);
    }

    // extract the Android angles in degrees from the Android rotation matrix
    void fAndroidAnglesDegFromRotationMatrix(double R[][3],
                                             double& pfPhiDeg,
                                             double& pfTheDeg,
                                             double& pfPsiDeg,
                                             double& pfRhoDeg,
                                             double& pfChiDeg) {
        // calculate the roll angle -90.0 <= Phi <= 90.0 deg
        pfPhiDeg = asin_deg(R[0][2]);

        // calculate the pitch angle -180.0 <= The < 180.0 deg
        pfTheDeg = atan2_deg(-R[1][2], R[2][2]);

        // map +180 pitch onto the functionally equivalent -180 deg pitch
        if (pfTheDeg == 180.0F) {
            pfTheDeg = -180.0F;
        }

        // calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
        if (pfPhiDeg == 90.0F) {
            // vertical downwards gimbal lock case
            pfPsiDeg = atan2_deg(R[1][0], R[1][1]) - pfTheDeg;
        }
        else if (pfPhiDeg == -90.0F) {
            // vertical upwards gimbal lock case
            pfPsiDeg = atan2_deg(R[1][0], R[1][1]) + pfTheDeg;
        }
        else {
            // // general case
            pfPsiDeg = atan2_deg(-R[0][1], R[0][0]);
        }

        // map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
        if (pfPsiDeg < 0.0F) {
            pfPsiDeg += 360.0F;
        }

        // check for rounding errors mapping small negative angle to 360 deg
        if (pfPsiDeg >= 360.0F) {
            pfPsiDeg = 0.0F;
        }

        // the compass heading angle Rho equals the yaw angle Psi
        // this definition is compliant with Motorola Xoom tablet behavior
        pfRhoDeg = pfPsiDeg;

        // calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
        pfChiDeg = acos_deg(R[2][2]);
    }

    // extract the Windows 8 angles in degrees from the Windows 8 rotation matrix
    void fWin8AnglesDegFromRotationMatrix(double R[][3],
                                          double& pfPhiDeg,
                                          double& pfTheDeg,
                                          double& pfPsiDeg,
                                          double& pfRhoDeg,
                                          double& pfChiDeg) {
        // calculate the roll angle -90.0 <= Phi <= 90.0 deg
        if (R[2][2] == 0.0F) {
            if (R[0][2] >= 0.0F) {
                // tan(phi) is -infinity
                pfPhiDeg = -90.0F;
            }
            else {
                // tan(phi) is +infinity
                pfPhiDeg = 90.0F;
            }
        }
        else {
            // general case
            pfPhiDeg = atan_deg(-R[0][2] / R[2][2]);
        }

        // first calculate the pitch angle The in the range -90.0 <= The <= 90.0 deg
        pfTheDeg = asin_deg(R[1][2]);

        // use R[2][2]=cos(Phi)*cos(The) to correct the quadrant of The remembering
        // cos(Phi) is non-negative so that cos(The) has the same sign as R[2][2].
        if (R[2][2] < 0.0F) {
            // wrap The around +90 deg and -90 deg giving result 90 to 270 deg
            pfTheDeg = 180.0F - pfTheDeg;
        }

        // map the pitch angle The to the range -180.0 <= The < 180.0 deg
        if (pfTheDeg >= 180.0F) {
            pfTheDeg -= 360.0F;
        }

        // calculate the yaw angle Psi
        if (pfTheDeg == 90.0F) {
            // vertical upwards gimbal lock case: -270 <= Psi < 90 deg
            pfPsiDeg = atan2_deg(R[0][1], R[0][0]) - pfPhiDeg;
        }
        else if (pfTheDeg == -90.0F) {
            // vertical downwards gimbal lock case: -270 <= Psi < 90 deg
            pfPsiDeg = atan2_deg(R[0][1], R[0][0]) + pfPhiDeg;
        }
        else {
            // general case: -180 <= Psi < 180 deg
            pfPsiDeg = atan2_deg(-R[1][0], R[1][1]);

            // correct the quadrant for Psi using the value of The (deg) to give -180 <= Psi < 380 deg
            if (std::fabs(pfTheDeg) >= 90.0F) {
                pfPsiDeg += 180.0F;
            }
        }

        // map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
        if (pfPsiDeg < 0.0F) {
            pfPsiDeg += 360.0F;
        }

        // check for any rounding error mapping small negative angle to 360 deg
        if (pfPsiDeg >= 360.0F) {
            pfPsiDeg = 0.0F;
        }

        // compute the compass angle Rho = 360 - Psi
        pfRhoDeg = 360.0F - pfPsiDeg;

        // check for rounding errors mapping small negative angle to 360 deg and zero degree case
        if (pfRhoDeg >= 360.0F) {
            pfRhoDeg = 0.0F;
        }

        // calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
        pfChiDeg = acos_deg(R[2][2]);
    }

    // computes normalized rotation quaternion from a rotation vector (deg)
    void fQuaternionFromRotationVectorDeg(Eigen::Quaternion<double>& pq, double rvecdeg[], double fscaling) {
        double fetadeg    = 0.0;  // rotation angle (deg)
        double fetarad    = 0.0;  // rotation angle (rad)
        double fetarad2   = 0.0;  // eta (rad)^2
        double fetarad4   = 0.0;  // eta (rad)^4
        double sinhalfeta = 0.0;  // sin(eta/2)
        double fvecsq     = 0.0;  // q1^2+q2^2+q3^2
        double ftmp       = 0.0;  // scratch variable

        // compute the scaled rotation angle eta (deg) which can be both positve or negative
        fetadeg  = fscaling * std::sqrt(rvecdeg[0] * rvecdeg[0] + rvecdeg[1] * rvecdeg[1] + rvecdeg[2] * rvecdeg[2]);
        fetarad  = fetadeg * FDEGTORAD;
        fetarad2 = fetarad * fetarad;

        // calculate the sine and cosine using small angle approximations or exact
        // angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3 axes) at 200Hz and 405 deg/s at
        // 50Hz
        if (fetarad2 <= 0.02F) {
            // use MacLaurin series up to and including third order
            sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
        }
        else if (fetarad2 <= 0.06F) {
            // use MacLaurin series up to and including fifth order
            // angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703
            // deg/s at 50Hz
            fetarad4   = fetarad2 * fetarad2;
            sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
        }
        else {
            // use exact calculation
            sinhalfeta = static_cast<double>(std::sin(0.5 * fetarad));
        }
        double q1;
        double q2;
        double q3;

        // compute the vector quaternion components q1, q2, q3
        if (fetadeg != 0.0F) {
            // general case with non-zero rotation angle
            ftmp = fscaling * sinhalfeta / fetadeg;
            q1   = rvecdeg[0] * ftmp;  // q1 = nx * sin(eta/2)
            q2   = rvecdeg[1] * ftmp;  // q2 = ny * sin(eta/2)
            q3   = rvecdeg[2] * ftmp;  // q3 = nz * sin(eta/2)
        }
        else {
            // zero rotation angle giving zero vector component
            q1 = q2 = q3 = 0.0F;
        }

        // compute the scalar quaternion component q0 by explicit normalization
        // taking care to avoid rounding errors giving negative operand to sqrt
        fvecsq = q1 * q1 + q2 * q2 + q3 * q3;
        if (fvecsq <= 1.0F) {
            // normal case
            pq.w() = std::sqrt(1.0F - fvecsq);
        }
        else {
            // rounding errors are present
            pq.w() = 0.0F;
        }
        pq.vec() = Eigen::Matrix<double, 3, 1>(q1, q2, q3);
    }

    // compute the orientation quaternion from a 3x3 rotation matrix
    void fQuaternionFromRotationMatrix(double R[][3], Eigen::Quaternion<double>& pq) {
        double fq0sq    = 0.0;  // q0^2
        double recip4q0 = 0.0;  // 1/4q0
        double q1;
        double q2;
        double q3;

        // the quaternion is not explicitly normalized in this function on the assumption that it
        // is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
        // the quaternion will also be normalized even if the case of small q0

        // get q0^2 and q0
        fq0sq  = 0.25F * (1.0F + R[0][0] + R[1][1] + R[2][2]);
        pq.w() = std::sqrt(std::fabs(fq0sq));

        // normal case when q0 is not small meaning rotation angle not near 180 deg
        if (pq.w() > SMALLQ0) {
            // calculate q1 to q3
            recip4q0 = 0.25F / pq.w();
            q1       = recip4q0 * (R[1][2] - R[2][1]);
            q2       = recip4q0 * (R[2][0] - R[0][2]);
            q3       = recip4q0 * (R[0][1] - R[1][0]);
        }  // end of general case
        else {
            // special case of near 180 deg corresponds to nearly symmetric matrix
            // which is not numerically well conditioned for division by small q0
            // instead get absolute values of q1 to q3 from leading diagonal
            q1 = std::sqrt(std::fabs(0.5F * (1.0F + R[0][0]) - fq0sq));
            q2 = std::sqrt(std::fabs(0.5F * (1.0F + R[1][1]) - fq0sq));
            q3 = std::sqrt(std::fabs(0.5F * (1.0F + R[2][2]) - fq0sq));

            // correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
            if ((R[1][2] - R[2][1]) < 0.0F) {
                q1 = -q1;
            }
            if ((R[2][0] - R[0][2]) < 0.0F) {
                q2 = -q2;
            }
            if ((R[0][1] - R[1][0]) < 0.0F) {
                q3 = -q3;
            }
        }  // end of special case

        pq.vec() = Eigen::Matrix<double, 3, 1>(q1, q2, q3);
    }

    // compute the rotation matrix from an orientation quaternion
    void fRotationMatrixFromQuaternion(double R[][3], Eigen::Quaternion<double>& pq) {
        double f2q                                  = 0.0;
        double f2q0q0                               = 0.0;
        double f2q0q1                               = 0.0;
        double f2q0q2                               = 0.0;
        double f2q0q3                               = 0.0;
        double f2q1q1                               = 0.0;
        double f2q1q2                               = 0.0;
        double f2q1q3                               = 0.0;
        double f2q2q2                               = 0.0;
        double f2q2q3                               = 0.0;
        double f2q3q3                               = 0.0;
        const Eigen::Matrix<double, 3, 1> imag_part = pq.vec();
        double q1                                   = imag_part.x();
        double q2                                   = imag_part.y();
        double q3                                   = imag_part.z();

        // calculate products
        f2q    = 2.0F * pq.w();
        f2q0q0 = f2q * pq.w();
        f2q0q1 = f2q * q1;
        f2q0q2 = f2q * q2;
        f2q0q3 = f2q * q3;
        f2q    = 2.0F * q1;
        f2q1q1 = f2q * q1;
        f2q1q2 = f2q * q2;
        f2q1q3 = f2q * q3;
        f2q    = 2.0F * q2;
        f2q2q2 = f2q * q2;
        f2q2q3 = f2q * q3;
        f2q3q3 = 2.0F * q3 * q3;

        // calculate the rotation matrix assuming the quaternion is normalized
        R[0][0] = f2q0q0 + f2q1q1 - 1.0;
        R[0][1] = f2q1q2 + f2q0q3;
        R[0][2] = f2q1q3 - f2q0q2;
        R[1][0] = f2q1q2 - f2q0q3;
        R[1][1] = f2q0q0 + f2q2q2 - 1.0;
        R[1][2] = f2q2q3 + f2q0q1;
        R[2][0] = f2q1q3 + f2q0q2;
        R[2][1] = f2q2q3 - f2q0q1;
        R[2][2] = f2q0q0 + f2q3q3 - 1.0;

        return;
    }

    // function calculate the rotation vector from a rotation matrix
    void fRotationVectorDegFromRotationMatrix(double R[][3], double rvecdeg[]) {
        double ftrace;    // trace of the rotation matrix
        double fetadeg;   // rotation angle eta (deg)
        double fmodulus;  // modulus of axis * angle vector = 2|sin(eta)|
        double ftmp;      // scratch variable

        // calculate the trace of the rotation matrix = 1+2cos(eta) in range -1 to +3
        // and eta (deg) in range 0 to 180 deg inclusive
        // checking for rounding errors that might take the trace outside this range
        ftrace = R[0][0] + R[1][1] + R[2][2];
        if (ftrace >= 3.0F) {
            fetadeg = 0.0F;
        }
        else if (ftrace <= -1.0F) {
            fetadeg = 180.0F;
        }
        else {
            fetadeg = acosf(0.5F * (ftrace - 1.0F)) * FRADTODEG;
        }

        // set the rvecdeg vector to differences across the diagonal = 2*n*sin(eta)
        // and calculate its modulus equal to 2|sin(eta)|
        // the modulus approaches zero near 0 and 180 deg (when sin(eta) approaches zero)
        rvecdeg[0] = R[1][2] - R[2][1];
        rvecdeg[1] = R[2][0] - R[0][2];
        rvecdeg[2] = R[0][1] - R[1][0];
        fmodulus   = std::sqrt(rvecdeg[0] * rvecdeg[0] + rvecdeg[1] * rvecdeg[1] + rvecdeg[2] * rvecdeg[2]);

        // normalize the rotation vector for general, 0 deg and 180 deg rotation cases
        if (fmodulus > SMALLMODULUS) {
            // general case away from 0 and 180 deg rotation
            ftmp = fetadeg / fmodulus;
            rvecdeg[0] *= ftmp;  // set x component to eta(deg) * nx
            rvecdeg[1] *= ftmp;  // set y component to eta(deg) * ny
            rvecdeg[2] *= ftmp;  // set z component to eta(deg) * nz
        }                        // end of general case
        else if (ftrace >= 0.0F) {
            // near 0 deg rotation (trace = 3): matrix is nearly identity matrix
            // R[1][2]-R[2][1]=2*nx*eta(rad) and similarly for other components
            ftmp = 0.5F * FRADTODEG;
            rvecdeg[0] *= ftmp;
            rvecdeg[1] *= ftmp;
            rvecdeg[2] *= ftmp;
        }  // end of zero deg case
        else {
            // near 180 deg (trace = -1): matrix is nearly symmetric
            // calculate the absolute value of the components of the axis-angle vector
            rvecdeg[0] = 180.0F * std::sqrt(std::fabs(0.5F * (R[0][0] + 1.0F)));
            rvecdeg[1] = 180.0F * std::sqrt(std::fabs(0.5F * (R[1][1] + 1.0F)));
            rvecdeg[2] = 180.0F * std::sqrt(std::fabs(0.5F * (R[2][2] + 1.0F)));

            // correct the signs of the three components by examining the signs of differenced off-diagonal terms
            if ((R[1][2] - R[2][1]) < 0.0F)
                rvecdeg[0] = -rvecdeg[0];
            if ((R[2][0] - R[0][2]) < 0.0F)
                rvecdeg[1] = -rvecdeg[1];
            if ((R[0][1] - R[1][0]) < 0.0F)
                rvecdeg[2] = -rvecdeg[2];

        }  // end of 180 deg case

        return;
    }

    // computes rotation vector (deg) from rotation quaternion
    void fRotationVectorDegFromQuaternion(Eigen::Quaternion<double>& pq, double rvecdeg[]) {
        double fetarad;     // rotation angle (rad)
        double fetadeg;     // rotation angle (deg)
        double sinhalfeta;  // sin(eta/2)
        double ftmp;        // scratch variable

        // calculate the rotation angle in the range 0 <= eta < 360 deg
        if ((pq.w() >= 1.0F) || (pq.w() <= -1.0F)) {
            // rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
            fetarad = 0.0F;
            fetadeg = 0.0F;
        }
        else {
            // general case returning 0 < eta < 360 deg
            fetarad = 2.0F * acosf(pq.w());
            fetadeg = fetarad * FRADTODEG;
        }

        // map the rotation angle onto the range -180 deg <= eta < 180 deg
        if (fetadeg >= 180.0F) {
            fetadeg -= 360.0F;
            fetarad = fetadeg * FDEGTORAD;
        }

        // calculate sin(eta/2) which will be in the range -1 to +1
        sinhalfeta = std::sin(0.5 * fetarad);

        // calculate the rotation vector (deg)
        if (sinhalfeta == 0.0F) {
            // the rotation angle eta is zero and the axis is irrelevant
            rvecdeg[0] = rvecdeg[1] = rvecdeg[2] = 0.0F;
        }
        else {
            Eigen::Matrix<double, 3, 1> imag_part = pq.vec();
            // general case with non-zero rotation angle
            ftmp       = fetadeg / sinhalfeta;
            rvecdeg[0] = imag_part.x() * ftmp;
            rvecdeg[1] = imag_part.y() * ftmp;
            rvecdeg[2] = imag_part.z() * ftmp;
        }

        return;
    }

    // function low pass filters an orientation quaternion and computes virtual gyro rotation rate
    void fLPFOrientationQuaternion(Eigen::Quaternion<double>& pq,
                                   Eigen::Quaternion<double>& pLPq,
                                   double flpf,
                                   double delta_t,
                                   double angular_velocity_vec[],
                                   int loopcounter) {
        // local variables
        Eigen::Quaternion<double> fdeltaq;  // delta rotation quaternion
        double rvecdeg[3];                  // rotation vector (deg)
        double ftmp;                        // scratch variable

        // initialize delay line on first pass: LPq[n]=q[n]
        if (loopcounter == 0) {
            pLPq = pq;
        }

        // set fdeltaqn to the delta rotation quaternion conjg(fLPq[n-1) . fqn
        fdeltaq = qconjgAxB(pLPq, pq);
        if (fdeltaq.w() < 0.0F) {
            fdeltaq.w()   = -fdeltaq.w();
            fdeltaq.vec() = -fdeltaq.vec();
            // fdeltaq.q1  = -fdeltaq.q1;
            // fdeltaq.q2  = -fdeltaq.q2;
            // fdeltaq.q3  = -fdeltaq.q3;
        }

        // set ftmp to a scaled lpf value which equals flpf in the limit of small rotations (q0=1)
        // but which rises as the delta rotation angle increases (q0 tends to zero)
        ftmp = flpf + 0.75F * (1.0F - fdeltaq.w());
        if (ftmp > 1.0F) {
            ftmp = 1.0F;
        }

        // scale the delta rotation by the corrected lpf value
        // fdeltaq.q1 *= ftmp;
        // fdeltaq.q2 *= ftmp;
        // fdeltaq.q3 *= ftmp;
        fdeltaq.vec() = ftmp * fdeltaq.vec();

        // compute the scalar component q0
        // ftmp = fdeltaq.q1 * fdeltaq.q1 + fdeltaq.q2 * fdeltaq.q2 + fdeltaq.q3 * fdeltaq.q3;
        ftmp = fdeltaq.vec().dot(fdeltaq.vec());
        if (ftmp <= 1.0) {
            // normal case
            fdeltaq.w() = std::sqrt(1.0 - ftmp);
        }
        else {
            // rounding errors present so simply set scalar component to 0
            fdeltaq.w() = 0.0;
        }

        // calculate the delta rotation vector from fdeltaqn and the virtual gyro angular velocity (deg/s)
        fRotationVectorDegFromQuaternion(fdeltaq, rvecdeg);
        ftmp                    = 1.0 / delta_t;
        angular_velocity_vec[0] = rvecdeg[0] * ftmp;
        angular_velocity_vec[1] = rvecdeg[1] * ftmp;
        angular_velocity_vec[2] = rvecdeg[2] * ftmp;

        // set LPq[n] = LPq[n-1] . deltaq[n]
        qAeqAxB(pLPq, fdeltaq);

        // renormalize the low pass filtered quaternion to prevent error accumulation
        // the renormalization function ensures that q0 is non-negative
        fqAeqNormqA(pLPq);

        return;
    }

    // function low pass filters a scalar
    void fLPFScalar(double& pfS, double& pfLPS, double& flpf, int loopcounter) {
        // set S[LP,n]=S[n] on first pass
        if (loopcounter == 0) {
            pfLPS = pfS;
        }

        // apply the exponential low pass filter
        pfLPS += flpf * (pfS - pfLPS);

        return;
    }

    // function compute the quaternion product qA * qB
    void qAeqBxC(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB, Eigen::Quaternion<double>& pqC) {
        Eigen::Matrix<double, 3, 1> imag_part_B = pqB.vec();
        const double b_q1                       = imag_part_B.x();
        const double b_q2                       = imag_part_B.y();
        const double b_q3                       = imag_part_B.z();
        Eigen::Matrix<double, 3, 1> imag_part_C = pqC.vec();
        const double c_q1                       = imag_part_C.x();
        const double c_q2                       = imag_part_C.y();
        const double c_q3                       = imag_part_C.z();

        pqA               = Eigen::Quaternion<double>::Identity();
        pqA.w()           = pqB.w() * pqC.w() - b_q1 * c_q1 - b_q2 * c_q2 - b_q3 * c_q3;
        const double a_q1 = pqB.w() * c_q1 + b_q1 * pqC.w() + b_q2 * c_q3 - b_q3 * c_q2;
        const double a_q2 = pqB.w() * c_q2 - b_q1 * c_q3 + b_q2 * pqC.w() + b_q3 * c_q1;
        const double a_q3 = pqB.w() * c_q3 + b_q1 * c_q2 - b_q2 * c_q1 + b_q3 * pqC.w();
        pqA.vec()         = Eigen::Matrix<double, 3, 1>(a_q1, a_q2, a_q3);
        return;
    }

    // function compute the quaternion product qA = qA * qB
    void qAeqAxB(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB) {
        Eigen::Quaternion<double> qProd;
        Eigen::Matrix<double, 3, 1> imag_part_A = pqA.vec();
        const double a_q1                       = imag_part_A.x();
        const double a_q2                       = imag_part_A.y();
        const double a_q3                       = imag_part_A.z();
        Eigen::Matrix<double, 3, 1> imag_part_B = pqB.vec();
        const double b_q1                       = imag_part_B.x();
        const double b_q2                       = imag_part_B.y();
        const double b_q3                       = imag_part_B.z();
        // perform the quaternion product
        qProd.w()       = pqA.w() * pqB.w() - a_q1 * b_q1 - a_q2 * b_q2 - a_q3 * b_q3;
        const double q1 = pqA.w() * b_q1 + a_q1 * pqB.w() + a_q2 * b_q3 - a_q3 * b_q2;
        const double q2 = pqA.w() * b_q2 - a_q1 * b_q3 + a_q2 * pqB.w() + a_q3 * b_q1;
        const double q3 = pqA.w() * b_q3 + a_q1 * b_q2 - a_q2 * b_q1 + a_q3 * pqB.w();

        // copy the result back into qA
        // pqA = qProd;
        pqA.w()   = qProd.w();
        pqA.vec() = Eigen::Matrix<double, 3, 1>(q1, q2, q3);
        return;
    }

    // function compute the quaternion product conjg(qA) * qB
    Eigen::Quaternion<double> qconjgAxB(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB) {
        Eigen::Quaternion<double> qProd;
        Eigen::Matrix<double, 3, 1> imag_part_A = pqA.vec();
        const double a_q1                       = imag_part_A.x();
        const double a_q2                       = imag_part_A.y();
        const double a_q3                       = imag_part_A.z();
        Eigen::Matrix<double, 3, 1> imag_part_B = pqB.vec();
        const double b_q1                       = imag_part_B.x();
        const double b_q2                       = imag_part_B.y();
        const double b_q3                       = imag_part_B.z();


        qProd.w()       = pqA.w() * pqB.w() + a_q1 * b_q1 + a_q2 * b_q2 + a_q3 * b_q3;
        const double q1 = pqA.w() * b_q1 - a_q1 * pqB.w() - a_q2 * b_q3 + a_q3 * b_q2;
        const double q2 = pqA.w() * b_q2 + a_q1 * b_q3 - a_q2 * pqB.w() - a_q3 * b_q1;
        const double q3 = pqA.w() * b_q3 - a_q1 * b_q2 + a_q2 * b_q1 - a_q3 * pqB.w();

        return qProd;
    }

    // function normalizes a rotation quaternion and ensures q0 is non-negative
    void fqAeqNormqA(Eigen::Quaternion<double>& pqA) {
        double fNorm;  // quaternion Norm
        const Eigen::Matrix<double, 3, 1> imag_part = pqA.vec();
        double q1                                   = imag_part.x();
        double q2                                   = imag_part.y();
        double q3                                   = imag_part.z();

        // calculate the quaternion Norm
        fNorm = std::sqrt(pqA.w() * pqA.w() + q1 * q1 + q2 * q2 + q3 * q3);
        if (fNorm > CORRUPTQUAT) {
            // general case
            fNorm = 1.0F / fNorm;
            pqA.w() *= fNorm;
            q1 *= fNorm;
            q2 *= fNorm;
            q3 *= fNorm;
        }
        else {
            // return with identity quaternion since the quaternion is corrupted
            pqA.w() = 1.0F;
            q1 = q2 = q3 = 0.0F;
        }

        // correct a negative scalar component if the function was called with negative q0
        if (pqA.w() < 0.0F) {
            pqA.w() = -pqA.w();
            q1      = -q1;
            q2      = -q2;
            q3      = -q3;
        }
        pqA.vec() = Eigen::Matrix<double, 3, 1>(q1, q2, q3);
        return;
    }

    // set a quaternion to the unit quaternion
    void fqAeq1(Eigen::Quaternion<double>& pqA) {
        // pqA.w() = 1.0F;
        // pqA.q1 = pqA.q2 = pqA.q3 = 0.0F;
        pqA = Eigen::Quaternion<double>::Identity();
        return;
    }
}  // namespace filter::orientation