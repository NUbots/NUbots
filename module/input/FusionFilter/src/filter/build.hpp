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
// This file contains build controls for a sensor fusion project.
// Board and MCU customization is done via Processor Expert.  The fusion
// code sits above that software layer, and can usually be ported from
// one board environment to another with no changes.

#ifndef BUILD_HPP
#define BUILD_HPP

// coordinate system for the build
// TODO: Make an enum
static constexpr int NED     = 0;  // identifier for NED angle output
static constexpr int ANDROID = 1;  // identifier for Android angle output
static constexpr int WIN8    = 2;  // identifier for Windows 8 angle output

// sampling rate and kalman filter timing
static constexpr int FTM_INCLK_HZ     = 1000000;  // int: 1MHz FTM timer frequency set in PE: do not change
static constexpr int SENSORFS         = 200;      // int: 200Hz: frequency (Hz) of sensor sampling process
static constexpr int OVERSAMPLE_RATIO = 1;        // int: 8x: 3DOF, 6DOF, 9DOF run at SENSORFS / OVERSAMPLE_RATIO Hz

// geomagnetic model parameters
static constexpr double DEFAULTB = 50.0;  // default geomagnetic field (uT)

// useful multiplicative conversion constants
static constexpr double PI           = 3.141592654;       // Pi
static constexpr double FDEGTORAD    = 0.01745329251994;  // degrees to radians conversion = pi / 180
static constexpr double FRADTODEG    = 57.2957795130823;  // radians to degrees conversion = 180 / pi
static constexpr double FRECIP180    = 0.0055555555555;   // multiplicative factor 1/180
static constexpr double ONETHIRD     = 0.33333333;        // one third
static constexpr double ONESIXTH     = 0.166666667;       // one sixth
static constexpr double ONETWELFTH   = 0.0833333333;      // one twelfth
static constexpr double ONEOVER48    = 0.02083333333;     // 1 / 48
static constexpr double ONEOVER120   = 0.0083333333;      // 1 / 120
static constexpr double ONEOVER3840  = 0.0002604166667;   // 1 / 3840
static constexpr double ONEOVERROOT2 = 0.707106781;       // 1/sqrt(2)
static constexpr double ROOT3OVER2   = 0.866025403784;    // sqrt(3)/2

// *********************************************************************************
// COMPUTE_6DOF_GY_KALMAN constants
// *********************************************************************************
// kalman filter noise variances
static constexpr double FQVA_6DOF_GY_KALMAN = 2E-6;  // accelerometer noise g^2 so 1.4mg RMS
static constexpr double FQVG_6DOF_GY_KALMAN = 0.3;   // gyro noise (deg/s)^2
// gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
static constexpr double FQWB_6DOF_GY_KALMAN = 1E-9;
// linear acceleration drift g^2 (increase slows convergence to g but reduces sensitivity to shake)
static constexpr double FQWA_6DOF_GY_KALMAN = 1E-4;
// initialization of Qw covariance matrix
static constexpr double FQWINITTHTH_6DOF_GY_KALMAN = 2000E-5;  // th_e * th_e terms
static constexpr double FQWINITBB_6DOF_GY_KALMAN   = 250E-3;   // for FXAS21000: b_e * b_e terms
static constexpr double FQWINITTHB_6DOF_GY_KALMAN  = 0.0;      // th_e * b_e terms
static constexpr double FQWINITAA_6DOF_GY_KALMAN   = 10E-5;    // a_e * a_e terms (increase slows convergence to g but
                                                               // reduces sensitivity to shake)
// linear acceleration time constant
static constexpr double FCA_6DOF_GY_KALMAN = 0.5;  // linear acceleration decay factor

// *********************************************************************************
// COMPUTE_9DOF_GBY_KALMAN constants
// *********************************************************************************
// kalman filter noise variances
static constexpr double FQVA_9DOF_GBY_KALMAN = 2E-6;  // accelerometer noise g^2 so 1.4mg RMS
static constexpr double FQVM_9DOF_GBY_KALMAN = 0.1;   // magnetometer noise uT^2
static constexpr double FQVG_9DOF_GBY_KALMAN = 0.3;   // gyro noise (deg/s)^2
static constexpr double FQWB_9DOF_GBY_KALMAN = 1E-9;  // gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
static constexpr double FQWA_9DOF_GBY_KALMAN = 1E-4;  // linear acceleration drift g^2 (increase slows convergence to
                                                      // g but reduces sensitivity to shake)
static constexpr double FQWD_9DOF_GBY_KALMAN = 0.5;   // magnetic disturbance drift uT^2 (increase slows convergence to
                                                      // B but reduces sensitivity to magnet)
// initialization of Qw covariance matrix
static constexpr double FQWINITTHTH_9DOF_GBY_KALMAN = 2000E-5;  // th_e * th_e terms
static constexpr double FQWINITBB_9DOF_GBY_KALMAN   = 250E-3;   // b_e * b_e terms
static constexpr double FQWINITTHB_9DOF_GBY_KALMAN  = 0.0;      // th_e * b_e terms
static constexpr double FQWINITAA_9DOF_GBY_KALMAN   = 10E-5;    // a_e * a_e terms (increase slows convergence to g but
                                                                // reduces sensitivity to shake)
static constexpr double FQWINITDD_9DOF_GBY_KALMAN = 600E-3;     // d_e * d_e terms (increase slows convergence to B but
                                                                // reduces sensitivity to magnet)
// linear acceleration and magnetic disturbance time constants
static constexpr double FCA_9DOF_GBY_KALMAN = 0.5;  // linear acceleration decay factor
static constexpr double FCD_9DOF_GBY_KALMAN = 0.5;  // magnetic disturbance decay factor
// maximum geomagnetic inclination angle tracked by Kalman filter
static constexpr double SINDELTAMAX = 0.9063078;  // sin of max +ve geomagnetic inclination angle: here 65.0 deg
static constexpr double COSDELTAMAX = 0.4226183;  // cos of max +ve geomagnetic inclination angle: here 65.0 deg

#endif  // BUILD_HPP
