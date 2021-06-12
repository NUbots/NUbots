// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     & Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     & Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     & Neither the name of Freescale Semiconductor, Inc. nor the
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
#ifndef ORIENTATION_HPP
#define ORIENTATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "build.hpp"

namespace filter::orientation {
    // function prototypes
    void f3DOFTiltNED(double fR[][3], double fGp[]);
    void f3DOFTiltAndroid(double fR[][3], double fGp[]);
    void f3DOFTiltWin8(double fR[][3], double fGp[]);
    void f3DOFMagnetometerMatrixNED(double fR[][3], double fBc[]);
    void f3DOFMagnetometerMatrixAndroid(double fR[][3], double fBc[]);
    void f3DOFMagnetometerMatrixWin8(double fR[][3], double fBc[]);
    void fNEDAnglesDegFromRotationMatrix(double R[][3],
                                         double& pfPhiDeg,
                                         double& pfTheDeg,
                                         double& pfPsiDeg,
                                         double& pfRhoDeg,
                                         double& pfChiDeg);
    void fAndroidAnglesDegFromRotationMatrix(double R[][3],
                                             double& pfPhiDeg,
                                             double& pfTheDeg,
                                             double& pfPsiDeg,
                                             double& pfRhoDeg,
                                             double& pfChiDeg);
    void fWin8AnglesDegFromRotationMatrix(double R[][3],
                                          double& pfPhiDeg,
                                          double& pfTheDeg,
                                          double& pfPsiDeg,
                                          double& pfRhoDeg,
                                          double& pfChiDeg);
    void fQuaternionFromRotationMatrix(double R[][3], Eigen::Quaternion<double>& pq);
    void fRotationMatrixFromQuaternion(double R[][3], Eigen::Quaternion<double>& pq);
    void fLPFScalar(double& pfS, double& pfLPS, double& flpf, int loopcounter);
    void qAeqBxC(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB, Eigen::Quaternion<double>& pqC);
    void qAeqAxB(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB);
    Eigen::Quaternion<double> qconjgAxB(Eigen::Quaternion<double>& pqA, Eigen::Quaternion<double>& pqB);
    void fqAeqNormqA(Eigen::Quaternion<double>& pqA);
    void fqAeq1(Eigen::Quaternion<double>& pqA);
    void fRotationVectorDegFromRotationMatrix(double R[][3], double rvecdeg[]);
    void fQuaternionFromRotationVectorDeg(Eigen::Quaternion<double>& pq, double rvecdeg[], double fscaling);
    void fRotationVectorDegFromQuaternion(Eigen::Quaternion<double>& pq, double rvecdeg[]);
    void fLPFOrientationQuaternion(Eigen::Quaternion<double>& pq,
                                   Eigen::Quaternion<double>& pLPq,
                                   double flpf,
                                   double delta_t,
                                   double angular_velocity_vec[],
                                   int loopcounter);
}  // namespace filter::orientation
#endif  // #ifndef ORIENTATION_HPP
