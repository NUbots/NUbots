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
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "build.hpp"
namespace filter::matrix {
    // function prototypes
    void f3x3matrixAeqI(double A[][3]);
    void fmatrixAeqI(double* A[], int rc);
    void f3x3matrixAeqScalar(double A[][3], double Scalar);
    void f3x3matrixAeqInvSymB(double A[][3], double B[][3]);
    void f3x3matrixAeqAxScalar(double A[][3], double Scalar);
    void f3x3matrixAeqMinusA(double A[][3]);
    double f3x3matrixDetA(double A[][3]);
    void eigencompute(double A[][10], double eigval[], double eigvec[][10], int n);
    void fmatrixAeqInvA(double* A[], int iColInd[], int iRowInd[], int iPivot[], int isize);
    void fmatrixAeqRenormRotA(double A[][3]);
}  // namespace filter::matrix
#endif  // #ifndef MATRIX_HPP
