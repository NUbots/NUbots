/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_EIGEN_ARMADILLO_H
#define UTILITY_SUPPORT_EIGEN_ARMADILLO_H

#include <armadillo>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

// We officially hate armadillo from this point on


// FLOAT
template <int... Args>
auto convert(const typename arma::Col<float>::template fixed<2>& avec) {
    return (Eigen::Map<Eigen::Matrix<float, 2, 1, Args...>>(const_cast<float*>(avec.memptr()), 2));
}
template <int... Args>
auto convert(const typename arma::Col<float>::template fixed<3>& avec) {
    return (Eigen::Map<Eigen::Matrix<float, 3, 1, Args...>>(const_cast<float*>(avec.memptr()), 3));
}
template <int... Args>
auto convert(const typename arma::Col<float>::template fixed<4>& avec) {
    return (Eigen::Map<Eigen::Matrix<float, 4, 1, Args...>>(const_cast<float*>(avec.memptr()), 4));
}
template <int... Args>
auto convert(const typename arma::Col<float>& avec) {
    return (
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1, Args...>>(const_cast<float*>(avec.memptr()), avec.n_elem));
}
template <int... Args>
auto convert(const typename arma::Mat<float>::template fixed<2, 2>& amat) {
    return (Eigen::Map<Eigen::Matrix<float, 2, 2, Args...>>(const_cast<float*>(amat.memptr()), 2, 2));
}
template <int... Args>
auto convert(const typename arma::Mat<float>::template fixed<3, 3>& amat) {
    return (Eigen::Map<Eigen::Matrix<float, 3, 3, Args...>>(const_cast<float*>(amat.memptr()), 3, 3));
}
template <int... Args>
auto convert(const typename arma::Mat<float>::template fixed<4, 4>& amat) {
    return (Eigen::Map<Eigen::Matrix<float, 4, 4, Args...>>(const_cast<float*>(amat.memptr()), 4, 4));
}
template <int... Args>
auto convert(const typename arma::Mat<float>& amat) {
    return (Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Args...>>(
        const_cast<float*>(amat.memptr()), amat.n_rows, amat.n_cols));
}
// DOUBLE
template <int... Args>
auto convert(const typename arma::Col<double>::template fixed<2>& avec) {
    return (Eigen::Map<Eigen::Matrix<double, 2, 1, Args...>>(const_cast<double*>(avec.memptr()), 2));
}
template <int... Args>
auto convert(const typename arma::Col<double>::template fixed<3>& avec) {
    return (Eigen::Map<Eigen::Matrix<double, 3, 1, Args...>>(const_cast<double*>(avec.memptr()), 3));
}
template <int... Args>
auto convert(const utility::math::matrix::Transform2D& avec) {
    return (Eigen::Map<Eigen::Matrix<double, 3, 1, Args...>>(const_cast<double*>(avec.memptr()), 3));
}
template <int... Args>
auto convert(const typename arma::Col<double>::template fixed<4>& avec) {
    return (Eigen::Map<Eigen::Matrix<double, 4, 1, Args...>>(const_cast<double*>(avec.memptr()), 4));
}
template <int... Args>
auto convert(const typename arma::Col<double>& avec) {
    return (
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1, Args...>>(const_cast<double*>(avec.memptr()), avec.n_elem));
}
template <int... Args>
auto convert(const typename arma::Mat<double>::template fixed<2, 2>& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 2, 2, Args...>>(const_cast<double*>(amat.memptr()), 2, 2));
}
template <int... Args>
auto convert(const utility::math::matrix::Rotation2D& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 2, 2, Args...>>(const_cast<double*>(amat.memptr()), 2, 2));
}
template <int... Args>
auto convert(const typename arma::Mat<double>::template fixed<3, 3>& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 3, 3, Args...>>(const_cast<double*>(amat.memptr()), 3, 3));
}
template <int... Args>
auto convert(const utility::math::matrix::Rotation3D& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 3, 3, Args...>>(const_cast<double*>(amat.memptr()), 3, 3));
}
template <int... Args>
auto convert(const typename arma::Mat<double>::template fixed<4, 4>& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 4, 4, Args...>>(const_cast<double*>(amat.memptr()), 4, 4));
}
template <int... Args>
auto convert(const utility::math::matrix::Transform3D& amat) {
    return (Eigen::Map<Eigen::Matrix<double, 4, 4, Args...>>(const_cast<double*>(amat.memptr()), 4, 4));
}
template <int... Args>
auto convert(const typename arma::Mat<double>& amat) {
    return (Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Args...>>(
        const_cast<double*>(amat.memptr()), amat.n_rows, amat.n_cols));
}
// INT
template <int... Args>
auto convert(const typename arma::Col<int>::template fixed<2>& avec) {
    return (Eigen::Map<Eigen::Matrix<int, 2, 1, Args...>>(const_cast<int*>(avec.memptr()), 2));
}
template <int... Args>
auto convert(const typename arma::Col<int>::template fixed<3>& avec) {
    return (Eigen::Map<Eigen::Matrix<int, 3, 1, Args...>>(const_cast<int*>(avec.memptr()), 3));
}
template <int... Args>
auto convert(const typename arma::Col<int>::template fixed<4>& avec) {
    return (Eigen::Map<Eigen::Matrix<int, 4, 1, Args...>>(const_cast<int*>(avec.memptr()), 4));
}
template <int... Args>
auto convert(const typename arma::Col<int>& avec) {
    return (Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, 1, Args...>>(const_cast<int*>(avec.memptr()), avec.n_elem));
}
template <int... Args>
auto convert(const typename arma::Mat<int>::template fixed<2, 2>& amat) {
    return (Eigen::Map<Eigen::Matrix<int, 2, 2, Args...>>(const_cast<int*>(amat.memptr()), 2, 2));
}
template <int... Args>
auto convert(const typename arma::Mat<int>::template fixed<3, 3>& amat) {
    return (Eigen::Map<Eigen::Matrix<int, 3, 3, Args...>>(const_cast<int*>(amat.memptr()), 3, 3));
}
template <int... Args>
auto convert(const typename arma::Mat<int>::template fixed<4, 4>& amat) {
    return (Eigen::Map<Eigen::Matrix<int, 4, 4, Args...>>(const_cast<int*>(amat.memptr()), 4, 4));
}
template <int... Args>
auto convert(const typename arma::Mat<int>& amat) {
    return (Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Args...>>(
        const_cast<int*>(amat.memptr()), amat.n_rows, amat.n_cols));
}
// UINT
template <int... Args>
auto convert(const typename arma::Col<uint>::template fixed<2>& avec) {
    return (Eigen::Map<Eigen::Matrix<uint, 2, 1, Args...>>(const_cast<uint*>(avec.memptr()), 2));
}
template <int... Args>
auto convert(const typename arma::Col<uint>::template fixed<3>& avec) {
    return (Eigen::Map<Eigen::Matrix<uint, 3, 1, Args...>>(const_cast<uint*>(avec.memptr()), 3));
}
template <int... Args>
auto convert(const typename arma::Col<uint>::template fixed<4>& avec) {
    return (Eigen::Map<Eigen::Matrix<uint, 4, 1, Args...>>(const_cast<uint*>(avec.memptr()), 4));
}
template <int... Args>
auto convert(const typename arma::Col<uint>& avec) {
    return (Eigen::Map<Eigen::Matrix<uint, Eigen::Dynamic, 1, Args...>>(const_cast<uint*>(avec.memptr()), avec.n_elem));
}
template <int... Args>
auto convert(const typename arma::Mat<uint>::template fixed<2, 2>& amat) {
    return (Eigen::Map<Eigen::Matrix<uint, 2, 2, Args...>>(const_cast<uint*>(amat.memptr()), 2, 2));
}
template <int... Args>
auto convert(const typename arma::Mat<uint>::template fixed<3, 3>& amat) {
    return (Eigen::Map<Eigen::Matrix<uint, 3, 3, Args...>>(const_cast<uint*>(amat.memptr()), 3, 3));
}
template <int... Args>
auto convert(const typename arma::Mat<uint>::template fixed<4, 4>& amat) {
    return (Eigen::Map<Eigen::Matrix<uint, 4, 4, Args...>>(const_cast<uint*>(amat.memptr()), 4, 4));
}
template <int... Args>
auto convert(const typename arma::Mat<uint>& amat) {
    return (Eigen::Map<Eigen::Matrix<uint, Eigen::Dynamic, Eigen::Dynamic, Args...>>(
        const_cast<uint*>(amat.memptr()), amat.n_rows, amat.n_cols));
}

template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 2, 1, O, MR, MC>& evec) {
    typename arma::Col<Scalar>::template fixed<2> avec;
    Eigen::Map<Eigen::Matrix<Scalar, 2, 1, O, MR, MC>>(avec.memptr(), 2) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 3, 1, O, MR, MC>& evec) {
    typename arma::Col<Scalar>::template fixed<3> avec;
    Eigen::Map<Eigen::Matrix<Scalar, 3, 1, O, MR, MC>>(avec.memptr(), 3) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 4, 1, O, MR, MC>& evec) {
    typename arma::Col<Scalar>::template fixed<4> avec;
    Eigen::Map<Eigen::Matrix<Scalar, 4, 1, O, MR, MC>>(avec.memptr(), 4) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 6, 1, O, MR, MC>& evec) {
    typename arma::Col<Scalar>::template fixed<4> avec;
    Eigen::Map<Eigen::Matrix<Scalar, 6, 1, O, MR, MC>>(avec.memptr(), 6) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1, O, MR, MC>& evec) {
    typename arma::Col<Scalar> avec(evec.rows());
    Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1, O, MR, MC>>(avec.memptr(), evec.rows()) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 1, Eigen::Dynamic, O, MR, MC>& evec) {
    typename arma::Row<Scalar> avec(evec.cols());
    Eigen::Map<Eigen::Matrix<Scalar, 1, Eigen::Dynamic, O, MR, MC>>(avec.memptr(), evec.cols()) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 2, 2, O, MR, MC>& evec) {
    typename arma::Mat<Scalar>::template fixed<2, 2> avec;
    Eigen::Map<Eigen::Matrix<Scalar, 2, 2, O, MR, MC>>(avec.memptr(), 2, 2) = evec;
    return (avec);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 3, 3, O, MR, MC>& emat) {
    typename arma::Mat<Scalar>::template fixed<3, 3> amat;
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3, O, MR, MC>>(amat.memptr(), 3, 3) = emat;
    return (amat);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, 4, 4, O, MR, MC>& emat) {
    typename arma::Mat<Scalar>::template fixed<4, 4> amat;
    Eigen::Map<Eigen::Matrix<Scalar, 4, 4, O, MR, MC>>(amat.memptr(), 4, 4) = emat;
    return (amat);
}
template <typename Scalar, int O, int MR, int MC>
auto convert(const typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, O, MR, MC>& emat) {
    typename arma::Mat<Scalar> amat(emat.rows(), emat.cols());
    Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, O, MR, MC>>(
        amat.memptr(), emat.rows(), emat.cols()) = emat;
    return (amat);
}

#endif  // UTILITY_SUPPORT_EIGEN_ARMADILLO_H
