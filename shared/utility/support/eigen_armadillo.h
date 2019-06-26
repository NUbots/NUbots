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

#include <Eigen/Core>
#include <Eigen/Dense>

template <typename Scalar, int elems>
inline typename arma::Col<Scalar>::template fixed<elems> convert(
    const Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign>& evec) {
    typename arma::Col<Scalar>::template fixed<elems> avec;
    Eigen::Map<Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign>>(avec.memptr(), elems) = evec;
    return (avec);
}

template <typename Scalar, int elems>
inline Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign> convert(
    const typename arma::Col<Scalar>::template fixed<elems>& avec) {
    return (Eigen::Map<Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign>>(const_cast<Scalar*>(avec.memptr()), elems));
}

template <typename Scalar, int rows, int cols>
inline typename std::enable_if_t<rows != Eigen::Dynamic && cols != Eigen::Dynamic,
                                 typename arma::Mat<Scalar>::template fixed<rows, cols>>
convert(const Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>& emat) {
    typename arma::Mat<Scalar>::template fixed<rows, cols> amat;
    Eigen::Map<Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>>(amat.memptr(), rows, cols) = emat;
    return (amat);
}

template <typename Scalar, int rows, int cols>
inline Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign> convert(
    const typename arma::Mat<Scalar>::template fixed<rows, cols>& amat) {
    return (Eigen::Map<Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>>(
        const_cast<Scalar*>(amat.memptr()), rows, cols));
}

template <typename Scalar>
inline arma::Col<Scalar> convert(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& evec) {
    arma::Col<Scalar> avec;
    Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(avec.memptr(), evec.rows(), 1) = evec;
    return (avec);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1> convert(const arma::Col<Scalar>& avec) {
    return (Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(const_cast<Scalar*>(avec.memptr()), avec.n_elem));
}

template <typename Scalar, int rows = Eigen::Dynamic, int cols = Eigen::Dynamic>
inline typename std::enable_if_t<rows == Eigen::Dynamic && cols == Eigen::Dynamic, typename arma::Mat<Scalar>> convert(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& emat) {
    typename arma::Mat<Scalar> amat;
    Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(amat.memptr(), emat.rows(), emat.cols()) = emat;
    return (amat);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> convert(const arma::Mat<Scalar>& amat) {
    return (Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
        const_cast<Scalar*>(amat.memptr()), amat.n_rows, amat.n_cols));
}

/**
 * @brief Generalised function for streaming Armadillo vectors into Eigen vectors.
 *
 * @details This uses the above templated convert functions to set the components of an Eigen vector.
 *
 * @param emat The Eigen vector
 * @param ama The Armadillo vector
 *
 * @return The original protocol buffer instance
 */
template <typename Scalar, int elems>
inline Eigen::Matrix<Scalar, elems, 1>& operator<<(Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign>& evec,
                                                   const typename arma::Col<Scalar>::template fixed<elems>& avec) {
    evec = convert(avec);
    return (evec);
}

template <typename Scalar, int elems>
inline typename arma::Col<Scalar>::template fixed<elems>& operator<<(
    typename arma::Col<Scalar>::template fixed<elems>& avec,
    const Eigen::Matrix<Scalar, elems, 1, Eigen::DontAlign>& evec) {
    avec = convert(evec);
    return (avec);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& operator<<(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& evec,
                                                            const arma::Col<Scalar>& avec) {
    evec = convert(avec);
    return (evec);
}

template <typename Scalar, int elems>
inline arma::Col<Scalar>& operator<<(arma::Col<Scalar>& avec, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& evec) {
    avec = convert(evec);
    return (avec);
}

/**
 * @brief Generalised function for streaming Armadillo square matricies into Eigen square matrices
 *
 * @details This uses the above templated convert functions to set the components of an Eigen matrix.
 *
 * @param emat The Eigen matrix
 * @param ama The Armadillo matrix
 *
 * @return The converted matrix instance
 */
template <typename Scalar, int rows, int cols>
inline Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>& operator<<(
    Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>& evec,
    const typename arma::Mat<Scalar>::template fixed<rows, cols>& avec) {
    evec = convert(avec);
    return (evec);
}

template <typename Scalar, int rows, int cols>
inline typename arma::Mat<Scalar>::template fixed<rows, cols>& operator<<(
    typename arma::Mat<Scalar>::template fixed<rows, cols>& avec,
    const Eigen::Matrix<Scalar, rows, cols, Eigen::DontAlign>& evec) {
    avec = convert(evec);
    return (avec);
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& operator<<(
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& evec,
    const arma::Mat<Scalar>& avec) {
    evec = convert(avec);
    return (evec);
}

template <typename Scalar>
inline arma::Mat<Scalar>& operator<<(arma::Mat<Scalar>& avec,
                                     const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& evec) {
    avec = convert(evec);
    return (avec);
}

#endif
