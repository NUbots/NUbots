/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef UTILITY_VISION_PROJECTION_HPP
#define UTILITY_VISION_PROJECTION_HPP

// https://wiki.panotools.org/Fisheye_Projection

#include <Eigen/Core>
#include <cmath>

#include "message/input/Image.hpp"

namespace utility::vision {

    namespace equidistant {
        template <typename T>
        inline T r(const T& theta, const T& f) {
            return f * theta;
        }

        template <typename T>
        inline T theta(const T& r, const T& f) {
            return r / f;
        }
    }  // namespace equidistant

    namespace equisolid {
        template <typename T>
        inline T r(const T& theta, const T& f) {
            return T(2.0) * f * std::sin(theta * T(0.5));
        }

        template <typename T>
        inline T theta(const T& r, const T& f) {
            return T(2.0) * std::asin(r / (T(2.0) * f));
        }
    }  // namespace equisolid

    namespace rectilinear {
        template <typename T>
        inline T r(const T& theta, const T& f) {
            return f * std::tan(std::min(std::max(theta, T(0.0)), T(M_PI_2)));
        }

        template <typename T>
        inline T theta(const T& r, const T& f) {
            return std::atan(r / f);
        }
    }  // namespace rectilinear

    /**
     * @brief Calculates polynomial coefficients that approximate the inverse distortion
     *
     * @details
     *  These coefficients are based on math from the paper
     *  An Exact Formula for Calculating Inverse Radial Lens Distortions
     *  https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/pdf/sensors-16-00807.pdf
     *  These terms have been stripped back to only include k1 and k2 and only uses the first 4 terms
     *  In general for most cases this provides an accuracy of around 0.2 pixels which is sufficient.
     *  If more accuracy is required in the future or more input parameters are used they can be adjusted here.
     *
     * @tparam T the scalar type used for calculations and storage (normally one of float or double)
     *
     * @param lens  the paramters that describe the lens that we are using to project
     *
     * @return the inverse coefficients that go from an undistorted image to a distorted one
     */
    template <typename T, typename Lens>
    inline Eigen::Matrix<T, 4, 1> inverse_coefficients(const Lens& lens) {
        const auto& k = lens.k;
        return Eigen::Matrix<T, 4, 1>(
            -k[0],
            T(3.0) * (k[0] * k[0]) - k[1],
            T(-12.0) * (k[0] * k[0]) * k[0] + T(8.0) * k[0] * k[1],
            T(55.0) * (k[0] * k[0]) * (k[0] * k[0]) - T(55.0) * (k[0] * k[0]) * k[1] + T(5.0) * (k[1] * k[1]));
    }

    /**
     * @brief Undistorts radial distortion using the provided distortion coefficients
     *
     * @details
     *  Given a radial distance from the optical centre, this applies a polynomial distortion model in order to
     *  approximate an ideal lens. After the radial distance has gone through this function it will approximate the
     *  equivilant radius in an ideal lens projection (depending on which base lens projection you are using).
     *
     * @tparam T the scalar type used for calculations and storage (normally one of float or double)
     *
     * @param r the radial distance from the optical centre
     * @param lens  the paramters that describe the lens that we are using to project
     *
     * @return the undistorted radial distance from the optical centre
     */
    template <typename T, typename Lens>
    inline T distort(const T& r, const Lens& lens) {
        // Uses the math from the paper
        // An Exact Formula for Calculating Inverse Radial Lens Distortions
        // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/pdf/sensors-16-00807.pdf
        // These terms have been stripped back to only include k1 and k2 and only uses the first 4 terms
        // if more are needed in the future go and get them from the original paper
        // TODO(thouliston) if performance ever becomes an issue, this can be precomputed for the same k values
        const Eigen::Matrix<T, 4, 1> ik = inverse_coefficients<T>(lens);
        return r
               * (1.0                                                  //
                  + ik[0] * (r * r)                                    //
                  + ik[1] * ((r * r) * (r * r))                        //
                  + ik[2] * ((r * r) * (r * r)) * (r * r)              //
                  + ik[3] * ((r * r) * (r * r)) * ((r * r) * (r * r))  //
               );
    }

    /**
     * @brief Undistorts radial distortion using the provided distortion coefficients
     *
     * @details
     *  Given a radial distance from the optical centre, this applies a polynomial distortion model in order to
     *  approximate an ideal lens. After the radial distance has gone through this function it will approximate the
     *  equivalent radius in an ideal lens projection (depending on which base lens projection you are using).
     *
     * @tparam T the scalar type used for calculations and storage (normally one of float or double)
     *
     * @param r the radial distance from the optical centre
     * @param lens  the paramters that describe the lens that we are using to project
     *
     * @return the undistorted radial distance from the optical centre
     */
    template <typename T, typename Lens>
    inline T undistort(const T& r, const Lens& lens) {
        const auto& k = lens.k;
        // These parenthesis are important as they allow the compiler to optimise further
        return r * (1.0 + k[0] * (r * r) + k[1] * (r * r) * (r * r));
    }

    /**
     * @brief Projects a unit vector into a pixel coordinate while working out which lens model to use via the lens
     *        parameters.
     *
     * @details
     *  This function expects a unit vector in camera space. For this camera space is defined as a coordinate system
     *  with the x axis going down the viewing direction of the camera, y is to the left of the image, and z is up in
     *  the resulting image. The pixel coordinate that results will have (0,0) at the top left of the image, with x
     *  to the right and y down.
     *
     * @tparam T the scalar type used for calculations and storage (normally one of float or double)
     *
     * @param p     the unit vector to project
     * @param lens  the paramters that describe the lens that we are using to project
     *
     * @return a pixel coordinate that this vector projects into
     */
    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time, typename Lens>
    Eigen::Matrix<T, 2, 1> project(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const Lens& lens,
        const Eigen::Matrix<T, 2, 1>& dimensions) {

        const T f          = lens.focal_length;
        const T theta      = std::acos(ray.x());
        const T rsin_theta = T(1) / std::sqrt(T(1) - ray.x() * ray.x());
        T r_u;
        switch (lens.projection.value) {
            case Lens::Projection::RECTILINEAR: r_u = rectilinear::r(theta, f); break;
            case Lens::Projection::EQUISOLID: r_u = equisolid::r(theta, f); break;
            case Lens::Projection::EQUIDISTANT: r_u = equidistant::r(theta, f); break;
            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }
        // If the dimensions are not width-normalised then we need to normalise r_u and then un-normalise r_d
        const T r_d = distort(r_u, lens);

        // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
        // Sometimes x is greater than one due to floating point error, this almost certainly means that we are
        // facing directly forward
        Eigen::Matrix<T, 2, 1> screen =
            ray.x() >= T(1) ? Eigen::Matrix<T, 2, 1>::Zero()
                            : Eigen::Matrix<T, 2, 1>(r_d * ray.y() * rsin_theta, r_d * ray.z() * rsin_theta);

        // Apply our offset to move into image space (0 at top left, x to the right, y down)
        // Then apply the offset to the centre of our lens
        return (dimensions * T(0.5)) - screen - lens.centre.template cast<T>();
    }

    /**
     * @brief Unprojects a pixel coordinate into a unit vector working out which lens model to use via the lens
     * parameters.
     *
     * @details
     *  This function expects a pixel coordinate having (0,0) at the top left of the image, with x to the right and
     *  y down. It will then convert this into a unit vector in camera space. For this camera space is defined as a
     *  coordinate system with the x axis going down the viewing direction of the camera, y is to the left of the
     *  image, and z is up.
     *
     * @tparam T the scalar type used for calculations and storage (normally one of float or double)
     *
     * @param px         the pixel coordinate to unproject
     * @param lens       the parameters that describe the lens that we are using to unproject
     * @param dimensions the dimensions of the image
     *
     * @return the unit vector that this pixel represents in camera space
     */
    template <typename T, typename Lens>
    Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& px,
                                     const Lens& lens,
                                     const Eigen::Matrix<T, 2, 1>& dimensions) {

        Eigen::Matrix<T, 2, 1> screen = (dimensions * T(0.5)) - px - lens.centre.template cast<T>();

        // Perform the unprojection math
        const T f   = lens.focal_length;
        const T r_d = screen.norm();
        if (r_d == T(0)) {
            return Eigen::Matrix<T, 3, 1>::UnitX();
        }
        // If the dimensions are not width-normalised then we need to normalise r_d and then un-normalise r_u
        const T r_u = undistort(r_d, lens);
        T theta;
        switch (lens.projection.value) {
            case Lens::Projection::RECTILINEAR: theta = rectilinear::theta(r_u, f); break;
            case Lens::Projection::EQUISOLID: theta = equisolid::theta(r_u, f); break;
            case Lens::Projection::EQUIDISTANT: theta = equidistant::theta(r_u, f); break;
            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }

        const T sin_theta = std::sin(theta);

        return Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * screen.x() / r_d, sin_theta * screen.y() / r_d);
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_HPP
