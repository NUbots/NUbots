/*
 * Copyright (C) 2017 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef MESH_CIRCLE_HPP
#define MESH_CIRCLE_HPP

#include <cmath>
#include <limits>

namespace mesh {

template <typename Scalar>
struct Circle {

    /**
     * @brief Construct a new Circle object for building a visual mesh
     *
     * @param height the height of the object above the observation plane
     * @param radius the radius of the circle
     * @param intersections the number of intersections to ensure with this object
     * @param max_distance  the maximum distance we want to look for this object
     */
    Circle(const Scalar& height, const Scalar& radius, const size_t& intersections, const Scalar& max_distance)
        : h(height), r(radius), i(intersections), d(max_distance) {}

    /**
     * @brief Given a value for phi and a camera height, return the value to the next phi in the sequence.
     *
     * @param phi_n the current phi value in the series
     * @param c     the height of the camera above the observation plane
     *
     * @return the next phi in the sequence (phi_n+1)
     */
    Scalar phi(const Scalar& phi_n, const Scalar& c) const {

        // Our effective height above the plane
        Scalar eh = c - h;

        // If we are beyond our max distance return nan
        if (std::abs(eh) * tan(phi_n > M_PI_2 ? M_PI - phi_n : phi_n) > d) {
            return std::numeric_limits<Scalar>::quiet_NaN();
        }

        // Valid below the horizon
        if (eh > 0 && phi_n < M_PI_2) {
            return atan((2 * r / i + eh * tan(phi_n)) / eh);
        }
        // Valid above the horizon
        else if (eh < 0 && phi_n > M_PI_2) {
            return M_PI - atan((2 * r / i - eh * tan(M_PI - phi_n)) / -eh);
        }
        // Other situations are invalid so return NaN
        else {
            return std::numeric_limits<Scalar>::quiet_NaN();
        }
    }

    /**
     * @brief Given a value for phi and a camera height, return the angular width for an object
     *
     * @param phi the phi value to calculate our theta value for
     * @param c the height of the camera above the observation plane
     *
     * @return the angular width of the object around a phi circle
     */
    Scalar theta(const Scalar& phi, const Scalar& c) const {

        // Our effective height above the observation plane
        Scalar eh = c - h;

        // If we are beyond our max distance return nan
        if (std::abs(eh) * tan(phi > M_PI_2 ? M_PI - phi : phi) > d) {
            return std::numeric_limits<Scalar>::quiet_NaN();
        }

        // Valid below the horizon
        if (eh > 0 && phi < M_PI_2) {
            return 2 * asin(r / (eh * tan(phi) + r)) / i;
        }
        // Valid above the horizon
        else if (eh < 0 && phi > M_PI_2) {
            return 2 * asin(r / (-eh * tan(M_PI - phi) + r)) / i;
        }
        // Other situations are invalid so return NaN
        else {
            return std::numeric_limits<Scalar>::quiet_NaN();
        }
    }

    /// The height of the object above the observation plane
    Scalar h;
    // The radius of the sphere
    Scalar r;
    // The number of intersections the mesh should have with this sphere
    size_t i;
    /// The maximum distance we want to see this object
    Scalar d;
};

}  // namespace mesh

#endif  // MESH_CIRCLE_HPP
