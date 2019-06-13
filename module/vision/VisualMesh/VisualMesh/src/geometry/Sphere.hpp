/*
 * Copyright (C) 2017-2018 Trent Houliston <trent@houliston.me>
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

#ifndef VISUALMESH_GEOMETRY_SPHERE_HPP
#define VISUALMESH_GEOMETRY_SPHERE_HPP

#include <cmath>
#include <limits>

namespace visualmesh {
namespace geometry {

  template <typename Scalar>
  struct Sphere {

    /**
     * @brief Construct a new Sphere object for building a visual mesh
     *
     * @param radius the radius of the sphere
     * @param intersections the number of intersections to ensure with this object
     * @param max_distance  the maximum distance we want to look for this object
     */
    Sphere(const Scalar& radius, const Scalar& intersections, const Scalar& max_distance)
      : r(radius), k(intersections), d(max_distance) {}

    /**
     * @brief Given a value for phi and a camera height, return the value to the next phi in the sequence.
     *
     * @param phi_n the current phi value in the series
     * @param h     the height of the camera above the observation plane
     *
     * @return the next phi in the sequence (phi_{n+1})
     */
    Scalar phi(const Scalar& phi_n, const Scalar& h) const {

      // If we are beyond our max distance return nan
      if (std::abs(h) * std::tan(phi_n > M_PI_2 ? M_PI - phi_n : phi_n) > d) {
        return std::numeric_limits<Scalar>::quiet_NaN();
      }

      // Our effective radius for the number of intersections
      Scalar er = (h / 2) * (1 - std::pow(1 - 2 * r / h, 1 / k));

      // Valid below the horizon
      if (h > 0 && phi_n < M_PI_2) {
        return 2 * std::atan(er / (std::cos(phi_n) * (h - er)) + std::tan(phi_n)) - phi_n;
      }
      // Valid above the horizon
      else if (h < 0 && phi_n > M_PI_2) {
        return 2 * std::atan(er / (std::cos(M_PI - phi_n) * (-h - er)) + std::tan(M_PI - phi_n)) - M_PI - phi_n;
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
     * @param h the height of the camera above the observation plane
     *
     * @return the angular width of the object around a phi circle
     */
    Scalar theta(const Scalar& phi, const Scalar& h) const {

      // If we are beyond our max distance return nan
      if (std::abs(h) * std::tan(phi > M_PI_2 ? M_PI - phi : phi) > d) {
        return std::numeric_limits<Scalar>::quiet_NaN();
      }

      // Valid below the horizon
      if (h > 0 && phi < M_PI_2) { return 2 * std::asin(r / ((h - r) * std::tan(phi))) / k; }
      // Valid above the horizon
      else if (h < 0 && phi > M_PI_2) {
        return 2 * std::asin(r / (-(h - r) * std::tan(M_PI - phi))) / k;
      }
      // Other situations are invalid so return NaN
      else {
        return std::numeric_limits<Scalar>::quiet_NaN();
      }
    }

    // The radius of the sphere
    Scalar r;
    // The number of intersections the mesh should have with this sphere
    Scalar k;
    /// The maximum distance we want to see this object
    Scalar d;
  };

}  // namespace geometry
}  // namespace visualmesh

#endif  // VISUALMESH_GEOMETRY_SPHERE_HPP
