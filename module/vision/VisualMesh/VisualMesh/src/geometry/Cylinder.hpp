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

#ifndef VISUALMESH_GEOMETRY_CYLINDER_HPP
#define VISUALMESH_GEOMETRY_CYLINDER_HPP

#include <cmath>
#include "Sphere.hpp"

namespace visualmesh {
namespace geometry {

  template <typename Scalar>
  struct Cylinder {

    /**
     * @brief Construct a new Cylinder object for building a visual mesh.
     *        This implementation uses the spheres at both ends method rather than an entire cylinder.
     *
     * @param cylinder_height the height of the cylinder
     * @param intersections the number of intersections to ensure with a spherical section of the cylinder
     * @param max_distance  the maximum distance we want to look for this object
     */
    Cylinder(const Scalar& cylinder_height,
             const Scalar& radius,
             const unsigned int& intersections,
             const Scalar& max_distance)
      : sphere(radius, intersections, max_distance), ch(cylinder_height) {}

    /**
     * @brief Given a value for phi and a camera height, return the value to the next phi in the sequence.
     *
     * @param phi_n the current phi value in the series
     * @param h     the height of the camera above the observation plane
     *
     * @return the next phi in the sequence (phi_{n+1})
     */
    Scalar phi(const Scalar& phi_n, const Scalar& h) const {
      // Get values from the upper and lower observation planes
      Scalar u = sphere.phi(phi_n, h - ch);
      Scalar l = sphere.phi(phi_n, h);

      // If one is nan return the other one
      if (std::isnan(u)) {
        // If both are nan this will return nan here
        return l;
      }
      else {
        // If l is nan return u, otherwise return the smallest change
        return std::isnan(l) ? u : std::abs(u - phi_n) < std::abs(l - phi_n) ? u : l;
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
      // Get values from the upper and lower sphere
      Scalar u = sphere.theta(phi, h - ch);
      Scalar l = sphere.theta(phi, h);

      // If one is nan return the other one
      if (std::isnan(u)) {
        // If both are nan this will return nan here
        return l;
      }
      else {
        // If l is nan return u, otherwise return the smallest change
        return std::isnan(l) ? u : u < l ? u : l;
      }
    }

    /// The sphere used to represent either end of the cylinder
    Sphere<Scalar> sphere;
    /// The height of the cylinder
    Scalar ch;
  };

}  // namespace geometry
}  // namespace visualmesh

#endif  // VISUALMESH_GEOMETRY_CYLINDER_HPP
