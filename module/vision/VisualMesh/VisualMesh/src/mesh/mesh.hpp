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

#ifndef VISUALMESH_MESH_HPP
#define VISUALMESH_MESH_HPP

#include <array>
#include <utility>
#include <vector>
#include "lens.hpp"
#include "util/math.hpp"

namespace visualmesh {

template <typename Scalar>
struct Node {
  /// The unit vector in the direction for this node
  vec4<Scalar> ray;
  /// Relative indices to the linked hexagon nodes in the LUT ordered TL, TR, L, R, BL, BR,
  std::array<int, 6> neighbours;
};

template <typename Scalar>
struct Row {
  Row(const Scalar& phi, const int& begin, const int& end) : phi(phi), begin(begin), end(end) {}

  /// The phi value this row represents
  Scalar phi;
  /// The index of the beginning of this row in the node table
  int begin;
  /// The index of one past the end of this row in the node table
  int end;

  /**
   * @brief Compare based on phi
   *
   * @param other other item to compare to
   *
   * @return if our phi is less than other phi
   */
  bool operator<(const Row& other) const {
    return phi < other.phi;
  }
};

template <typename Scalar>
struct Mesh {

  template <typename Shape>
  Mesh(const Shape& shape, const Scalar& h) {

    // This is a list of phi values along with the delta theta values associated with them
    std::vector<std::pair<Scalar, int>> phis;

    // Add our 0 point at the bottom if that is a valid location
    if (shape.phi(Scalar(0.0), h) < Scalar(M_PI_2)) { phis.emplace_back(Scalar(0.0), 1); }

    // Loop from directly down up to the horizon (if phi is nan it will stop)
    // So we don't have a single point at the base, we move half a jump forward
    // The final step is when phi is >= M_PI_2 or NaN, both of which compare false here
    for (Scalar phi = shape.phi(Scalar(0.0), h); phi < Scalar(M_PI_2);) {

      // Calculate our theta
      Scalar theta = shape.theta(phi, h);

      // We don't add NaN thetas
      if (!std::isnan(theta)) {
        // Push back the phi, and the number of whole shapes we can fit
        phis.emplace_back(phi, static_cast<unsigned int>(std::ceil(Scalar(2.0) * M_PI / theta)));
      }

      // Calculate our next phi value
      phi = shape.phi(phi, h);
    }

    // Add our 0 point at the bottom if that is a valid location
    if (Scalar(M_PI) + shape.phi(M_PI, h) > Scalar(M_PI_2)) { phis.emplace_back(Scalar(M_PI), 1); }

    // Loop from directly up down to the horizon
    // The final step is when phi is <= M_PI_2 or NaN, both of which compare false here
    for (Scalar phi = shape.phi(M_PI, h); phi > Scalar(M_PI_2);) {

      // Calculate our theta
      Scalar theta = shape.theta(phi, h);

      // Don't push ban NaN thetas
      if (!std::isnan(theta)) {
        // Push back the phi, and the number of whole shapes we can fit
        phis.emplace_back(phi, static_cast<unsigned int>(std::ceil(Scalar(2.0) * M_PI / theta)));
      }

      // Calculate our next phi value
      phi = shape.phi(phi, h);
    }

    // Sort the list by phi to create a contiguous area
    std::sort(phis.begin(), phis.end());

    // Work out how big our LUT will be
    unsigned int lut_size = 0;
    for (const auto& v : phis) {
      lut_size += v.second;
    }
    nodes.reserve(lut_size);

    // The start and end of each row in the final lut
    rows.reserve(phis.size());

    // Loop through our LUT and calculate our left and right neighbours
    for (const auto& v : phis) {

      // Get our phi and delta theta values for a clean circle
      const auto& phi      = v.first;
      const Scalar sin_phi = std::sin(phi);
      const Scalar cos_phi = std::cos(phi);
      const auto& steps    = v.second;
      const Scalar dtheta  = (Scalar(2.0) * M_PI) / steps;

      // We will use the start position of each row later for linking the graph
      rows.emplace_back(phi, nodes.size(), nodes.size() + steps);

      // Generate for each of the theta values from 0 to 2 pi
      Scalar theta = 0;
      for (int i = 0; i < steps; ++i) {
        Node<Scalar> n;

        // Calculate our unit vector with x facing forward and z up
        n.ray = {{
          std::cos(theta) * sin_phi,  //
          std::sin(theta) * sin_phi,  //
          -cos_phi,                   //
          Scalar(0.0)                 //
        }};

        // Get the indices for our left/right neighbours relative to this row
        // If the next/previous point wraps around, we wrap it here
        const int l = i == 0 ? steps - 1 : i - 1;
        const int r = i == steps - 1 ? 0 : i + 1;

        // Set these two neighbours and default the others to ourself
        n.neighbours[0] = 0;
        n.neighbours[1] = 0;
        n.neighbours[2] = l - i;  // L
        n.neighbours[3] = r - i;  // R
        n.neighbours[4] = 0;
        n.neighbours[5] = 0;

        // Move on to the next theta value
        theta += dtheta;

        nodes.push_back(std::move(n));
      }
    }


    /**
     * This function links to the previous and next rows
     *
     * @param i       the absolute index to the node we are linking
     * @param pos     the position of this node in its row as a value between 0 and 1
     * @param start   the start of the row to link to
     * @param size    the size of the row we are linking to
     * @param offset  the offset for our neighbour (0 for TL,TR 4 for BL BR)
     */
    auto link = [](std::vector<Node<Scalar>>& lut,
                   const int& i,
                   const Scalar& pos,
                   const int& start,
                   const int& size,
                   const unsigned int offset) {
      // Grab our current node
      auto& node = lut[i];

      // If the size of the row we are linking to is 1, all elements will link to it
      // This is the case for the very first and very last row
      if (size == 1) {

        // Now use these to set our TL and TR neighbours
        node.neighbours[offset]     = start - i;
        node.neighbours[offset + 1] = start - i;
      }
      else {
        // Work out if we are closer to the left or right and make an offset var for it
        // Note this bool is used like a bool and int. It is 0 when we should access TR first
        // and 1 when we should access TL first. This is to avoid accessing values which wrap around
        // and instead access a non wrap element and use its neighbours to work out ours
        const bool left = pos > Scalar(0.5);

        // Get our closest neighbour on the previous row and use it to work out where the other one
        // is This will be the Right element when < 0.5 and Left when > 0.5
        const int o1 = start + std::floor(pos * size + !left);  // Use `left` to add one to one
        const int o2 = o1 + lut[o1].neighbours[2 + left];       // But not the other

        // Now use these to set our TL and TR neighbours
        node.neighbours[offset]     = (left ? o1 : o2) - i;
        node.neighbours[offset + 1] = (left ? o2 : o1) - i;
      }
    };

    // Now we iterate upwards and downwards to fill in the missing links
    for (unsigned int r = 1; r + 1 < rows.size(); ++r) {

      // Alias for convenience
      const auto& prev    = rows[r - 1];
      const auto& current = rows[r];
      const auto& next    = rows[r + 1];

      // Work out how big our rows are if they are within valid indices
      const int prev_size    = prev.end - prev.begin;
      const int current_size = current.end - current.begin;
      const int next_size    = next.end - next.begin;

      // Go through all the nodes on our current row
      for (int i = current.begin; i < current.end; ++i) {

        // Find where we are in our row as a value between 0 and 1
        const Scalar pos = Scalar(i - current.begin) / Scalar(current_size);

        // Perform both links
        link(nodes, i, pos, prev.begin, prev_size, 0);
        link(nodes, i, pos, next.begin, next_size, 4);
      }
    }

    // Now we have to deal with the very first, and very last rows as they can't be linked in the normal way
    if (!rows.empty()) {
      const auto& front    = rows.front();
      const int front_size = front.end - front.begin;

      const auto& row_2    = rows.size() > 1 ? rows[1] : rows.front();
      const int row_2_size = row_2.end - row_2.begin;

      const auto& back    = rows.back();
      const int back_size = back.end - back.begin;

      const auto& row_2_last    = rows.size() > 1 ? rows[rows.size() - 1] : rows.back();
      const int row_2_last_size = row_2_last.end - row_2_last.begin;

      // Link to our next row in a circle
      if (front_size == 1) {
        Scalar delta(Scalar(row_2_size) / Scalar(6.0));
        auto& n = nodes.front().neighbours;
        for (int i = 0; i < 6; ++i) {
          // Get the position on the next row
          n[i] = row_2.begin + int(std::round(delta * i));
        }
      }

      // Link to our next row in a circle
      if (back_size == 1) {
        Scalar delta(Scalar(row_2_last_size) / Scalar(6.0));
        auto& n = nodes.back().neighbours;
        for (int i = 0; i < 6; ++i) {
          // Get the position on the previous row
          n[i] = row_2_last.begin + int(std::round(delta * i)) - (nodes.size() - 1);
        }
      }
    }

    // Convert all the relative indices we calculated to absolute indices
    for (unsigned int i = 0; i < nodes.size(); ++i) {
      for (auto& n : nodes[i].neighbours) {
        n = i + n;
      }
    }
  }

  /**
   * Performs a visual mesh lookup, finding the start and end indexes for visual mesh points that are on the screen.
   * It uses the provided `theta_limits` function to identify the start and end points on the screen for a specific phi.
   *
   * @param theta_limits the function that is used to identify the start and end of theta for a phi value
   *
   * @tparam Func        the type of the function that identifies theta ranges given a phi value
   */
  template <typename Func>
  std::vector<std::pair<unsigned int, unsigned int>> lookup(Func&& theta_limits) const {

    std::vector<std::pair<unsigned int, unsigned int>> indices;

    // Loop through each phi row
    for (const auto& row : rows) {

      const auto row_size = row.end - row.begin;

      // Get the theta values that are valid for this phi
      const auto theta_ranges = theta_limits(row.phi);

      // Work out what this range means in terms of theta
      for (const auto& range : theta_ranges) {

        // Convert our theta values into local indices
        int begin = std::ceil(row_size * range.first * (Scalar(1.0) / (Scalar(2.0) * M_PI)));
        int end   = std::ceil(row_size * range.second * (Scalar(1.0) / (Scalar(2.0) * M_PI)));

        // Floating point numbers are annoying... did you know pi * 1/pi is slightly larger than 1?
        // It's also possible that our theta ranges cross the wrap around but the indices mean they don't
        // This will cause segfaults unless we fix the wrap
        begin = begin > row_size ? 0 : begin;
        end   = end > row_size ? row_size : end;

        // If we define an empty range don't bother doing any more
        if (begin != end) {
          // If we define a nice enclosed range range add it
          if (begin < end) { indices.emplace_back(row.begin + begin, row.begin + end); }
          // Our phi values wrap around so we need two ranges
          else {
            indices.emplace_back(row.begin, row.begin + end);
            indices.emplace_back(row.begin + begin, row.end);
          }
        }
      }
    }

    return indices;
  }

  std::vector<std::pair<unsigned int, unsigned int>> lookup_rectilinear(const mat4<Scalar>& Hoc,
                                                                        const Lens<Scalar>& lens) const {

    // We multiply a lot of things by 2
    constexpr const Scalar x2 = Scalar(2.0);

    // Extract our rotation matrix
    const mat3<Scalar> Roc = {{
      {{Hoc[0][0], Hoc[0][1], Hoc[0][2]}},  //
      {{Hoc[1][0], Hoc[1][1], Hoc[1][2]}},  //
      {{Hoc[2][0], Hoc[2][1], Hoc[2][2]}}   //
    }};

    // Extract the camera vector
    const std::array<Scalar, 3> cam = {{Hoc[0][0], Hoc[1][0], Hoc[2][0]}};

    // Work out how much additional y and z we get from our field of view if we have a focal length of 1
    const Scalar y_extent = (lens.dimensions[0] * static_cast<Scalar>(0.5)) / lens.focal_length;
    const Scalar z_extent = (lens.dimensions[1] * static_cast<Scalar>(0.5)) / lens.focal_length;

    // The centre offset in this space
    const Scalar y_offset = -lens.centre[0] / lens.focal_length;
    const Scalar z_offset = -lens.centre[1] / lens.focal_length;

    /* The labels for each of the corners of the frustum is shown below.
        ^    T       U
        |        C
        z    W       V
        <- y
     */
    // Make vectors to the corners in cam space, making sure to apply
    const std::array<vec3<Scalar>, 4> rNCc = {{
      {{Scalar(1.0), +y_extent + y_offset, +z_extent + z_offset}},  // rTCc
      {{Scalar(1.0), -y_extent + y_offset, +z_extent + z_offset}},  // rUCc
      {{Scalar(1.0), -y_extent + y_offset, -z_extent + z_offset}},  // rVCc
      {{Scalar(1.0), +y_extent + y_offset, -z_extent + z_offset}}   // rWCc
    }};

    // Rotate these into world space by multiplying by the rotation matrix
    const std::array<vec3<Scalar>, 4> rNCo = {{
      {{dot(rNCc[0], Roc[0]), dot(rNCc[0], Roc[1]), dot(rNCc[0], Roc[2])}},  // rTCo
      {{dot(rNCc[1], Roc[0]), dot(rNCc[1], Roc[1]), dot(rNCc[1], Roc[2])}},  // rUCo
      {{dot(rNCc[2], Roc[0]), dot(rNCc[2], Roc[1]), dot(rNCc[2], Roc[2])}},  // rVCo
      {{dot(rNCc[3], Roc[0]), dot(rNCc[3], Roc[1]), dot(rNCc[3], Roc[2])}},  // rWCo
    }};

    // Make our corner to next corner vectors
    // In cam space these are 0,1,0 style vectors so we just get a col of the other matrix
    // But since we are multiplying by the transpose we get a row of the matrix
    // When we are storing this matrix we represent each corner as N and the following clockwise corner
    // as M Then it is multiplied by the extent to make a vector of the length of the edge of the
    // frustum
    const std::array<vec3<Scalar>, 4> rMNo = {{
      {{-Roc[0][1] * x2 * y_extent, -Roc[1][1] * x2 * y_extent, -Roc[2][1] * x2 * y_extent}},  // rUTo
      {{-Roc[0][2] * x2 * z_extent, -Roc[1][2] * x2 * z_extent, -Roc[2][2] * x2 * z_extent}},  // rVUo
      {{+Roc[0][1] * x2 * y_extent, +Roc[1][1] * x2 * y_extent, +Roc[2][1] * x2 * y_extent}},  // rWVo
      {{+Roc[0][2] * x2 * z_extent, +Roc[1][2] * x2 * z_extent, +Roc[2][2] * x2 * z_extent}}   // rTWo
    }};

    // Make our normals to the frustum edges
    const std::array<vec3<Scalar>, 4> edges = {{
      cross(rNCo[0], rNCo[1]),  // Top edge
      cross(rNCo[1], rNCo[2]),  // Left edge
      cross(rNCo[2], rNCo[3]),  // Base edge
      cross(rNCo[3], rNCo[0]),  // Right edge
    }};

    // These calculations are intermediates for the solution to the cone/line equation. Since these
    // parts are the same for all phi values, we can pre-calculate them here to save effort later
    std::array<std::array<Scalar, 6>, 4> eq_parts;
    for (int i = 0; i < 4; ++i) {
      const auto& o = rNCo[i];  // Line origin
      const auto& d = rMNo[i];  // Line direction

      // Later we will use these constants like so
      // (p[0] + c2 * p[1] ± sqrt(c2 * p[2] + p[3]))/(p[4] + c2 * p[5]);

      // c2 dependant part of numerator
      eq_parts[i][0] = d[2] * o[2];  // -dz oz

      // Non c2 dependant part of numerator
      eq_parts[i][1] = -d[1] * o[1] - d[0] * o[0];  // -dy oy - dx ox

      // c2 dependant part of discriminant
      eq_parts[i][2] = d[0] * d[0] * o[2] * o[2]         // dx^2 oz^2
                       - x2 * d[0] * d[2] * o[0] * o[2]  // 2 dx dz ox oz
                       + d[1] * d[1] * o[2] * o[2]       // dy^2 oz^2
                       - x2 * d[1] * d[2] * o[1] * o[2]  // 2 dy dz oy oz
                       + d[2] * d[2] * o[0] * o[0]       // d_z^2 o_x^2
                       + d[2] * d[2] * o[1] * o[1];      // d_z^2 o_y^2

      // non c2 dependant part of discriminant
      eq_parts[i][3] = -d[0] * d[0] * o[1] * o[1]        // dx^2 oy^2
                       + x2 * d[0] * d[1] * o[0] * o[1]  // 2 dx dy ox oy
                       - d[1] * d[1] * o[0] * o[0];      // dy^2 ox^2

      // c2 dependant part of denominator
      eq_parts[i][4] = -d[2] * d[2];  // -(dz^2)

      // non c2 dependant part of denominator
      eq_parts[i][5] = d[0] * d[0] + d[1] * d[1];  // dx^2 + dy^2
    }

    // Calculate our theta limits
    auto theta_limits = [&](const Scalar& phi) {
      // Precalculate some trigonometric functions
      const Scalar sin_phi = std::sin(phi);
      const Scalar cos_phi = std::cos(phi);
      const Scalar tan_phi = std::tan(phi);

      // Cone gradient squared
      const Scalar c2 = tan_phi * tan_phi;

      // Store any limits we find
      std::vector<Scalar> limits;

      // Count how many complex solutions we get
      int complex_sols = 0;

      for (int i = 0; i < 4; ++i) {
        // We make a line origin + ray to define a parametric line
        // Note that both of these vectors are always unit length
        const auto& o = rNCo[i];  // Line origin
        const auto& d = rMNo[i];  // Line direction

        // Calculate the first half of our numerator
        const Scalar num = c2 * eq_parts[i][0] + eq_parts[i][1];

        // Calculate our discriminant.
        const Scalar disc = c2 * eq_parts[i][2] + eq_parts[i][3];

        // Calculate our denominator
        const Scalar denom = c2 * eq_parts[i][4] + eq_parts[i][5];

        // We need to count how many complex solutions we get, if at least 3 are complex phi is totally enclosed
        // We also don't care about the case with one solution (touching an edge)
        if (disc <= Scalar(0.0)) { ++complex_sols; }
        else if (denom != Scalar(0.0)) {

          // We have two intersections with either the upper or lower cone
          Scalar root = std::sqrt(disc);

          // Get our two solutions for t
          for (const Scalar t : {(num + root) / denom, (num - root) / denom}) {

            // Check we are within the valid range for our segment.
            // Since we set the length of the direction vector to the length of the side we can
            // check it's less than one
            if (t >= Scalar(0.0) && t <= Scalar(1.0)) {

              // We check z first to make sure it's on the correct side
              const Scalar z = o[2] + d[2] * t;

              // If we are both above, or both below the horizon
              if ((z > Scalar(0.0)) == (phi > M_PI_2)) {

                const Scalar x     = o[0] + d[0] * t;
                const Scalar y     = o[1] + d[1] * t;
                const Scalar theta = std::atan2(y, x);
                // atan2 gives a result from -pi -> pi, we need 0 -> 2 pi
                limits.emplace_back(theta > 0 ? theta : theta + M_PI * Scalar(2.0));
              }
            }
          }
        }
      }

      // If at least 3 solutions are complex we totally enclose the phi however we still need to check the cone is on
      // the correct side. 3 is appropriate here as 3 would mean only 1 intersection (just touching)
      if (complex_sols > 3 && ((cos_phi > Scalar(0.0)) == (cam[2] < Scalar(0.0)))) {

        // Make a test unit vector that is on the cone, theta=0 is easiest
        const vec3<Scalar> test_vec = {{sin_phi, Scalar(0.0), -cos_phi}};

        bool external = false;
        for (int i = 0; !external && i < 4; ++i) {
          // If we get a negative dot product our point is external
          external = Scalar(0.0) > dot(test_vec, edges[i]);
        }
        if (!external) {
          return std::vector<std::pair<Scalar, Scalar>>(1, std::make_pair(Scalar(0.0), Scalar(2.0) * M_PI));
        }
      }
      // If we have a sane number of intersections
      else if (limits.size() >= 2) {
        // Sort the limits
        std::sort(limits.begin(), limits.end());

        // Get a test point half way between the first two points
        const Scalar test_theta = (limits[0] + limits[1]) * Scalar(0.5);
        const Scalar sin_theta  = std::sin(test_theta);
        const Scalar cos_theta  = std::cos(test_theta);

        // Make a unit vector from the phi and theta
        const vec3<Scalar> test_vec = {{cos_theta * sin_phi, sin_theta * sin_phi, -cos_phi}};

        bool first_is_end = false;
        for (int i = 0; !first_is_end && i < 4; ++i) {
          // If we get a negative dot product our first point is an end segment
          first_is_end = Scalar(0.0) > dot(test_vec, edges[i]);
        }

        // If we have an odd number of solutions, we need to remove one of them (the one that should be just touching)
        // Now that we know if the first solution is the start or end of the block we can remove the appropriate point
        if (limits.size() == 3) {
          if (first_is_end) {
            // Remove the 3rd element
            limits.erase(std::next(limits.end(), -1));
          }
          else {
            // Remove the 2nd element
            limits.erase(std::next(limits.begin(), 1));
          }
        }

        // If this is entering, point 0 is a start, and point 1 is an end
        std::vector<std::pair<Scalar, Scalar>> output;
        for (unsigned int i = first_is_end ? 1 : 0; i + 1 < limits.size(); i += 2) {
          output.emplace_back(limits[i], limits[i + 1]);
        }
        if (first_is_end) { output.emplace_back(limits.back(), limits.front()); }
        return output;
      }

      // Default to returning an empty list
      return std::vector<std::pair<Scalar, Scalar>>();
    };

    return lookup(theta_limits);
  }

  std::vector<std::pair<unsigned int, unsigned int>> lookup_fisheye(const mat4<Scalar>& Hoc,
                                                                    const Lens<Scalar>& lens) const {
    // Solution for intersections on the edge is the intersection between a unit sphere, a plane, and a
    // cone. The cone is the cone made by the phi angle, and the plane intersects with the unit sphere to
    // form the circle that defines the edge of the field of view of the camera.
    //
    // Unit sphere
    // x^2 + y^2 + z^2 = 1
    //
    // Cone (don't need to check side for phi since it's squared)
    // z^2 = (x^2+y^2)/c^2
    // c = tan(phi)
    //
    // Plane
    // N = the unit vector in the direction of the camera
    // r_0 = N * cos(fov/2)
    // N . (r - r_0) = 0
    //
    // To simplify things however, we remove the y component and assume the camera vector is only ever
    // on the x/z plane. We calculate the offset to make this happen and re apply it at the end

    // The gradient of our field of view cone
    const Scalar cos_half_fov = std::cos(lens.fov * Scalar(0.5));
    const vec3<Scalar> cam    = {{Hoc[0][0], Hoc[1][0], Hoc[2][0]}};

    // The cameras inclination from straight down (same reference frame as phi)
    const Scalar cam_inc  = std::acos(-cam[2]);
    const Scalar half_fov = lens.fov * 0.5;

    auto theta_limits = [&](const Scalar& phi) -> std::array<std::pair<Scalar, Scalar>, 1> {
      // Check if we are intersecting with an upper or lower cone
      const bool upper = phi > M_PI_2;

      // First we should check if this phi is totally contained in our fov
      // Work out what our largest fully contained phi value is
      // We can work this out by subtracting our offset angle from our fov and checking if phi is
      // smaller
      if ((upper && half_fov - (M_PI - cam_inc) > M_PI - phi) || (!upper && half_fov - cam_inc > phi)) {
        return {{std::make_pair(Scalar(0.0), Scalar(2.0) * M_PI)}};
      }
      // Also if we can tell that the phi is totally outside we can bail out early
      // To check this we check phi is greater than our inclination plus our fov
      if ((upper && half_fov + (M_PI - cam_inc) < M_PI - phi) || (!upper && half_fov + cam_inc < phi)) {
        return {{std::make_pair(Scalar(0.0), Scalar(0.0))}};
      }

      // The solution only works for camera vectors that lie in the x/z plane
      // So we have to rotate our vector into that space, solve it and then rotate them back
      // Normally this would be somewhat unsafe as cam[1] and cam[0] could be both 0
      // However, that case is resolved by the checks above that confirm we intersect
      const Scalar offset     = std::atan2(cam[1], cam[0]);
      const Scalar sin_offset = std::sin(offset);
      const Scalar cos_offset = std::cos(offset);

      // Now we must rotate our cam vector before doing the solution
      // Since y will be 0, and z doesn't change we only need this one
      const Scalar r_x = cam[0] * cos_offset + cam[1] * sin_offset;

      // The z component of our solution
      const Scalar z = -std::cos(phi);

      // Calculate intermediate products
      const Scalar a = Scalar(1.0) - z * z;  // aka sin^2(phi)
      const Scalar x = (cos_half_fov - cam[2] * z) / r_x;

      // The y component is ± this square root
      const Scalar y_disc = a - x * x;

      if (y_disc < 0) { return {{std::make_pair(Scalar(0.0), Scalar(0.0))}}; }

      const Scalar y  = std::sqrt(y_disc);
      const Scalar t1 = offset + std::atan2(-y, x);
      const Scalar t2 = offset + std::atan2(y, x);

      return {{std::make_pair(t1 > Scalar(0.0) ? t1 : t1 + Scalar(2.0) * M_PI,
                              t2 > Scalar(0.0) ? t2 : t2 + Scalar(2.0) * M_PI)}};
    };

    // Lookup the mesh
    return lookup(theta_limits);
  }

  std::vector<std::pair<unsigned int, unsigned int>> lookup(const mat4<Scalar>& Hoc, const Lens<Scalar>& lens) const {

    // Cut down how many points we send here by calculating how many will be on screen
    switch (lens.projection) {
      case RECTILINEAR: return lookup_rectilinear(Hoc, lens);
      case EQUIDISTANT:
      case EQUISOLID: return lookup_fisheye(Hoc, lens);
      default: { throw std::runtime_error("Unknown lens type"); }
    }
  }

  /// The lookup table for this mesh
  std::vector<Node<Scalar>> nodes;
  /// A set of individual rows for phi values.
  /// `begin` and `end` refer to the table with end being 1 past the end
  std::vector<Row<Scalar>> rows;
};

}  // namespace visualmesh

#endif  // VISUALMESH_MESH_HPP
