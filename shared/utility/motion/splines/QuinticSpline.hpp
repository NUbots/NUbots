/*
 * Copyright (c) Hamburg Bit-Bots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
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
 *
 * This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
 * The original files can be found at:
 * https://github.com/Rhoban/model/
 */

#ifndef UTILITY_MOTION_SPLINES_QUINTICSPLINE_HPP
#define UTILITY_MOTION_SPLINES_QUINTICSPLINE_HPP

#include <Eigen/Core>

namespace utility::motion::splines {

    template <typename Scalar>
    class QuinticSpline {
    public:
        /**
         * @brief Construct a quintic spline with duration and start and end waypoints.
         * @param start_waypoint Start waypoint (start position, start velocity, start acceleration).
         * @param end_waypoint End waypoint (end position, end velocity, end acceleration).
         * @param t Time at which to compute the position.
         */
        QuinticSpline(const Eigen::Matrix<Scalar, 3, 1> start_waypoint,
                      const Eigen::Matrix<Scalar, 3, 1> end_waypoint,
                      const Scalar& t) {
            // Compute the coefficients of the quintic polynomial.
            if (t <= static_cast<Scalar>(1e-5)) {
                throw std::logic_error("QuinticSpline invalid duration: " + std::to_string(t));
            }
            const Scalar t2 = t * t;
            const Scalar t3 = t2 * t;
            const Scalar t4 = t3 * t;
            const Scalar t5 = t4 * t;
            coefficients[0] = start_waypoint[0];
            coefficients[1] = start_waypoint[1];
            coefficients[2] = start_waypoint[2] / 2;
            coefficients[3] = -(-end_waypoint[2] * t2 + 3 * start_waypoint[2] * t2 + 8 * end_waypoint[1] * t
                                + 12 * start_waypoint[1] * t - 20 * end_waypoint[0] + 20 * start_waypoint[0])
                              / (2 * t3);
            coefficients[4] = (-2 * end_waypoint[2] * t2 + 3 * start_waypoint[2] * t2 + 14 * end_waypoint[1] * t
                               + 16 * start_waypoint[1] * t - 30 * end_waypoint[0] + 30 * start_waypoint[0])
                              / (2 * t4);
            coefficients[5] = -(-end_waypoint[2] * t2 + start_waypoint[2] * t2 + 6 * end_waypoint[1] * t
                                + 6 * start_waypoint[1] * t - 12 * end_waypoint[0] + 12 * start_waypoint[0])
                              / (2 * t5);
        }


        /**
         * @brief Compute the position at a given time.
         * @param time Time at which to compute the position.
         * @return Position vector at the given time.
         */
        Scalar position(Scalar time) const {
            Scalar tt       = 1;
            Scalar position = 0;
            for (size_t i = 0; i < size_t(coefficients.size()); ++i) {
                position += coefficients[i] * tt;
                tt *= time;
            }
            return position;
        }

        /**
         * @brief Compute the velocity at a given time.
         * @param t Time at which to compute the velocity.
         * @return Velocity at the given time.
         */
        Scalar velocity(Scalar time) const {
            Scalar tt       = 1;
            Scalar velocity = 0;
            for (size_t i = 1; i < size_t(coefficients.size()); ++i) {
                velocity += coefficients[i] * i * tt;
                tt *= time;
            }
            return velocity;
        }

        /**
         * @brief Compute the acceleration at a given time.
         * @param t Time at which to compute the acceleration.
         * @return Acceleration at the given time.
         */
        Scalar acceleration(Scalar time) const {
            Scalar tt           = 1;
            Scalar acceleration = 0;
            for (size_t i = 2; i < size_t(coefficients.size()); ++i) {
                acceleration += coefficients[i] * i * (i - 1) * tt;
                tt *= time;
            }
            return acceleration;
        }

    private:
        /// @brief Spline Coefficients
        Eigen::Matrix<Scalar, 6, 1> coefficients = Eigen::Matrix<Scalar, 6, 1>::Zero();
    };

}  // namespace utility::motion::splines

#endif  // MOTION_QUINTICSPLINE_HPP
