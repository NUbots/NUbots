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

#ifndef UTILITY_MOTION_SPLINES_SMOOTHSPLINE_HPP
#define UTILITY_MOTION_SPLINES_SMOOTHSPLINE_HPP

#include <vector>

#include "Spline.hpp"

namespace utility::motion::splines {

    /**
     * SmoothSpline
     *
     * Implementation of 5th order polynomial splines trajectory known to minimize jerk
     */
    template <typename Scalar>
    class SmoothSpline : public Spline<Scalar> {
    public:
        /**
         * Simple point struture
         */
        struct Point {
            Scalar time         = 0;
            Scalar position     = 0;
            Scalar velocity     = 0;
            Scalar acceleration = 0;
        };

        void reset() {
            points.clear();
            Spline<Scalar>::splines.clear();
        }

        /**
         * Add a new point with its time, position value, velocity and acceleration
         */
        void addPoint(const Scalar& time,
                      const Scalar& position,
                      const Scalar& velocity     = static_cast<Scalar&>(0),
                      const Scalar& acceleration = static_cast<Scalar&>(0)) {
            points.push_back({time, position, velocity, acceleration});
            computeSplines();
        }

        /**
         * Access to points container
         */
        [[nodiscard]] const std::vector<Point>& getPoints() const {
            return points;
        }

        [[nodiscard]] std::vector<Point>& getPoints() {
            return points;
        }

        /**
         * Recompute splines interpolation model
         */
        void computeSplines() {
            Spline<Scalar>::splines.clear();
            if (points.size() < 2) {
                return;
            }

            std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2) -> bool {
                return p1.time < p2.time;
            });

            for (size_t i = 1; i < points.size(); i++) {
                const Scalar time = points[i].time - points[i - 1].time;
                if (time > static_cast<Scalar>(0.00001)) {
                    Spline<Scalar>::splines.push_back({polynomFit(time,
                                                                  points[i - 1].position,
                                                                  points[i - 1].velocity,
                                                                  points[i - 1].acceleration,
                                                                  points[i].position,
                                                                  points[i].velocity,
                                                                  points[i].acceleration),
                                                       points[i - 1].time,
                                                       points[i].time});
                }
            }
        }

    protected:
        /**
         * Inherit Load Points
         */
        void importCallBack() override {
            const size_t size = Spline<Scalar>::splines.size();
            if (size == 0) {
                return;
            }

            const Scalar tBegin = Spline<Scalar>::splines.front().min;
            points.push_back(
                {tBegin, Spline<Scalar>::pos(tBegin), Spline<Scalar>::vel(tBegin), Spline<Scalar>::acc(tBegin)});

            for (size_t i = 1; i < size; i++) {
                const Scalar t1   = Spline<Scalar>::splines[i - 1].max;
                const Scalar t2   = Spline<Scalar>::splines[i].min;
                const Scalar pos1 = Spline<Scalar>::pos(t1);
                const Scalar vel1 = Spline<Scalar>::vel(t1);
                const Scalar acc1 = Spline<Scalar>::acc(t1);
                const Scalar pos2 = Spline<Scalar>::pos(t2);
                const Scalar vel2 = Spline<Scalar>::vel(t2);
                const Scalar acc2 = Spline<Scalar>::acc(t2);

                if (std::abs(t2 - t1) < static_cast<Scalar>(0.0001)
                    && std::abs(pos2 - pos1) < static_cast<Scalar>(0.0001)
                    && std::abs(vel2 - vel1) < static_cast<Scalar>(0.0001)
                    && std::abs(acc2 - acc1) < static_cast<Scalar>(0.0001)) {
                    points.push_back({t1, pos1, vel1, acc1});
                }
                else {
                    points.push_back({t1, pos1, vel1, acc1});
                    points.push_back({t2, pos2, vel2, acc2});
                }
            }

            const Scalar tEnd = Spline<Scalar>::splines.back().max;
            points.push_back({tEnd, Spline<Scalar>::pos(tEnd), Spline<Scalar>::vel(tEnd), Spline<Scalar>::acc(tEnd)});
        }

    private:
        /**
         * Points container
         */
        std::vector<Point> points{};

        /**
         * Fit a polynom between 0 and t with given pos, vel and acc initial and final conditions
         */
        [[nodiscard]] Polynom<Scalar> polynomFit(const Scalar& t,
                                                 const Scalar& pos1,
                                                 const Scalar& vel1,
                                                 const Scalar& acc1,
                                                 const Scalar& pos2,
                                                 const Scalar& vel2,
                                                 const Scalar& acc2) const {
            if (t <= static_cast<Scalar>(0.00001)) {
                throw std::logic_error("SmoothSpline invalid spline interval");
            }
            const Scalar t2 = t * t;
            const Scalar t3 = t2 * t;
            const Scalar t4 = t3 * t;
            const Scalar t5 = t4 * t;
            Polynom<Scalar> p{};
            p.getCoefs().resize(6);
            p.getCoefs()[0] = pos1;
            p.getCoefs()[1] = vel1;
            p.getCoefs()[2] = acc1 / 2;
            p.getCoefs()[3] =
                -(-acc2 * t2 + 3 * acc1 * t2 + 8 * vel2 * t + 12 * vel1 * t - 20 * pos2 + 20 * pos1) / (2 * t3);
            p.getCoefs()[4] =
                (-2 * acc2 * t2 + 3 * acc1 * t2 + 14 * vel2 * t + 16 * vel1 * t - 30 * pos2 + 30 * pos1) / (2 * t4);
            p.getCoefs()[5] =
                -(-acc2 * t2 + acc1 * t2 + 6 * vel2 * t + 6 * vel1 * t - 12 * pos2 + 12 * pos1) / (2 * t5);

            return p;
        }
    };

}  // namespace utility::motion::splines
#endif
