/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_SMOOTHSPLINE_HPP
#define UTILITY_MOTION_SPLINES_SMOOTHSPLINE_HPP

#include <vector>

#include "Spline.hpp"

namespace utility {
namespace motion {
    namespace splines {

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
                Scalar time;
                Scalar position;
                Scalar velocity;
                Scalar acceleration;
            };

            void reset() {
                points.clear();
                Spline<Scalar>::splines.clear();
            }

            /**
             * Add a new point with its time, position value, velocity and acceleration
             */
            void addPoint(Scalar time,
                          Scalar position,
                          Scalar velocity     = static_cast<Scalar>(0),
                          Scalar acceleration = static_cast<Scalar>(0)) {
                points.push_back({time, position, velocity, acceleration});
                computeSplines();
            }

            /**
             * Access to points container
             */
            const std::vector<Point>& getPoints() const {
                return points;
            }

            std::vector<Point>& getPoints() {
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
                    Scalar time = points[i].time - points[i - 1].time;
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
            virtual void importCallBack() override {
                size_t size = Spline<Scalar>::splines.size();
                if (size == 0) {
                    return;
                }

                Scalar tBegin = Spline<Scalar>::splines.front().min;
                points.push_back(
                    {tBegin, Spline<Scalar>::pos(tBegin), Spline<Scalar>::vel(tBegin), Spline<Scalar>::acc(tBegin)});

                for (size_t i = 1; i < size; i++) {
                    Scalar t1   = Spline<Scalar>::splines[i - 1].max;
                    Scalar t2   = Spline<Scalar>::splines[i].min;
                    Scalar pos1 = Spline<Scalar>::pos(t1);
                    Scalar vel1 = Spline<Scalar>::vel(t1);
                    Scalar acc1 = Spline<Scalar>::acc(t1);
                    Scalar pos2 = Spline<Scalar>::pos(t2);
                    Scalar vel2 = Spline<Scalar>::vel(t2);
                    Scalar acc2 = Spline<Scalar>::acc(t2);

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

                Scalar tEnd = Spline<Scalar>::splines.back().max;
                points.push_back(
                    {tEnd, Spline<Scalar>::pos(tEnd), Spline<Scalar>::vel(tEnd), Spline<Scalar>::acc(tEnd)});
            }

        private:
            /**
             * Points container
             */
            std::vector<Point> points;

            /**
             * Fit a polynom between 0 and t with given pos, vel and acc initial and final conditions
             */
            Polynom<Scalar> polynomFit(Scalar t,
                                       Scalar pos1,
                                       Scalar vel1,
                                       Scalar acc1,
                                       Scalar pos2,
                                       Scalar vel2,
                                       Scalar acc2) const {
                if (t <= static_cast<Scalar>(0.00001)) {
                    throw std::logic_error("SmoothSpline invalid spline interval");
                }
                Scalar t2 = t * t;
                Scalar t3 = t2 * t;
                Scalar t4 = t3 * t;
                Scalar t5 = t4 * t;
                Polynom<Scalar> p;
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

    }  // namespace splines
}  // namespace motion
}  // namespace utility

#endif
