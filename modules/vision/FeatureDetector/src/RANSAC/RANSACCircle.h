/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_RANSACCIRCLE_H
#define MODULES_VISION_RANSACCIRCLE_H

#include <vector>
#include <armadillo>

#include "../NUPoint.h"

namespace modules {
    namespace vision {

        template<typename T>
        class RANSACCircle {
        public:
            RANSACCircle() : m_centre(0,0), m_radius(0) {
                // Empty constructor.
            }

            bool regenerate(const std::vector<T>& points) {
                if (points.size() == minPointsForFit()) {
                    return constructFromPoints(points[0], points[1], points[2], 1.0e-2);
                }

                else {
                    return false;
                }
            }

            unsigned int minPointsForFit() const {
                return 3;
            }

            double calculateError(T p) const {
                return std::abs(arma::norm(p.screen - m_centre.screen, 2) - m_radius);
            }

            double getRadius() const {
                return m_radius;
            }
            
            T getCentre() const {
                return m_centre;
            }

        private:
            bool constructFromPoints(const T& point1, const T& point2, const T& point3, double tolerance = 1.0e-6) {
                T ab = point1 - point2;
                T bc = point2 - point3;
                double det = ((ab[0] * bc[1]) - (bc[0] * ab[1]));

                if (std::abs(det) < tolerance) {
                    return false;
                }

                // double b_len_sqr = p2.squareAbs();
                double b_len_sqr = arma::dot(point2, point2);

                double ab_norm = (arma::dot(point1, point1) - b_len_sqr) / 2.0;
                double bc_norm = (b_len_sqr - arma::dot(point3, point3)) / 2.0;

                det = 1 / det;
                m_centre[0] = ((ab_norm * bc[1]) - (bc_norm * ab[1])) * det;
                m_centre[1] = ((ab[0] * bc_norm) - (bc[0] * ab_norm)) * det;

                m_radius = arma::norm(m_centre - point1, 2);

                return true;
            }

        private:
            T m_centre;
            double m_radius;
        };

        template<>
        class RANSACCircle<NUPoint> {
        public:
            RANSACCircle() : m_radius(0) {
                // Empty constructor.
            }

            bool regenerate(const std::vector<NUPoint>& points) {
                if (points.size() == minPointsForFit()) {
                    return constructFromPoints(points[0], points[1], points[2], 1.0e-2);
                }

                else {
                    return false;
                }
            }

            unsigned int minPointsForFit() const {
                return 3;
            }

            double calculateError(const NUPoint& point) const {
                return std::abs(arma::norm(point.groundCartesian - m_centre.groundCartesian, 2) - m_radius);
            }

            double getRadius() const {
                return m_radius;
            }

            NUPoint getCentre() const {
                return m_centre;
            }

        private:
            bool constructFromPoints(const NUPoint& point1, const NUPoint& point2, const NUPoint& point3, double tolerance = 1.0e-6) {
                arma::vec2 pa = point1.groundCartesian;
                arma::vec2 pb = point2.groundCartesian;
                arma::vec2 pc = point3.groundCartesian;
                arma::vec2 ab = pa - pb;
                arma::vec2 bc = pb - pc;
                double det = ((ab[0] * bc[1]) - (bc[0] * ab[1]));

                if (std::abs(det) < tolerance) {
                    return false;
                }

                // double b_len_sqr = pb.squareAbs();
                double b_len_sqr = arma::dot(pb, pb);

                double ab_norm = (arma::dot(pa, pa) - b_len_sqr) / 2.0;
                double bc_norm = (b_len_sqr - arma::dot(pc, pc)) / 2.0;

                det = 1.0 / det;
                m_centre.groundCartesian[0] = ((ab_norm * bc[1]) - (bc_norm * ab[1])) * det;
                m_centre.groundCartesian[1] = ((ab[0] * bc_norm) - (bc[0] * ab_norm)) * det;

                m_radius = arma::norm(m_centre.groundCartesian - pa, 2);

                return true;
            }

        private:
            NUPoint m_centre;
            double m_radius;
        };

    }
}

#endif // MODULES_VISION_RANSACCIRCLE_H
