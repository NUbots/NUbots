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

#include "CornerDetector.h"

namespace modules {
    namespace vision {

        using messages::vision::CornerPoint::Type;

        CornerDetector::CornerDetector() {
           // Empty constructor. 
        }

        void CornerDetector::setParamerters(double TOLERANCE_) {
            TOLERANCE = std::max(std::min(TOLERANCE_, 1.0), 0.0);
        }

        std::vector<CornerPoint> CornerDetector::run(const std::vector<FieldLine>& lines) const {
            std::vector<FieldLine>::const_iterator it1, it2;
            std::vector<CornerPoint> results;

            if (lines.size() < 2) {
                return results;
            }

            if ((TOLERANCE < 0) || (TOLERANCE > 1)) {
                NUClear::log<NUClear::ERROR>("CornerDetector::run called with invalid tolerance: ",  TOLERANCE, " must be in [0, 1].");
                return results;
            }

            for (std::vector<FieldLIne>::const_iterator line = lines.begin(); line != (lines.end() - 1); line++) {
                Line line1 = line->getGroundLineEquation();
                arma::vec2<NUPoint> line1Points = line->getEndPoints();

                for (std::vector<FieldLine>::const_iterator nextLine = (line + 1); nextLine != lines.end(); nextLine++) {
                    Line line2 = nextLine->getGroundLineEquation();
                    arma::vec2<NUPoint> line2Points = nextLine->getEndPonts();

                    if (line1.getAngleBetween(line2) > ((1 - TOLERANCE) * Math::pi() * 0.5)) {
                        // Nearly perpendicular.
                        // Now build corner from end points.
                        NUPoint intersection;

                        if (line1.getIntersection(line2, intersection.groundCartesian)) {
                            CornerPoint::TYPE type = findCorner(line1Points, line2Points, intersection, TOLERANCE);

                            if (type != CornerPoint::INVALID) {
                                // Need screen locaction.
                                if (line->getScreenLineEquation().getIntersection(nextLine->getScreenLineEquation(), intersection.screenCartesian)) {
                                    results.push_back(CornerPoint(type, intersection));
                                }

                                else {
                                    NUClear::log<NUClear::ERROR>("CornerDetector::run - no intersection found for screen lines - ",
                                                                 "transforms are probably not valid, not publishing corner\t", 
                                                                 line->getScreenLineEquation(), "\t", nextLine->getScreenLineEquation());
                                }
                            }
                        }
                    }
                }
            }

            return results;
        }

        Type CornerDetector::findCorner(const std::vector<NUPoint>& ep1, std::vector<NUPoint>& ep2, const NUPoint& intersection, double tolerance) const {
            arma::vec2 mid1 = (ep1[0].groundCartesian + ep1[1].groundCartesian) * 0.5;
            arma::vec2 mid2 = (ep2[0].groundCartesian + ep2[1].groundCartesian) * 0.5;

            // Compare end points and midpoints to see what is closest to the intersection.
            double d1x = arma::norm(intersection.groundCartesian - ep1[0].groundCartesian, 2);
            double d1y = arma::norm(intersection.groundCartesian - ep1[1].groundCartesian, 2);
            double d1m = arma::norm(intersection.groundCartesian - mid1, 2);
            double d2x = arma::norm(intersection.groundCartesian - ep2[0].groundCartesian, 2);
            double d2y = arma::norm(intersection.groundCartesian - ep2[1].groundCartesian, 2);
            double d2m = arma::norm(intersection.groundCartesian - mid2, 2);

            double min1 = std::min(d1m, std::min(d1x, d1y));
            double min2 = std::min(d2m, std::min(d2x, d2y));

            // Removed distance check.
            //if ((min1 < (tolerance * arma::norm(ep1[0].ground - ep1[1].ground, 2))) && (min2 < (tolerance * arma::norm(ep2[0].ground - ep2[1].ground, 2)))) {
                // Check distances are within tolerance of the length of the lines.
                // Perhaps do this later with only 1 line.

                if ((d1m == min1) && (d2m == min2)) {
                    return Type::X;
                }
                
                else if ((d1m == min1) || (d2m == min2)) {
                    return Type::T;
                }

                else {
                    return Type::L;
                }
            //}
            //
            //else {
            //    return Type::INVALID;
            //}
        }

    }
}
