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

#include "Ball.h"

namespace modules {
    namespace vision {


        Ball::Ball() {
            m_id = BALL;
            m_diameter = 0;
        }

        Ball::Ball(const arma::vec2& centre, double max_box_size) {
            m_id = BALL;

            double top = centre[1] - (max_box_size * 0.5);
            double bottom = centre[1] + (max_box_size * 0.5);
            double left = centre[0] - (max_box_size * 0.5);
            double right = centre[0] + (max_box_size * 0.5);

            arma::vec2 topPoint;
            arma::vec2 bottomPoint;
            arma::vec2 rightPoint;
            arma::vec2 leftPoint;
            topPoint << (right - left) * 0.5 << top;
            bottomPoint << (right - left) * 0.5 << bottom;
            rightPoint << right << (bottom - top) * 0.5;
            leftPoint << left << (bottom - top) * 0.5;

            m_diameter = std::max(bottomPoint[1] - topPoint[1], rightPoint[0] - leftPoint[0]);
            m_location.screenCartesian = centre;
            m_sizeOnScreen << m_diameter << m_diameter;
        }

		void Ball::setParameters(bool THROWOUT_ON_ABOVE_KIN_HOR_BALL_,
                                float MAX_DISTANCE_METHOD_DISCREPENCY_BALL_,
                                bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL_,
                                bool THROWOUT_SMALL_BALLS_,
                                float MIN_BALL_DIAMETER_PIXELS_,
                                bool THROWOUT_DISTANT_BALLS_,
                                float MAX_BALL_DISTANCE_,
                                float BALL_WIDTH_,
                                const DISTANCE_METHOD& BALL_DISTANCE_METHOD_,
                                const VisionKinematics& visionKinematics) {
			THROWOUT_ON_ABOVE_KIN_HOR_BALL = THROWOUT_ON_ABOVE_KIN_HOR_BALL_;
			MAX_DISTANCE_METHOD_DISCREPENCY_BALL = MAX_DISTANCE_METHOD_DISCREPENCY_BALL_;
			THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL_;
			THROWOUT_SMALL_BALLS = THROWOUT_SMALL_BALLS_;
			MIN_BALL_DIAMETER_PIXELS = MIN_BALL_DIAMETER_PIXELS_;
			THROWOUT_DISTANT_BALLS = THROWOUT_DISTANT_BALLS_;
			MAX_BALL_DISTANCE = MAX_BALL_DISTANCE_;
			BALL_WIDTH = BALL_WIDTH_;
			BALL_DISTANCE_METHOD = BALL_DISTANCE_METHOD_;

            // Internal variables contain valid values at this point.
            // All necessary constants have been set.
            // It is now safe to determine the validity of this Ball object.
            valid = (calculatePositions(visionKinematics) && check());
		}

        float Ball::getRadius() const {
            return (m_diameter * 0.5);
        }

        bool Ball::check() const {
            // Various throwouts here.

            // Throwout for below horizon.
			// TODO: Add kinematics horizon.
            if (THROWOUT_ON_ABOVE_KIN_HOR_BALL && true /* !(VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screenCartesian[0], m_location.screenCartesian[1]))*/ ) {
                std::cout << "Ball::check() - Ball above horizon: should not occur" << std::endl;

                return false;
            }
            
            // //Distance discrepency throwout - if width method says ball is a lot closer than d2p (by specified value) then discard
            // if ((THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL) && (std::abs(width_dist - d2p) > MAX_DISTANCE_METHOD_DISCREPENCY_BALL)) {
            //     return false;
            // }

            // Throw out if ball is too small.
            if (THROWOUT_SMALL_BALLS && (m_diameter < MIN_BALL_DIAMETER_PIXELS)) {
                return false;
            }
            
            // Throw out if ball is too far away.
            if (THROWOUT_DISTANT_BALLS && (m_location.neckRelativeRadial[0] > MAX_BALL_DISTANCE)) {
                return false;
            }
            
            // All checks passed, keep ball.
            return true;
        }

        bool Ball::calculatePositions(const VisionKinematics& visionKinematics) {
            // To the bottom of the Goal Post.
            NUPoint d2pLocation, widthLocation;

            d2pLocation.screenCartesian = m_location.screenCartesian;
            widthLocation.screenCartesian = m_location.screenCartesian;

            double widthDistance = ((BALL_WIDTH * visionKinematics.getCameraDistanceInPixels()) / (m_diameter));

            visionKinematics.calculateRepresentationsFromPixelLocation(d2pLocation);
            visionKinematics.calculateRepresentationsFromPixelLocation(widthLocation, true, widthDistance);

            switch (BALL_DISTANCE_METHOD) {
                case D2P: {
                    m_location = d2pLocation;
                    break;
                }

                case WIDTH: {
                    m_location = widthLocation;

                    break;
                }

                case AVERAGE: {
                    //average distances
                    m_location.screenCartesian      = (d2pLocation.screenCartesian + widthLocation.screenCartesian) * 0.5;
                    m_location.neckRelativeRadial   = (d2pLocation.neckRelativeRadial + widthLocation.neckRelativeRadial) * 0.5;
                    m_location.screenAngular        = (d2pLocation.screenAngular + widthLocation.screenAngular) * 0.5;
                    m_location.groundCartesian      = (d2pLocation.groundCartesian + widthLocation.groundCartesian) * 0.5;

                    break;
                }

                case LEAST: {
                    m_location = ((d2pLocation.neckRelativeRadial[0] < widthLocation.neckRelativeRadial[0]) ? d2pLocation : widthLocation);

                    break;
                }
            }
            m_sphericalError = visionKinematics.calculateSphericalError(m_location, BALL_DISTANCE_METHOD, m_diameter);
            return (m_location.neckRelativeRadial[0] > 0);
        }


        /*!
        *   @brief Calculates the distance using the set METHOD and the provided coordinate angles.
        *   @param bearing The angle about the z axis.
        *   @param elevation The angle about the y axis.
        */
        //double Ball::distanceToBall(double bearing, double elevation) {
        //    //reset distance values
        //    bool d2pvalid = false;
        //    d2p = 0;
        //    width_dist = 0;
        //    double result = 0;
        //    //get distance to point from base

        //    d2pvalid = m_transfomer.isDistanceToPointValid();
        //    if(d2pvalid)
        //        d2p = m_transformer.distanceToPoint(bearing, elevation);

        //    //get distance from width
        //    width_dist = BALL_WIDTH*m_transformer.getCameraDistanceInPixels()/m_sizeOnScreen[0];

        //    switch (BALL_DISTANCE_METHOD) {
        //    case D2P:
        //        distance_valid = d2pvalid && d2p > 0;
        //        result = d2p;
        //        break;
        //    case WIDTH:
        //        distance_valid = true;
        //        result = width_dist;
        //        break;
        //    case AVERAGE:
        //        //average distances
        //        distance_valid = d2pvalid && d2p > 0;
        //        result = (d2p + width_dist) * 0.5;
        //        break;
        //    case LEAST:
        //        distance_valid = d2pvalid && d2p > 0;
        //        result = (distance_valid ? std::min(d2p, width_dist) : width_dist);
        //        break;
        //    }

        //    return result;
        //}

        std::ostream& operator<< (std::ostream& output, const Ball& ball) {
            output << "Ball " << std::endl;
            output << "\tpixelloc: [" << ball.m_location.screenCartesian[0] << ", " << ball.m_location.screenCartesian[1] << "]" << std::endl;
            output << " angularloc: [" << ball.m_location.screenAngular[0] << ", " << ball.m_location.screenAngular[1] << "]" << std::endl;
            output << "\trelative field coords: [" << ball.m_location.neckRelativeRadial[0] << ", " << ball.m_location.neckRelativeRadial[1] << 
                        ", " << ball.m_location.neckRelativeRadial[2] << "]" << std::endl;
            output << "\tspherical error: [" << ball.m_sphericalError[0] << ", " << ball.m_sphericalError[1] <<", "<< ball.m_sphericalError[2] << "]" << std::endl;
            output << "\tsize on screen: [" << ball.m_sizeOnScreen[0] << ", " << ball.m_sizeOnScreen[1] << "]";

            return output;
        }

        std::ostream& operator<< (std::ostream& output, const std::vector<Ball>& balls) {
            for (const auto& ball : balls) {
                output << ball << std::endl;
            }

            return output;
        }

    }
}
