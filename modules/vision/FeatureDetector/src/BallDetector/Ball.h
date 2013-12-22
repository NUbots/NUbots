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

#ifndef MODULES_VISION_BALL_H
#define MODULES_VISION_BALL_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/VisionObjects.h"

#include "../VisionFieldObject.h"
#include "../VisionKinematics.h"

namespace modules {
    namespace vision {

        class Ball : public VisionFieldObject {
        public:
            Ball();
            Ball(const arma::vec2& centre, double diameter);
            
			void setParameters(bool THROWOUT_ON_ABOVE_KIN_HOR_BALL_,
									float MAX_DISTANCE_METHOD_DISCREPENCY_BALL_,
									bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL_,
									bool THROWOUT_SMALL_BALLS_,
									float MIN_BALL_DIAMETER_PIXELS_,
									bool THROWOUT_DISTANT_BALLS_,
									float MAX_BALL_DISTANCE_,
									float BALL_WIDTH_,
									const DistanceMethod& BALL_DISTANCE_METHOD_,
									const VisionKinematics& transformer);

            /*!
              @brief returns the radius.
              @return the radius of the ball in pixels.
              */
            float getRadius() const;

            /*!
              @brief pushes the ball to the external field objects.
              @param fieldobjects a pointer to the global list of field objects.
              @param timestamp the image timestamp.
              @return the success of the operation.
              */
            bool addToExternalFieldObjects(std::unique_ptr<messages::vision::BallObject> ball) const;

            //! @brief applies a series of checks to decide if the ball is valid.
            bool check() const;
            
            double findScreenError(VisionFieldObject* other) const;
            double findGroundError(VisionFieldObject* other) const;
            
            //! @brief output stream operator
            friend std::ostream& operator<< (std::ostream& output, const Ball& b);

            //! @brief output stream operator for a std::vector of balls
            friend std::ostream& operator<< (std::ostream& output, const std::vector<Ball>& b);
            
        private:
            /*!
              @brief calculates various positions values of the ball.
              @return whether the ball is valid.
              */
            bool calculatePositions();
            
            /*!
              @brief calculates distance to the ball based on the global ball distance metric.
              @param bearing the angle between the ball and the image centre in the xy plane.
              @param elevation the angle between the ball and the image centre in the xz plane.
              @return the distance to the ball in cm.
              */
            //double distanceToBall(double bearing, double elevation);
            
        public:
            int m_diameter;     //! @variable the radius of the ball in pixels
            
			bool THROWOUT_ON_ABOVE_KIN_HOR_BALL;
			float MAX_DISTANCE_METHOD_DISCREPENCY_BALL;
			bool THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL;
			bool THROWOUT_SMALL_BALLS;
			float MIN_BALL_DIAMETER_PIXELS;
			bool THROWOUT_DISTANT_BALLS;
			float MAX_BALL_DISTANCE;
			float BALL_WIDTH;
			DistanceMethod BALL_DISTANCE_METHOD;
			VisionKinematics m_transformer;

            //private:
            //    float d2p;          //! @variable the distance of the ball in cm as found by the distance to point method
            //    float width_dist;   //! @variable the distance of the ball in cm as found by the width method.
        };

    }
}
#endif // MODULES_VISION_BALL_H
