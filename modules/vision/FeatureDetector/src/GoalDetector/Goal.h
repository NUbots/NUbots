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


#ifndef MODULES_VISION_GOAL_H
#define MODULES_VISION_GOAL_H

#include <nuclear>
#include <armadillo>

#include "messages/vision/VisionObjects.h"

#include "../VisionFieldObject.h"
#include "../Quad.h"
#include "../VisionKinematics.h"
#include "utility/math/Line.h"

namespace modules {
    namespace vision {

        class Goal : public VisionFieldObject {
        public:
            friend class GoalDetector_RANSAC;
            Goal(const VisionKinematics& visionKinematics, messages::vision::Goal::Type id = messages::vision::Goal::Type::UNKNOWN, const Quad& corners = Quad(), bool known = false);

            void setParameters(bool THROWOUT_SHORT_GOALS_, 
                               bool THROWOUT_NARROW_GOALS_, 
                               bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS_, 
                               bool THROWOUT_DISTANT_GOALS_, 
                               float MAX_GOAL_DISTANCE_, 
                               int MIN_GOAL_HEIGHT_, 
                               int MIN_GOAL_WIDTH_, 
                               float GOAL_WIDTH_, 
                               const DISTANCE_METHOD& GOAL_DISTANCE_METHOD_,
                               int EDGE_OF_SCREEN_MARGIN_);

            void setBase(const VisionKinematics& visionKinematics, arma::vec2 base);

            //! @brief reutns the pixel locations of the corners.
            const Quad& getQuad() const;

            /*!
              @brief pushes the goal to the external field objects.
              @param fieldobjects a pointer to the global list of field objects.
              @param timestamp the image timestamp.
              @return the success of the operation.
              */
            bool addToExternalFieldObjects(std::unique_ptr<messages::vision::Goal> goal) const;

            //! @brief applies a series of checks to decide if the goal is valid.
            bool check() const;
                
            virtual double findScreenError(VisionFieldObject* other) const;
            virtual double findGroundError(VisionFieldObject* other) const;

            //! @brief output stream operator.
            friend std::ostream& operator<< (std::ostream& output, const Goal& g);

            //! @brief output stream operator for a vector of goals.
            friend std::ostream& operator<< (std::ostream& output, const std::vector<Goal>& g);    
            
        private:
            /*!
              @brief calculates various positions values of the goal.
              @return whether the goal is valid.
              */
            bool calculatePositions(const VisionKinematics& visionKinematics);

            /*!
              @brief calculates distance to the goal based on the global goal distance metric.
              @param bearing the angle between the goal and the image centre in the xy plane.
              @param elevation the angle between the goal and the image centre in the xz plane.
              @return the distance to the goal in cm.
              */
        //    double distanceToGoal(double bearing, double elevation);
            
            bool calculateSphericalError
            (const VisionKinematics& visionKinematics);

            bool m_known;

            Quad m_corners;                                       //! @variable pixel locations of the corners
            NUPoint m_d2pLocation, m_widthLocation, m_heightLocation;
            double m_widthDistance, m_heightDistance;
            bool m_offTop, m_offBottom, m_offSide;

            bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS;                 //! Whether to throw out goals whose base is above the kinematics horizon.
            bool THROWOUT_DISTANT_GOALS;                          //! Whether to throw out goals too far away.
            bool THROWOUT_NARROW_GOALS;                           //! Whether to throw out goals that are too narrow.
            bool THROWOUT_SHORT_GOALS;                            //! Whether to throw out goals that are too short.

            float MAX_GOAL_DISTANCE;                              //! How far away a goal has to been to be ignored.
            float GOAL_WIDTH;                                     //! The physical width of the goal posts in cm
            float GOAL_HEIGHT;

            int MIN_GOAL_WIDTH;                                   //! The minimum width of a goal.
            int MIN_GOAL_HEIGHT;                                  //! The minimum height of a goal.

            DISTANCE_METHOD GOAL_DISTANCE_METHOD;

            int EDGE_OF_SCREEN_MARGIN;

            messages::vision::Goal::Type m_goalType;

        //public:
        //    double width_dist,
        //           d2p;
        };

    }
}

#endif // MODULES_VISION_GOAL_H
