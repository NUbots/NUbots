#include "Goal.h"

namespace modules {
    namespace vision {

        using utility::math::Line;

        Goal::Goal(const VisionKinematics& visionKinematics, messages::vision::Goal::Type id, const Quad &corners, bool known) {
            m_goalType = id;
            m_corners = corners;
            m_known = known;

            m_location.screenCartesian = corners.getBottomCentre();
            m_sizeOnScreen[0] = corners.getAverageWidth();
            m_sizeOnScreen[1] = corners.getAverageHeight();

            //CALCULATE DISTANCE AND BEARING VALS
            valid = (calculatePositions(visionKinematics) && check());
        }

        void Goal::setParameters(bool THROWOUT_SHORT_GOALS_, 
                                    bool THROWOUT_NARROW_GOALS_, 
                                    bool THROWOUT_ON_ABOVE_KIN_HOR_GOALS_, 
                                    bool THROWOUT_DISTANT_GOALS_, 
                                    float MAX_GOAL_DISTANCE_, 
                                    int MIN_GOAL_HEIGHT_, 
                                    int MIN_GOAL_WIDTH_, 
                                    float GOAL_WIDTH_, 
                                    const DISTANCE_METHOD& GOAL_DISTANCE_METHOD_,
                                    int EDGE_OF_SCREEN_MARGIN_) {
            THROWOUT_SHORT_GOALS = THROWOUT_SHORT_GOALS_;
            THROWOUT_NARROW_GOALS = THROWOUT_NARROW_GOALS_;
            THROWOUT_ON_ABOVE_KIN_HOR_GOALS = THROWOUT_ON_ABOVE_KIN_HOR_GOALS_;
            THROWOUT_DISTANT_GOALS = THROWOUT_DISTANT_GOALS_;
            MAX_GOAL_DISTANCE = MAX_GOAL_DISTANCE_;
            MIN_GOAL_HEIGHT = MIN_GOAL_HEIGHT_;
            MIN_GOAL_WIDTH = MIN_GOAL_WIDTH_;
            GOAL_WIDTH = GOAL_WIDTH_;
            GOAL_DISTANCE_METHOD = GOAL_DISTANCE_METHOD_; 
            EDGE_OF_SCREEN_MARGIN = EDGE_OF_SCREEN_MARGIN_;
        }

        void Goal::setBase(const VisionKinematics& visionKinematics,  arma::vec2 base) {
            Line l_line(m_corners.getBottomLeft(), m_corners.getTopLeft());
            Line r_line(m_corners.getBottomRight(), m_corners.getTopRight());

            arma::vec2 l_point = l_line.projectOnto(base);
            arma::vec2 r_point = r_line.projectOnto(base);

            m_location.screenCartesian[0] = (l_point[0] + r_point[0]) * 0.5;
            m_location.screenCartesian[1] = base[1];

            valid = (calculatePositions(visionKinematics) && check());
        }

        const Quad& Goal::getQuad() const {
            return m_corners;
        }



        /*IS THIS METHOD USED ANYWHERE???!?!*/

        /*!
        *   @brief Updates the external field objects with this goal.
        *   @param fieldobjects A pointer to the external field objects.
        *   @param timestamp The current timestamp to apply to the field objects.
        *
        *   This method uses the id of the goal to determine where to place it, it also
        *   includes no checks before placing them, and nor should it. For example, if this is 
        *   called on multiple YellowLeftGoals then localisation will only see the last one.
        */
    	/*bool Goal::addToExternalFieldObjects(std::unique_ptr<std::vector<messages::vision::Goal>> goals) const{
		    std::unique_ptr<std::vector<messages::vision::Goal>> temp = std::unique_ptr<std::vector<messages::vision::Goal>>(new std::vector<messages::vision::Goal>());

		
                temp->sphericalFromNeck = m_location.neckRelativeRadial;
                temp->sphericalError = m_sphericalError;
                temp->screenAngular = m_location.screenAngular;
                temp->screenCartesian = m_location.screenCartesian;
                temp->sizeOnScreen = m_sizeOnScreen;
                temp->timestamp = NUClear::clock::now();

                ball = std::move(temp);
		

            if (valid) {
		    messages::vision::Goal newGoal;

                switch(m_id) {
                    case GOAL_L: {
                        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Left Yellow Post");
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);

                        break;
                    }

                    case GOAL_R: {
                        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Right Yellow Post");
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);

                        break;
                    }

                    case GOAL_U: {
                        newAmbObj = AmbiguousObject(FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN, "Unknown Yellow Post");
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_LEFT_GOALPOST);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_BLUE_RIGHT_GOALPOST);

                        break;
                    }
                    
                    case GOAL_B_L: {
                        fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].UpdateVisualObject(m_location.neckRelativeRadial, m_spericalError, 
                                                                                                                     m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                                                     timestamp);

                        return true;
                    }
                    
                    case GOAL_B_R: {
                        fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].UpdateVisualObject(m_location.neckRelativeRadial, m_spericalError, 
                                                                                                                     m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                                                     timestamp);

                        return true;
                    }
                    
                    case GOAL_Y_L: {
                        fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].UpdateVisualObject(m_location.neckRelativeRadial, m_spericalError, 
                                                                                                                     m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                                                     timestamp);
                        
                        return true;
                    }
                    
                    case GOAL_Y_R: {
                        fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].UpdateVisualObject(m_location.neckRelativeRadial, m_spericalError, 
                                                                                                                     m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                                                                                                     timestamp);
                        
                        return true;
                    }
                    
                    default: {
                        // Invalid object - do not push to fieldobjects.
                        std::cout << "Goal::addToExternalFieldObjects - attempt to add invalid Goal object id: " << VFOName(m_id) << std::endl;

                        return false;
                    }            
                }

                //update ambiguous goal post and add it to ambiguousFieldObjects
                newAmbObj.UpdateVisualObject(m_location.neckRelativeRadial, m_spericalError, 
                                             m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                             timestamp);
                        
                fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);

                return true;
            

            else {
                return false;
            }
        }
*/
        

        bool Goal::check() const {
            // Various throwouts here.
            if (THROWOUT_SHORT_GOALS) {
                if(m_corners.getAverageHeight() <= MIN_GOAL_HEIGHT) {
                    return false;
                }
            }

            if (THROWOUT_NARROW_GOALS) {
                if(m_corners.getAverageWidth() <= MIN_GOAL_WIDTH) {
                    return false;
                }
            }
            //TODO KINHOR
            // Throwout for base below horizon.
         /*   if (THROWOUT_ON_ABOVE_KIN_HOR_GOALS && !(VisionBlackboard::getInstance()->getKinematicsHorizon().IsBelowHorizon(m_location.screenCartesian.x, m_location.screenCartesian.y))) {
                return false;
            }*/

            // Throw out if goal is too far away.
            if (THROWOUT_DISTANT_GOALS && (m_location.neckRelativeRadial[0] > MAX_GOAL_DISTANCE)) {
                return false;
            }
            
            //all checks passed, keep goalpost
            return true;
        }

        double Goal::findScreenError(VisionFieldObject* other) const {
            Goal* g = dynamic_cast<Goal*>(other);

            return (arma::norm(m_location.screenCartesian - g->m_location.screenCartesian, 2) + arma::norm(m_sizeOnScreen - g->m_sizeOnScreen, 2));
        }

        double Goal::findGroundError(VisionFieldObject* other) const {
            Goal* g = dynamic_cast<Goal*>(other);

            return arma::norm(m_location.groundCartesian - g->m_location.groundCartesian, 2);
        }

        /*!
        *   @brief Updates the spherical position, angular location and transformed spherical position.
        *   
        *   This method uses the camera transform and as such if it was not valid when retrieved from the wrapper
        *   this will leave m_transformed_spherical_position at all zeros.
        */
        bool Goal::calculatePositions(const VisionKinematics& visionKinematics){
            int imageWidth = visionKinematics.getImageSize()[0];
            int imageHeight = visionKinematics.getImageSize()[1];

            m_d2pLocation.screenCartesian = m_location.screenCartesian;
            m_widthLocation.screenCartesian = m_location.screenCartesian;
            m_heightLocation.screenCartesian = m_location.screenCartesian;

            // Get distance from width.
            m_widthDistance = ((GOAL_WIDTH * visionKinematics.getCameraDistanceInPixels()) / (m_sizeOnScreen[0]));

            // heightHeight = ((VisionConstants::GOAL_HEIGHT * visionKinematics.getCameraDistanceInPixels()) / (m_sizeOnScreen[1]));

            // D2P
            visionKinematics.calculateRepresentationsFromPixelLocation(m_d2pLocation);

            // WIDTH
            visionKinematics.calculateRepresentationsFromPixelLocation(m_widthLocation, true, m_widthDistance);

            // HEIGHT
            // visionKinematics.calculateRepresentationsFromPixelLocation(heightLocation, true, heightDistance);

            // Check if we are off the edge of the screen by a certain margin.
            m_offTop = (m_location.screenCartesian[1] - m_sizeOnScreen[1]) < EDGE_OF_SCREEN_MARGIN;
            m_offBottom = m_location.screenCartesian[1] >= (imageHeight - EDGE_OF_SCREEN_MARGIN);
            m_offSide = ((m_location.screenCartesian[0] - (0.5 * m_sizeOnScreen[0]) <= EDGE_OF_SCREEN_MARGIN) ||
                            (m_location.screenCartesian[0] + (0.5 * m_sizeOnScreen[0]) >= (imageWidth - EDGE_OF_SCREEN_MARGIN)));

            if (m_offBottom && m_offSide) {
                // We can't tell distance to these goals.
                m_location.neckRelativeRadial = arma::vec3();

                return false;
            }

            else if (m_offBottom || m_offTop) {
                // We can only use width.
                m_location = m_widthLocation;
            }

            else if (m_offSide) {
                // We can only use d2p.
                m_location = m_d2pLocation;
                return false;
            }

            else {
                // Use method of choice.
                switch (GOAL_DISTANCE_METHOD) {
                    case D2P: {
                        m_location = m_d2pLocation;

                        break;
                    }

                    case WIDTH: {
                        m_location = m_widthLocation;

                        break;
                    }

                    case AVERAGE: {
                        // Average distances.
                        m_location.screenCartesian = (m_d2pLocation.screenCartesian + m_widthLocation.screenCartesian) * 0.5;
                        m_location.neckRelativeRadial = (m_d2pLocation.neckRelativeRadial + m_widthLocation.neckRelativeRadial) * 0.5;
                        m_location.screenAngular = (m_d2pLocation.screenAngular + m_widthLocation.screenAngular) * 0.5;
                        m_location.groundCartesian = (m_d2pLocation.groundCartesian + m_widthLocation.groundCartesian) * 0.5;

                        break;
                    }

                    case LEAST: {
                        m_location = ((m_d2pLocation.neckRelativeRadial[0] < m_widthLocation.neckRelativeRadial[0]) ? m_d2pLocation : m_widthLocation);

                        break;
                    }
                }
            }

            return (m_location.neckRelativeRadial[0] > 0.0);
        }

        /*! @brief Stream insertion operator for a single ColourSegment.
         *      The segment is terminated by a newline.
         */
        std::ostream& operator<< (std::ostream& output, const Goal& g) {
            output << "Goal - " << std::endl;
            output << "\tpixelloc: " << g.m_location.screenCartesian << std::endl;
            output << "\tangularloc: " << g.m_location.screenAngular << std::endl;
            output << "\trelative field coords: " << g.m_location.neckRelativeRadial << std::endl;
            output << "\tspherical error: " << g.m_sphericalError << std::endl;
            output << "\tsize on screen: " << g.m_sizeOnScreen;
            return output;
        }

        /*! @brief Stream insertion operator for a std::vector of ColourSegments.
         *      Each segment is terminated by a newline.
         *  @relates ColourSegment
         */
        std::ostream& operator<< (std::ostream& output, const std::vector<Goal>& goals) {
            for (const auto& goal : goals) {
                output << goal << std::endl;        
            }

            return output;
        }

    }
}        
