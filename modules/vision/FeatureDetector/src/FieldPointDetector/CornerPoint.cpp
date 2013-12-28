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

#include "CornerPoint.h"

namespace modules {
    namespace vision {

        using messages::vision::CornerPoint::Type;

        CornerPoint::CornerPoint(const Type& type, const NUPoint& location) {
            m_sizeOnScreen  << 3 << 3;
            m_type = type;
            m_location = location;

            valid = (m_location.neckRelativeRadial[0] > 0);
        }

        bool CornerPoint::addToExternalFieldObjects(std::unique_ptr<messages::vision::CornerPoint> cornerPoint) const {
            std::unique_ptr<messages::vision::CornerPoint> temp = std::unique_ptr<messages::vision::CornerPoint>(new messages::vision::CornerPoint());

/*
            if (valid) {
                switch (m_type) {
                    case L: {
                        // labelling inside L since I have no idea which is which - going to match it with other possibilities
                        newAmbObj = AmbiguousObject(FieldObjects::FO_CORNER_UNKNOWN_INSIDE_L, "L Corner");
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_FIELD_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_PEN_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_CIRCLE);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_PEN_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_FIELD_RIGHT);

                        break;
                    }

                    case T: {
                        newAmbObj = AmbiguousObject(FieldObjects::FO_CORNER_UNKNOWN_T, "T Corner");
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_YELLOW_T_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_CENTRE_T_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_BLUE_T_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_YELLOW_RIGHT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_LEFT);
                        newAmbObj.addPossibleObjectID(FieldObjects::FO_CORNER_PROJECTED_T_BLUE_RIGHT);

                        break;
                    }
                    
                    case X: {
                        // At present a cross is ignored as none exist on the field and this indicates poor detection
                        return false;
                    }
                    
                    default: {
                        // Invalid object - do not push to fieldobjects.
                        std::cout << "CornerPoint::addToExternalFieldObjects - attempt to add invalid CornerPoint object id: " << VFOName(m_id) << std::endl;

                        return false;
                    }            
                }

                // Update ambiguous corner and add it to ambiguousFieldObjects.
                newAmbObj.UpdateVisualObject(m_location.neckRelativeRadial, m_sphericalError,
                                             m_location.screenAngular, m_location.screenCartesian, m_sizeOnScreen,
                                             timestamp);

                fieldobjects->ambiguousFieldObjects.push_back(newAmbObj);
*/

                cornerPoint = std::move(temp);

                return true;
            }

            else {
                return false;
            }
        }

        //! @brief Calculation of error for optimisation
        double CornerPoint::findScreenError(VisionFieldObject* other) const {
            CornerPoint* c = dynamic_cast<CornerPoint*>(other);

            return arma::norm(m_location.screenCartesian - c->m_location.screenCartesian, 2);
        }

        double CornerPoint::findGroundError(VisionFieldObject *other) const {
            CornerPoint* c = dynamic_cast<CornerPoint*>(other);

            return arma::norm(m_location.groundCartesian - c->m_location.groundCartesian, 2);
        }

        std::ostream& operator<< (std::ostream& output, const CornerPoint& c) {
            output << "CornerPoint - ";

            switch (c.m_type) {
                case CornerPoint::L: {
                    output << "L";

                    break;
                }

                case CornerPoint::T: {
                    output << "T";

                    break;
                }

                case CornerPoint::X: {
                    output << "X";

                    break;
                }

                default: {
                    output << "INVALID";

                    break;
                }
            }

            output << std::endl << "\tpixelloc: " << c.m_location.screenCartesian << std::endl;
            output << "\tangularloc: " << c.m_location.screenAngular << std::endl;
            output << "\trelative field coords: " << c.m_location.neckRelativeRadial << std::endl;
            output << "\tspherical error: [" << c.m_sphericalError << "]" << std::endl;
            output << "\tsize on screen: [" << c.m_sizeOnScreen << "]";

            return output;
        }

        std::ostream& operator<< (std::ostream& output, const std::vector<CornerPoint>& corners) {
            for (const auto& corner : corners) {
                output << corner << std::endl;
            }

            return output;
        }

    }
}
