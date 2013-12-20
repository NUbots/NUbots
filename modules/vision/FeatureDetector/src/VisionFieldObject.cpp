/*
 * This file is part of NUBots FeatureDetector.
 *
 * NUBots FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "VisionFieldObject.h"

namespace modules {
    namespace vision {

        VisionFieldObject::VisionFieldObject() {
            // Empty constructor.
        }

        VisionFieldObject::~VisionFieldObject() {
            // Empty destructor.
        }

        VFO_ID VisionFieldObject::getID() const {
            return m_id;
        }

        std::string VisionFieldObject::getName() const  {
            return VFOName(m_id);
        }

        bool VisionFieldObject::isValid() const {
            return valid;
        }

        const NUPoint& VisionFieldObject::getLocation() const {
            return m_location;
        }

        arma::vec2 VisionFieldObject::getScreenSize() const {
            return m_sizeOnScreen;
        }

        arma::vec3 VisionFieldObject::getRelativeFieldCoords() const {
            return m_location.neckRelativeRadial;
        }

        arma::vec2 VisionFieldObject::getLocationPixels() const {
            return m_location.screenCartesian;
        }

        arma::vec2 VisionFieldObject::getLocationAngular() const {
            return m_location.screenAngular;
        }

        bool VisionFieldObject::isGoal(VFO_ID id) {
            return ((id >= GOAL_L) && (id <= GOAL_U));
        }

        std::string VisionFieldObject::VFOName(VFO_ID id) {
            switch (id) {
                case BALL: {
                    return "BALL";
                }

                case FIELDLINE: {
                    return "FIELDLINE";
                }

                case CORNER: {
                    return "CORNER";
                }

                case CENTRE_CIRCLE: {
                    return "CENTRE_CIRCLE";
                }

                case OBSTACLE: {
                    return "OBSTACLE";
                }

                case GOAL_L: {
                    return "GOAL_L";
                }

                case GOAL_R: {
                    return "GOAL_R";
                }

                case GOAL_U: {
                    return "GOAL_U";
                }

                case GOAL_Y_L: {
                    return "GOAL_Y_L";
                }

                case GOAL_Y_R: {
                    return "GOAL_Y_R";
                }

                case GOAL_Y_U: {
                    return "GOAL_Y_U";
                }

                case GOAL_B_L: {
                    return "GOAL_B_L";
                }

                case GOAL_B_R: {
                    return "GOAL_B_R";
                }

                case GOAL_B_U: {
                    return "GOAL_B_U";
                }

                case INVALID:
                default: {
                    return "INVALID";
                }
            }
        }

        VFO_ID VisionFieldObject::VFOFromName(const std::string& name) {
            if (name.compare("BALL") == 0) {
                return BALL;
            }

            else if (name.compare("FIELDLINE") == 0) {
                return FIELDLINE;
            }

            else if (name.compare("CORNER") == 0) {
                return CORNER;
            }

            else if (name.compare("CENTRE_CIRCLE") == 0) {
                return CENTRE_CIRCLE;
            }

            else if (name.compare("GOAL_L") == 0) {
                return GOAL_L;
            }

            else if (name.compare("GOAL_R") == 0) {
                return GOAL_R;
            }

            else if (name.compare("GOAL_U") == 0) {
                return GOAL_U;
            }

            else if (name.compare("GOAL_Y_L") == 0) {
                return GOAL_Y_L;
            }

            else if (name.compare("GOAL_Y_R") == 0) {
                return GOAL_Y_R;
            }

            else if (name.compare("GOAL_Y_U") == 0) {
                return GOAL_Y_U;
            }

            else if (name.compare("GOAL_B_L") == 0) {
                return GOAL_B_L;
            }

            else if (name.compare("GOAL_B_R") == 0) {
                return GOAL_B_R;
            }

            else if (name.compare("GOAL_B_U") == 0) {
                return GOAL_B_U;
            }

            else if (name.compare("INVALID") == 0) {
                return INVALID;
            }

            else {
                return INVALID;
            }

        }

        DistanceMethod VisionFieldObject::getDistanceMethodFromName(const std::string& name) {
            if (name.compare("WIDTH") == 0) {
                return Width;
            }

            else if (name.compare("D2P") == 0) {
                return D2P;
            }

            else if (name.compare("LEAST") == 0) {
                return Least;
            }

            else if (name.compare("AVERAGE") == 0) {
                return Average;
            }

            else {
                return D2P;
            }
        }

        std::string VisionFieldObject::getDistanceMethodName(const DistanceMethod& method) {
            switch (method) {
                case Width: {
                    return "WIDTH";
                }

                case D2P: {
                    return "D2P";
                }

                case Average: {
                    return "AVERAGE";
                }

                case Least: {
                    return "LEAST";
                }

                default: {
                    return "UNKOWN";
                }
            }
        }

    }
}
