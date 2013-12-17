/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "VisionFieldObject.h"

namespace utility {
    namespace vision {

        VisionFieldObject::VisionFieldObject() {
            // Empty constructor.
        }

        virtual VisionFieldObject::~VisionFieldObject() {
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

        virtual arma::vec3 VisionFieldObject::getRelativeFieldCoords() const {
            return m_location.neckRelativeRadial;
        }

        arma::vec2 VisionFieldObject::getLocationPixels() const {
            return m_location.screenCartesian;
        }

        arma::vec2 VisionFieldObject::getLocationAngular() const {
            return m_location.screenAngular;
        }

        bool VisionFieldObject::isGoal(VFO_ID id) const {
            return ((id >= GOAL_L) && (id <= GOAL_U);
        }

        std::string VisionFieldObject::VFOName(VFO_ID id) {
            std::map<VFO_ID, std::pair<int, std::string> >::const_iterator it = vfomap.find(id);

            if(it == vfomap.end()) {
                std::cout << "Invalid VFO_ID passed to VFOName()" << std::endl;
                return -1;
            }

            else {
                return it->second.second;
            }
        }

        VFO_ID VisionFieldObject::VFOFromName(const std::string &name) {
            for (const auto& vfo : vfomap) {
                if (vfo.second.second.compare(name) == 0) {
                    return vfo.first;
                }
            }

            std::cout << "Invalid VFO_ID name passed to VFOFromName()" << std::endl;
            return -1;
        }

        VFO_ID VisionFieldObject::VFOFromInt(int n) {
            for (const auto& vfo : vfomap) {
                if (vfo.second.first == n) {
                    return vfo.first;
                }
            }

            std::cout << "Invalid VFO_ID number passed to VFOFromInt()" << std::endl;
            return -1;
        }

        int VisionFieldObject::intFromVFO(VFO_ID id) {
            std::map<VFO_ID, std::pair<int, std::string>>::const_iterator it = vfomap.find(id);

            if (it == vfomap.end()) {
                std::cout << "Invalid VFO_ID passed to intFromVFO()" << std::endl;
                return -1;
            }

            else {
                return it->second.first;
            }
        }

        int VisionFieldObject::numVFOIDs() {
            return vfomap.size();
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
