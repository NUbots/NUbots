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

#ifndef MODULES_VISION_CORNERPOINT_H
#define MODULES_VISION_CORNERPOINT_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/VisionObjects.h"

#include "../VisionFieldObject.h"
#include "../NUPoint.h"

namespace modules {
    namespace vision {

        class CornerPoint : public VisionFieldObject {
        public:
            CornerPoint(const messages::vision::CornerPoint::Type& type, const NUPoint& location);

            virtual bool addToExternalFieldObjects(std::unique_ptr<messages::vision::CornerPoint> cornerPoint) const;

            //! @brief Calculation of error for optimisation
            virtual double findScreenError(VisionFieldObject* other) const;
            virtual double findGroundError(VisionFieldObject* other) const;

            //! @brief output stream operator.
            friend std::ostream& operator<< (std::ostream& output, const CornerPoint& c);

            //! @brief output stream operator for a vector of corner points.
            friend std::ostream& operator<< (std::ostream& output, const std::vector<CornerPoint>& c);

        private:
            messages::vision::CornerPoint::Type m_type;
        };

    }
}

#endif // MODULES_VISION_CORNERPOINT_H
