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

#ifndef MODULES_VISION_FIELDLINE_H
#define MODULES_VISION_FIELDLINE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/VisionObjects.h"
#include "utility/math/Line.h"

#include "../VisionFieldObject.h"
#include "../LSFittedLine.h"
#include "../NUPoint.h"

namespace modules {
    namespace vision {

        class FieldLine : public VisionFieldObject {
        public:
            FieldLine(const LSFittedLine& screenLine, const LSFittedLine& groundLine);
            FieldLine(const std::vector<NUPoint>& endPoints);

            void set(const LSFittedLine& screenLine, const LSFittedLine& groundLine);
            void set(const std::vector<NUPoint>& endPoints);

            utility::math::Line getScreenLineEquation() const;
            utility::math::Line getGroundLineEquation() const;
            std::vector<NUPoint> getEndPoints() const;
            
            // Dummy until localisation can handle lines.
            bool addToExternalFieldObjects(std::unique_ptr<messages::vision::FieldLine> fieldLine) const;

            //! @brief Calculation of error for optimisation
            virtual double findScreenError(VisionFieldObject* other) const;
            virtual double findGroundError(VisionFieldObject* other) const;

            //! @brief output stream operator
            friend std::ostream& operator<< (std::ostream& output, const FieldLine& line);

            //! @brief output stream operator for a std::vector of FieldLines
            friend std::ostream& operator<< (std::ostream& output, const std::vector<FieldLine>& lines);

        private:
            utility::math::Line m_screenLine;
            utility::math::Line m_groundLine;
            std::vector<NUPoint> m_endPoints;
        };

    }
}


#endif // MODULES_VISION_FIELDLINE_H
