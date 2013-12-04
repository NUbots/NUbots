/*
 * This file is part of LUTClassifier.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_LUTCLASSIFIER_H
#define MODULES_VISION_LUTCLASSIFIER_H

#include <nuclear> 
#include <armadillo>
#include "messages/input/Image.h"

namespace modules {
    namespace vision {

        /**
         * Classifies a raw image, producing the colour segments for object detection
         *
         * @author Jake Fountain
         */
        class LUTClassifier : public NUClear::Reactor {
        private:
            /*! @brief Computes the visual green horizon.
                Note that the use of kinematics horizon has been replaced by dummmy code 
                @param image The raw image
            */ 
            std::vector<arma::vec> CalculateGreenHorizon(const messages::input::Image& image);
            
            /*! @brief Generates the scan lines
                
            */ 
            std::vector<int> GenerateScanLines(const messages::input::Image& image, const std::vector<arma::vec>& green_horizon_points);
            //ClassifiedImage ClassifyScanLines(std::vector<int> scan_lines);
        public:
            explicit LUTClassifier(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_LUTCLASSIFIER_H

