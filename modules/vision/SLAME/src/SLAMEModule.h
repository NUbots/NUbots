/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#ifndef MODULES_VISION_SLAME_MODULE_H
#define MODULES_VISION_SLAME_MODULE_H

#include "SLAMEFeature.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Image.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"

namespace modules{
	namespace vision{
 		template <class FeatureDetectorClass>
 		class SLAMEModule{
 		private:
 			std::vector<float> featureStrengths;
            std::vector<FeatureDetectorClass::ExtractedFeature> features;
            std::vector<utility::math::kalman::UKF<utility::math::kalman::InverseDepthPointModel>> featureFilters;

            FeatureDetectorClass featureExtractor;

            NUClear::clock::time_point lastTime;

 		public:
 		 	SLAMEModule();
 		 	std::unique_ptr<messages::vision::SLAMEObjects> getSLAMEObjects(const messages::input::Image& image, const messages::localisation::Self& self, const messages::input::Sensors& sensors);        
 		};
 	}
 }