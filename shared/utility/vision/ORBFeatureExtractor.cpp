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
#include "ORBFeatureExtractor.h"

namespace utility {
	namespace vision {
			ORBFeatureExtractor::ORBFeatureExtractor(){}

			std::vector<ORBFeatureExtractor::ExtractedFeature> ORBFeatureExtractor::extractFeatures(const messages::input::Image& image){
				//TODO: feature detection
				return std::vector<ExtractedFeature>();
			}

			//Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
			std::vector<std::tuple<int, int, float>> ORBFeatureExtractor::matchFeatures(std::vector<ExtractedFeature>& features, 
																					    const std::vector<ExtractedFeature>& newFeatures,
																					    int MAX_MATCHES)
			{
				//TODO: feature matching
				return std::vector<std::tuple<int, int, float>>();
			}
	}
}