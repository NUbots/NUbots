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


#ifndef UTILITY_VISION_MOCK_FEATURE_EXTRACTOR_H
#define UTILITY_VISION_MOCK_FEATURE_EXTRACTOR_H

#include <armadillo>
#include <yaml-cpp/yaml.h>

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/input/Image.h"
#include "message/localisation/FieldObject.h"

namespace utility {
	namespace vision {
		class MockFeatureExtractor{
		public:
			struct MockFeature{
				arma::vec position;
				double FALSE_NEGATIVE_PROB;
				double MISCLASSIFIED_PROB;
				double MISCLASSIFIED_AS_NEW_PROB;
				int id;
			};
		private:
			int numberOfFalseFeaturesDetected = 1;
			int MAX_DISTINCT_FALSE_FEATURES;
			std::vector<MockFeature> mockFeatures;
			double uniformSample(arma::vec/*2*/ range);
			bool sampleRandomBool(double probability_true);
		public:
			std::vector<MockFeature> setParameters(const YAML::Node& config);
			class ExtractedFeature {
			public:
				arma::vec screenAngular;	//Compulsory
				arma::vec screenPosition;
				double MISCLASSIFIED_PROB;
				int featureID;
				int numberOfTimesUpdated;
			};
			MockFeatureExtractor();
			std::vector<ExtractedFeature> extractFeatures(const message::localisation::Self& self, const message::input::Sensors& sensors);

			float FOV_X;
			float FOV_Y;

			//Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
			std::vector<std::tuple<int, int, float>> matchFeatures(std::vector<ExtractedFeature>& features,
																   const std::vector<ExtractedFeature>& newFeatures,
																   size_t MAX_MATCHES);
		};
	}
}

#endif