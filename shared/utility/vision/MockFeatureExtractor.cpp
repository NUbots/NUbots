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
#include "MockFeatureExtractor.h"
#include "utility/motion/kinematics/ForwardKinematics.h"
#include "utility/motion/kinematics/RobotModels.h"
#include <cstdlib>
#include <ctime>

namespace utility {
	namespace vision {

			using utility::motion::kinematics::DarwinModel;
			
			MockFeatureExtractor::MockFeatureExtractor(){}

			MockFeatureExtractor::setParameters(int NUMBER_OF_MOCK_POINTS,
												float MEAN_RADIUS,
												float RADIAL_DEVIATION,
												float HEIGHT,
												float HEIGHT_DEVIATION,
												float ANGULAR_DEVIATION,
												bool RANDOMIZE,
												int SEED
												){
				std::srand(SEED + std::time(0) * int(RANDOMIZE));
				for(int i = 0; i < NUMBER_OF_MOCK_POINTS; i++){
					float r = MEAN_RADIUS + 2 * RADIAL_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5); 
					float z = HEIGHT + 2 * HEIGHT_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5);
					float theta = 2 * M_PI * i / float(NUMBER_OF_MOCK_POINTS) +  2 * ANGULAR_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5);
					mockFeatures.push_back(arma::vec3({
						r * std::cos(theta),
						r * std::sin(theta),
						z
					}));
				}

			}

			std::vector<MockFeatureExtractor::ExtractedFeature> MockFeatureExtractor::extractFeatures(const messages::input::Image& image, const messages::localisation::Self& self, const messages::input::Sensors& sensors){
				//TODO: feature detection
				std::vector<ExtractedFeature> features;

				for (auto point : mockFeatures){
					ExtractedFeature f;
					f.screenAngular = utility::motion::kinematics::worldToCameraAngular<DarwinModel>(point, sensors, self);
					features.push_back(f);					
				}

				return features;
			}

			//Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
			std::vector<std::tuple<int, int, float>> MockFeatureExtractor::matchFeatures(std::vector<ExtractedFeature>& features, 
																					    const std::vector<ExtractedFeature>& newFeatures,
																					    int MAX_MATCHES)
			{
				//TODO: feature matching
				return std::vector<std::tuple<int, int, float>>();
			}
	}
}