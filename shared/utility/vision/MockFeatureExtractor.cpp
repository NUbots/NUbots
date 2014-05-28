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
#include "utility/math/matrix.h"
#include "messages/input/ServoID.h"
#include <cstdlib>
#include <ctime>

namespace utility {
	namespace vision {
			bool operator==(const MockFeatureExtractor::ExtractedFeature& lhs, const MockFeatureExtractor::ExtractedFeature& rhs){return (lhs.featureID == rhs.featureID);}

			using messages::input::ServoID;

			MockFeatureExtractor::MockFeatureExtractor(){}

			void MockFeatureExtractor::setParameters(const messages::support::Configuration<MockFeatureExtractor>& config){
				int NUMBER_OF_MOCK_POINTS = config["NUMBER_OF_MOCK_POINTS"];
				float MEAN_RADIUS = config["MEAN_RADIUS"];
				float RADIAL_DEVIATION = config["RADIAL_DEVIATION"];
				float HEIGHT = config["HEIGHT"];
				float HEIGHT_DEVIATION = config["HEIGHT_DEVIATION"];
				float ANGULAR_DEVIATION = config["ANGULAR_DEVIATION"];
				bool RANDOMIZE = config["RANDOMIZE"];
				int SEED = config["SEED"];

				FOV_X = config["FOV_X"];
				FOV_Y = config["FOV_Y"];

				std::srand(SEED + std::time(0) * int(RANDOMIZE));
				for(int i = 0; i < NUMBER_OF_MOCK_POINTS; i++){
					float r = MEAN_RADIUS + 2 * RADIAL_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5); 
					float z = HEIGHT + 2 * HEIGHT_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5);
					float theta = 2 * M_PI * i / float(NUMBER_OF_MOCK_POINTS) +  2 * ANGULAR_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5);
					mockFeatures.push_back(arma::vec4({
						r * std::cos(theta),
						r * std::sin(theta),
						z,
						1								
					}));
				}

			}

			std::vector<MockFeatureExtractor::ExtractedFeature> MockFeatureExtractor::extractFeatures(const messages::input::Image& image, const messages::localisation::Self& self, const messages::input::Sensors& sensors){
				
					std::vector<ExtractedFeature> features;
					arma::vec2 selfHeading = arma::normalise(self.heading);
					arma::mat44 robotToWorld_world = arma::mat44({   		  selfHeading[0],   		   -selfHeading[1], 0,            	 	self.position[0],
	                                                                 		  selfHeading[1],   		    selfHeading[0], 0,            	 	self.position[1],
	                                                                                        0,                            0, 1,         sensors.bodyCentreHeight,
	                                                                                        0,                            0, 0,                                1});

	                arma::mat44 cameraToBody_body = sensors.forwardKinematics.at(ServoID::HEAD_PITCH);

	                arma::mat44 robotToBody_body = arma::eye(4,4);
	                
	                robotToBody_body.submat(0,0,2,2) = sensors.orientation;

	                arma::mat44 worldToCamera_camera = utility::math::matrix::orthonormal44Inverse(cameraToBody_body) * robotToBody_body * utility::math::matrix::orthonormal44Inverse(robotToWorld_world);	                

	            int id = 0;
				for (auto point : mockFeatures){
					ExtractedFeature f;
	                arma::vec4 cameraToFeatureVector_cam =  worldToCamera_camera * point;
	                f.screenAngular = arma::vec2({ std::atan2(cameraToFeatureVector_cam[1], cameraToFeatureVector_cam[0]) , std::atan2(cameraToFeatureVector_cam[2], cameraToFeatureVector_cam[0])});
	                f.featureID = id++;
					if(std::fabs(f.screenAngular[0]) < FOV_X/2.0 && std::fabs(f.screenAngular[1]) < FOV_Y/2.0){
						features.push_back(f);
					}
				}
				return features;
			}

			//Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
			std::vector<std::tuple<int, int, float>> MockFeatureExtractor::matchFeatures(std::vector<ExtractedFeature>& features, 
																					    const std::vector<ExtractedFeature>& newFeatures,
																					    size_t MAX_MATCHES)
			{
				std::vector<std::tuple<int, int, float>> matches;
				
				for(size_t newFeatureIndex = 0; newFeatureIndex < newFeatures.size(); newFeatureIndex++){	//For each new feature	
					auto& newFeature = newFeatures[newFeatureIndex];

					for(size_t featureIndex = 0; featureIndex < features.size(); featureIndex++){		//Check if it matches any known features
						auto& feature = features[featureIndex];
						if(newFeature == feature){
							matches.push_back(std::tuple<int,int,float>(featureIndex,newFeatureIndex,1.0));
							break;
						}
						if(featureIndex == features.size()-1 && features.size() < MAX_MATCHES){	//If we dont match any known features, add it too the list if there is space
							features.push_back(newFeature);
							matches.push_back(std::tuple<int,int,float>(featureIndex+1,newFeatureIndex,1.0));

						}
					}
				}
				return matches;
			}
	}
}