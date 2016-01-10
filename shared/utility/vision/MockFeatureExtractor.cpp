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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <armadillo>

#include "MockFeatureExtractor.h"
#include "message/input/ServoID.h"
#include "utility/math/vision.h"
#include "utility/support/yaml_armadillo.h"

namespace utility {
	namespace vision {
		bool operator==(const MockFeatureExtractor::ExtractedFeature& lhs, const MockFeatureExtractor::ExtractedFeature& rhs){return (lhs.featureID == rhs.featureID);}

		using message::input::ServoID;

		MockFeatureExtractor::MockFeatureExtractor(){}

		std::vector<MockFeatureExtractor::MockFeature> MockFeatureExtractor::setParameters(const YAML::Node& config){
			int NUMBER_OF_MOCK_POINTS = config["NUMBER_OF_MOCK_POINTS"].as<int>();

			arma::vec RADIUS = config["RADIUS"].as<arma::vec>();
			arma::vec HEIGHT = config["HEIGHT"].as<arma::vec>();

			float ANGULAR_DEVIATION = config["ANGULAR_DEVIATION"].as<float>();
			bool RANDOMIZE = config["RANDOMIZE"].as<bool>();
			int SEED = config["SEED"].as<int>();

			arma::vec FALSE_NEGATIVE_PROB = config["FALSE_NEGATIVE_PROB"].as<arma::vec>();
			arma::vec MISCLASSIFIED_PROB = config["MISCLASSIFIED_PROB"].as<arma::vec>();
			arma::vec MISCLASSIFIED_AS_NEW_PROB = config["MISCLASSIFIED_AS_NEW_PROB"].as<arma::vec>();

			MAX_DISTINCT_FALSE_FEATURES = config["MAX_DISTINCT_FALSE_FEATURES"].as<int>();

			FOV_X = config["FOV_X"].as<float>();
			FOV_Y = config["FOV_Y"].as<float>();

			mockFeatures.clear();
			std::srand(SEED + std::time(0) * int(RANDOMIZE));
			NUClear::log("Generating mock features:");
			for(int i = 0; i < NUMBER_OF_MOCK_POINTS; i++){
				float r = uniformSample(RADIUS);
				float z = uniformSample(HEIGHT);
				float theta = 2 * M_PI * i / float(NUMBER_OF_MOCK_POINTS) +  2 * ANGULAR_DEVIATION * (std::rand() / float(RAND_MAX) - 0.5);
				mockFeatures.push_back({ arma::vec({r * std::cos(theta), r * std::sin(theta), z, 1}),
										 uniformSample(FALSE_NEGATIVE_PROB),
										 uniformSample(MISCLASSIFIED_PROB),
										 uniformSample(MISCLASSIFIED_AS_NEW_PROB),
										 i+1
										});
				std::cout << i+1 << " " << mockFeatures.back().position.at(0) << " "
				                        << mockFeatures.back().position.at(1) << " "
				                        << mockFeatures.back().position.at(2) << " "
				                        <<(1-mockFeatures.back().FALSE_NEGATIVE_PROB)*(1-mockFeatures.back().MISCLASSIFIED_PROB) << std::endl;
			}
			return mockFeatures;
		}

		std::vector<MockFeatureExtractor::ExtractedFeature> MockFeatureExtractor::extractFeatures(const message::localisation::Self& self, const message::input::Sensors& sensors){
			std::vector<MockFeatureExtractor::ExtractedFeature> features;
			arma::mat worldToCamera_camera = utility::math::vision::calculateWorldToCameraTransform(sensors, self);

			for (auto feature : mockFeatures){

				ExtractedFeature f;

                arma::vec cameraToFeatureVector_cam =  worldToCamera_camera * feature.position;
                f.screenAngular = utility::math::vision::screenAngularFromDirectionVector(cameraToFeatureVector_cam.rows(0,3));
                f.screenPosition = utility::math::vision::screenPositionFromDirectionVector(cameraToFeatureVector_cam.rows(0,3));
                f.numberOfTimesUpdated = 1;

                if(sampleRandomBool(feature.MISCLASSIFIED_PROB)){
                	f.MISCLASSIFIED_PROB = (1-feature.MISCLASSIFIED_AS_NEW_PROB) / float(mockFeatures.size());	//Assumes classifier knows when it is really wrong
                	if(sampleRandomBool(feature.MISCLASSIFIED_AS_NEW_PROB)){
                		f.featureID = numberOfFalseFeaturesDetected;
                		numberOfFalseFeaturesDetected = 1+(numberOfFalseFeaturesDetected) % MAX_DISTINCT_FALSE_FEATURES;
                	} else{
                		f.featureID = 1 + std::rand() % mockFeatures.size();
                	}
                } else {
                	f.featureID = feature.id;
                	f.MISCLASSIFIED_PROB = feature.MISCLASSIFIED_PROB;
                }

				if(std::fabs(f.screenAngular[0]) < FOV_X/2.0 && std::fabs(f.screenAngular[1]) < FOV_Y/2.0 && !sampleRandomBool(feature.FALSE_NEGATIVE_PROB)){
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
																				    size_t MAX_MATCHES/*TODO change to max search depth or something*/)
		{
			std::vector<std::tuple<int, int, float>> matches;

			for(size_t newFeatureIndex = 0; newFeatureIndex < newFeatures.size(); newFeatureIndex++){	//For each new feature
				auto& newFeature = newFeatures[newFeatureIndex];

				for(size_t featureIndex = 0; featureIndex <= features.size(); featureIndex++){		//Check if it matches any known features
					if(featureIndex == features.size() && features.size() < MAX_MATCHES){	//If we dont match any known features, add it too the list if there is space
						features.push_back(newFeature);
						matches.push_back(std::tuple<int,int,float>(featureIndex,newFeatureIndex,1-newFeature.MISCLASSIFIED_PROB));
						break;
					}
					auto& feature = features[featureIndex];
					if(newFeature.featureID == feature.featureID){
						matches.push_back(std::tuple<int,int,float>(featureIndex,newFeatureIndex,1-newFeature.MISCLASSIFIED_PROB));
						break;
					}
				}
			}
			return matches;
		}

		double MockFeatureExtractor::uniformSample(arma::vec/*2*/ range){
			double random = (std::rand() / float(RAND_MAX));
			return range[0]*(1-random) + range[1]*(random);
		}

		bool MockFeatureExtractor::sampleRandomBool(double probability_true){
			double random = (std::rand() / float(RAND_MAX));
			return random < probability_true;
		}


	}
}