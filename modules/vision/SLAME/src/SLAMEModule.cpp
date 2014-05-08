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

 #include "SLAMEModule.h"

namespace modules{
	namespace vision{
		
		using messages::vision::SLAMEObjects;
        using messages::input::Image;
        using messages::localisation::Self;
        using messages::input::Sensors;

		template <class FeatureDetectorClass>
		SLAMEModule<FeatureDetectorClass>::SLAMEModule():featureExtractor(){
 				lastTime = NUClear::clock::now();
 		}


		template <class FeatureDetectorClass>
		std::unique_ptr<SLAMEObjects> SLAMEModule<FeatureDetectorClass>::getSLAMEObjects(const Image& image, const Self& self, const Sensors& sensors){
			auto objectMessage = std::make_unique<std::vector<SLAMEObjects>>();	            

            std::vector<FeatureDetectorClass::ExtractedFeature> extractedFeatures = featureExtractor.extractFeatures(image);

            //Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
            //Order of vector is strongest to weakest
            //Add new features here to the feature list and pick up missing filters and strengths below
            std::vector<std::tuple<int, int, float>> matches = featureExtractor.matchFeatures(features, extractedFeatures, MAX_MATCHES);

            double deltaT = (sensors.timestamp - lastTime).count() / double(NUClear::clock::period::den);

            for(auto& match : matches){
                if(std::get<0>(match) < featureFilters.size()){     
                	//That is, we have seen this object before
                    //Create message about where we have seen the feature
                    objectMessage->push_back();

                    objectMessage->back().expectedState = featureFilters[std::get<0>(match)].get();
                    objectMessage->back().screenAngular = extractedFeatures[std::get<1>(match)].screenAngular;
                    objectMessage->back().strength = featureStrengths[std::get<0>(match)];
                    objectMessage->back().timestamp = sensors->timestamp;

                    //Update our beleif
                    featureStrengths[std::get<0>(match)] += std::get<2>(match);	//TODO make this better and use strengths
                    featureFilters[std::get<0>(match)].timeUpdate(deltaT);
                    featureFilters[std::get<0>(match)].measurementUpdate(extractedFeatures[std::get<1>(match)].screenAngular, std::pair<const Self&, const Sensors&>(self, sensors));
                } else {    //Otherwise we initialise a filter for the object
                    featureFilters.push_back(utility::math::kalman::UKF<utility::math::kalman::InverseDepthPointModel>());
                    featureStrengths.push_back(std::get<2>(match));

                    //TODO Calculate initial state and covariance
                    //featureFilters[std::get<0>(match)].reset(extractedFeatures[std::get<1>(match)].getScreenAngular(sensors), self);
                }
            }

            //TODO: sort objectMessage by strengths and take top k
            lastTime = sensors.timestamp;

            return std::move(objectMessage);
        }
	}
}