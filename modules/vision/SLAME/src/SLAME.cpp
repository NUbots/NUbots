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

#include "SLAME.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Image.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"

#include "utility/vision/SLAMEFeatureExtractor.h"   //Parent class
#include "utility/vision/ORBFeatureExtractor.h"    //Example subclass of SLAMEFeatureDetetor

#include "SLAMEFeature.h"

namespace modules {
    namespace vision {

        using messages::vision::SLAMEObjects;
        using messages::input::Image;
        using messages::localisation::Self;
        using messages::input::Sensors;

        using utility::vision::ExtractedFeature;
        using utility::vision::FeatureExtractor;


        SLAME::SLAME(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Configuration<SLAME>>>([this](const Configuration<SLAME>& config) {
                
            });

            on<Trigger<Image>, With<Self, Sensors>>([this](const time_t&, const Image& image, const Self& self, const sensors& sensors){

                std::vector<ExtractedFeature> extractedFeatures = featureExtractor.extractFeatures(image);

                //Get matches: format (featureIndex, extractedFeatureIndex, matchStrength)
                //Order of vector is strongest to weakest
                //Add new features here to the feature list and pick up missing filters and strengths below
                std::vector<std::tuple<int, int, float>> matches = featureExtractor.matchFeatures(features, extractedFeatures, MAX_MATCHES);

                auto objectMessage = std::make_unique<std::vector<SLAMEObjects>>();
                double deltaT = (sensors.timestamp - lastTime).count() / double(NUClear::clock::period::den);

                for(auto& match : matches){
                    if(std::get<0>(match) < featureFilters.size()){     //That is, we have seen this object before
                        //Create message about where we have seen the feature
                        objectMessage->push_back();

                        objectMessage->back().expectedFieldLocation = featureFilters[std::get<0>].getFieldCartesian();

                        objectMessage->back().sphericalFromNeck =   extractedFeatures[std::get<1>(match)].getSphericalFromNeck(sensors);
                        objectMessage->back().sphericalError =      extractedFeatures[std::get<1>(match)].getSphericalError(sensors);
                        objectMessage->back().screenAngular =       extractedFeatures[std::get<1>(match)].getScreenAngular(sensors);
                        objectMessage->back().screenCartesian =     extractedFeatures[std::get<1>(match)].getScreenCartesian(sensors);
                        objectMessage->back().sizeOnScreen =        extractedFeatures[std::get<1>(match)].getSizeOnScreen(sensors);
                        objectMessage->back().timestamp =           sensors->timestamp;

                        //Update our beleif
                        featureStrengths[std::get<0>(match)] += std::get<2>(match);
                        featureFilters[std::get<0>(match)].timeUpdate(deltaT);
                        featureFilters[std::get<0>(match)].measurementUpdate(extractedFeatures[std::get<1>(match)], self, sensors);
                    } else {    //Otherwise we initialise a filter for the object
                        featureFilters.resize(std::get<0>(match)+1);
                        featureStrengths.resize(std::get<0>(match)+1);

                        featureFilters[std::get<0>(match)].resetState(extractedFeatures[std::get<1>(match)].getScreenAngular(sensors), self);
                        featureStrengths[std::get<0>(match)] = std::get<2>(match);
                    }
                }

                lastTime = sensors.timestamp;
                emit(std::move(objectMessage));
            });
           
        }
    }
}
