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

namespace modules {
    namespace vision {




        SLAME::SLAME(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), ORBModule() {

            on<Trigger<Configuration<SLAME>>>([this](const Configuration<SLAME>& config) {
                std::string featureExtractorName = config["FEATURE_EXTRACTOR_TYPE"];

                if(featureExtractorName.compare("ORB") == 0){
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                } else if(featureExtractorName.compare("LSH") == 0) {
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::LSH;
                } else {
                    NUClear::log<NUClear::WARN>("SLAME - Loading default ORB feature detector.");
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                }

            });

            on<Trigger<Image>, With<std:vector<Self>, Sensors>>([this](const time_t&, const Image& image, const Self& self, const sensors& sensors){               
                switch(FEATURE_EXTRACTOR_TYPE){
                    case (FeatureExtractorType::ORB):
                        emit(ORBModule.getSLAMEObjects(image, self, sensors));
                        break;
                    case (FeatureExtractorType::LSH):
                        break;
                }
            });
           
        }
    }
}
