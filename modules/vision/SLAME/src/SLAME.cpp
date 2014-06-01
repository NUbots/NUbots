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
#include <cmath>
#include "SLAMEModule.h"


namespace modules {
    namespace vision {

        using utility::vision::ORBFeatureExtractor;
        using utility::vision::MockFeatureExtractor;
        using messages::input::Image;
        using messages::localisation::Self;
        using messages::support::Configuration;
        using messages::input::Sensors;

        SLAME::SLAME(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), ORBModule(), MockSLAMEModule() {

            on<Trigger<Configuration<SLAME>>>([this](const Configuration<SLAME>& config) {
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                std::string featureExtractorName = config["FEATURE_EXTRACTOR_TYPE"];
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);

                if(featureExtractorName.compare("ORB") == 0){
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                } else if(featureExtractorName.compare("LSH") == 0) {
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::LSH;
                } else if(featureExtractorName.compare("MOCK") == 0) {
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::MOCK;
                    fakeLocalisationHandle.enable();
                } else {
                    NUClear::log<NUClear::WARN>("SLAME - BAD CONFIG STRING: Loading default ORB feature detector.");
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                }
            });
            
            on<Trigger<Configuration<MockFeatureExtractor>>>([this](const Configuration<MockFeatureExtractor>& config) {
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                FAKE_LOCALISATION_PERIOD = config["FAKE_LOCALISATION_CONFIG"];
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                FAKE_LOCALISATION_RADIUS = config["FAKE_LOCALISATION_RADIUS"];
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                MockSLAMEModule.setParameters(config);
            });

            on<Trigger<Configuration<ORBFeatureExtractor>>>([this](const Configuration<ORBFeatureExtractor>& config) {
                ORBModule.setParameters(config);
            });

            on<Trigger<Image>, With<std::vector<Self>, Sensors>>("SLAME", [this](const Image& image, const std::vector<Self>& selfs, const Sensors& sensors){               
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                switch(FEATURE_EXTRACTOR_TYPE){
                    case (FeatureExtractorType::ORB):
                        emit(ORBModule.getSLAMEObjects(image, selfs[0], sensors));
                        break;
                    case (FeatureExtractorType::LSH):
                        break;
                    case (FeatureExtractorType::MOCK):
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                        emit(MockSLAMEModule.getSLAMEObjects(image, selfs[0], sensors));
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                        break;
                }
            });

            fakeLocalisationHandle = on<Trigger<Every<30, std::chrono::milliseconds>>>("Fake Localisation", [this](const time_t&){
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                auto selfs = std::make_unique<std::vector<Self>>(1);                
                auto& s = selfs->back();
                NUClear::clock::time_point now = NUClear::clock::now();
                NUClear::clock::duration t = now - start_time;
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                s.position = arma::vec2({FAKE_LOCALISATION_RADIUS * std::cos(2 * M_PI * std::chrono::duration_cast<std::chrono::seconds>(t).count() / FAKE_LOCALISATION_PERIOD), 
                                         FAKE_LOCALISATION_RADIUS * std::sin(2 * M_PI * std::chrono::duration_cast<std::chrono::seconds>(t).count() / FAKE_LOCALISATION_PERIOD)});
                s.heading = arma::vec2({cos(2 * M_PI * std::chrono::duration_cast<std::chrono::seconds>(t).count() / FAKE_LOCALISATION_PERIOD), 
                                        sin(2 * M_PI * std::chrono::duration_cast<std::chrono::seconds>(t).count() / FAKE_LOCALISATION_PERIOD)});
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                s.sr_xx = 0.01;
                s.sr_xy = 0;
                s.sr_yy = 0.01;
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
                emit(std::move(selfs));
            });
            fakeLocalisationHandle.disable();
            start_time = NUClear::clock::now();
            // debugHandle = on<Trigger<Every<10, Per<std::chrono::seconds>>>>([this] (const time_t& now, const Sensors& sensors) {
            //     switch(FEATURE_EXTRACTOR_TYPE){
            //         case (FeatureExtractorType::ORB):
            //             emit(ORBModule.testSLAME(sensors));
            //             break;
            //         case (FeatureExtractorType::LSH):
            //             break;
            //         case (FeatureExtractorType::MOCK):
            //             break;
            //     }
            // });
        }
    }
}
