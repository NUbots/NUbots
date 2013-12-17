/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "FeatureDetector.h"

namespace modules {
    namespace vision {

        using messages::vision::ClassifiedImage;
        
        FeatureDetector::FeatureDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) { 
            // Load feature detector constants.
            on<Trigger<Configuration<FeatureDetectorConfig>>>([this](const Configuration<FeatureDetectorConfig>& constants) {
                DETECT_LINES = constants.config["DETECT_LINES"];
                DETECT_GOALS = constants.config["DETECT_GOALS"];
                DETECT_BALLS = constants.config["DETECT_BALLS"];
                DETECT_OBSTACTLES = constants.config["DETECT_OBSTACTLES"];
                
                if(DETECT_LINES) {
                    detect_line_objects.enable();
                } else {
                    detect_line_objects.disable();
                }

                if(DETECT_GOALS) {
                    detect_goals.enable();
                } else {
                    detect_goals.disable();
                }

                if(DETECT_BALLS) {
                    detect_balls.enable();
                } else {
                    detect_balls.disable();
                }

                if(DETECT_OBSTACTLES) {
                    detect_obstacles.enable();
                } else {
                    detect_obstacles.disable();
                }

            });

            detect_line_objects = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {

            });

            detect_goals = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {

            });

            detect_balls = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {

            });

            detect_obstacles = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {

            });
        }
    }  // vision
}  // modules
