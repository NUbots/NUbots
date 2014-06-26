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

#include "LUTClassifier.h"

#include "messages/input/Image.h"
#include "messages/input/CameraParameters.h"
#include "messages/input/Sensors.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/SaveLookUpTable.h"
#include "messages/support/Configuration.h"

#include "QuexClassifier.h"

#include "Lexer.hpp"

namespace modules {
    namespace vision {

        using messages::input::Image;
        using messages::input::ServoID;
        using messages::input::Sensors;
        using messages::input::CameraParameters;
        using messages::vision::LookUpTable;
        using messages::vision::SaveLookUpTable;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::support::Configuration;
        using messages::support::SaveConfiguration;

        void LUTClassifier::insertSegments(ClassifiedImage<ObjectClass>& image, std::vector<ClassifiedImage<ObjectClass>::Segment>& segments, bool vertical) {
            ClassifiedImage<ObjectClass>::Segment* previous = nullptr;
            ClassifiedImage<ObjectClass>::Segment* current = nullptr;

            auto& target = vertical ? image.verticalSegments : image.horizontalSegments;

            for (auto& s : segments) {

                // Move in the data
                current = &(target.insert(std::make_pair(s.colour, std::move(s)))->second);

                // Link up the results
                current->previous = previous;
                if(previous) {
                    previous->next = current;
                }

                // Get ready for our next one
                previous = current;
            }
        }

        LUTClassifier::LUTClassifier(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), quex(new QuexClassifier) {

            on<Trigger<Configuration<LUTLocation>>>([this](const Configuration<LUTLocation>& config) {
                emit(std::make_unique<LookUpTable>(config.config.as<LookUpTable>()));
            });

            on<Trigger<SaveLookUpTable>, With<LookUpTable>>([this](const SaveLookUpTable&, const LookUpTable& lut) {
                emit(std::make_unique<SaveConfiguration>(SaveConfiguration{ LUTLocation::CONFIGURATION_PATH, YAML::Node(lut) }));
            });

            auto setParams = [this] (const CameraParameters& cam, const Configuration<LUTClassifier>& config) {

                // Visual horizon detector
                VISUAL_HORIZON_SPACING = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["spacing"].as<double>());
                VISUAL_HORIZON_BUFFER = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["horizon_buffer"].as<double>());
                VISUAL_HORIZON_SUBSAMPLING = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["subsampling"].as<double>())));
                MINIMUM_VISUAL_HORIZON_SEGMENT_SIZE = cam.effectiveScreenDistancePixels * tan(config["visual_horizon"]["minimum_segment_size"].as<double>());

                // // Goal detector
                GOAL_FINDER_LINE_SPACING = cam.effectiveScreenDistancePixels * tan(config["goals"]["spacing"].as<double>());
                GOAL_FINDER_SUBSAMPLING = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["goals"]["subsampling"].as<double>())));
                GOAL_FINDER_DETECTOR_LEVELS = config["goals"]["detector_levels"].as<std::vector<double>>();

                // Halve our levels
                for(auto& d : GOAL_FINDER_DETECTOR_LEVELS) {
                    d /= 2;
                }

                // // Ball Detector
                MIN_BALL_INTERSECTIONS = config["ball"]["intersections"].as<double>();
                ALPHA = cam.pixelsToTanThetaFactor[1];
                MIN_BALL_SEARCH_JUMP = std::max(1, int(cam.effectiveScreenDistancePixels * tan(config["ball"]["min_jump"].as<double>())));
            };

            // Trigger the same function when either update
            on<Trigger<CameraParameters>, With<Configuration<LUTClassifier>>>(setParams);
            on<With<CameraParameters>, Trigger<Configuration<LUTClassifier>>>(setParams);

            on<Trigger<Image>, With<LookUpTable, Sensors>, Options<Single>>("Classify Image", [this](const Image& image, const LookUpTable& lut, const Sensors& sensors) {

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass>>();

                // Find our horizon
                findHorizon(image, lut, sensors, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, sensors, *classifiedImage);

                // Find our goals
                findGoals(image, lut, sensors, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, sensors, *classifiedImage);

                // Find our ball
                findBall(image, lut, sensors, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, sensors, *classifiedImage);

                emit(std::move(classifiedImage));
            });

        }

        LUTClassifier::~LUTClassifier() {
            // TODO work out how to fix pimpl and fix it damnit!!
            delete quex;
        }

    }  // vision
}  // modules