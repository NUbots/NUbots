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
                VISUAL_HORIZON_SPACING = cam.focalLengthPixels * tan(config["visual_horizon"]["spacing"].as<double>());
                VISUAL_HORIZON_BUFFER = cam.focalLengthPixels * tan(config["visual_horizon"]["horizon_buffer"].as<double>());
                VISUAL_HORIZON_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["visual_horizon"]["subsampling"].as<double>())));
                VISUAL_HORIZON_MINIMUM_SEGMENT_SIZE = cam.focalLengthPixels * tan(config["visual_horizon"]["minimum_segment_size"].as<double>());

                // Goal detector
                GOAL_LINE_SPACING = cam.focalLengthPixels * tan(config["goals"]["spacing"].as<double>());
                GOAL_SUBSAMPLING = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["subsampling"].as<double>())));
                GOAL_EXTENSION_SCALE = config["goals"]["extension_scale"].as<double>() / 2;
                GOAL_MAXIMUM_VERTICAL_CLUSTER_SPACING = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["maximum_vertical_cluster_spacing"].as<double>())));
                GOAL_VERTICAL_CLUSTER_UPPER_BUFFER = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["vertical_cluster_upper_buffer"].as<double>())));
                GOAL_VERTICAL_CLUSTER_LOWER_BUFFER = std::max(1, int(cam.focalLengthPixels * tan(config["goals"]["vertical_cluster_lower_buffer"].as<double>())));
                GOAL_VERTICAL_SD_JUMP = config["goals"]["vertical_sd_jump"].as<double>();

                // Ball Detector
                BALL_MINIMUM_INTERSECTIONS_COARSE = config["ball"]["intersections_coarse"].as<double>();
                BALL_MINIMUM_INTERSECTIONS_FINE = config["ball"]["intersections_fine"].as<double>();
                BALL_SEARCH_CIRCLE_SCALE = config["ball"]["search_circle_scale"].as<double>();
                BALL_MAXIMUM_VERTICAL_CLUSTER_SPACING = std::max(1, int(cam.focalLengthPixels * tan(config["ball"]["maximum_vertical_cluster_spacing"].as<double>())));
                BALL_HORIZONTAL_SUBSAMPLE_FACTOR = config["ball"]["horizontal_subsample_factor"].as<double>();

                // Camera settings
                ALPHA = cam.pixelsToTanThetaFactor[1];
                FOCAL_LENGTH_PIXELS = cam.focalLengthPixels;
            };

            // Trigger the same function when either update
            on<Trigger<CameraParameters>, With<Configuration<LUTClassifier>>>(setParams);
            on<With<CameraParameters>, Trigger<Configuration<LUTClassifier>>>(setParams);

            on<Trigger<Image>, With<LookUpTable>, With<Raw<Sensors>>, Options<Single>>("Classify Image", [this](const Image& image, const LookUpTable& lut, const std::shared_ptr<const Sensors>& sensors) {

                // Our classified image
                auto classifiedImage = std::make_unique<ClassifiedImage<ObjectClass>>();

                // Set our width and height
                classifiedImage->dimensions = { image.width(), image.height() };

                // Attach our sensors
                classifiedImage->sensors = sensors;

                // Find our horizon
                findHorizon(image, lut, *classifiedImage);

                // Find our visual horizon
                findVisualHorizon(image, lut, *classifiedImage);

                // Find our goals
                findGoals(image, lut, *classifiedImage);

                // Find our ball (also helps with the bottom of goals)
                findBall(image, lut, *classifiedImage);

                // Enhance our goals
                enhanceGoals(image, lut, *classifiedImage);

                // Find our goals base
                //findGoalBases(image, lut, *classifiedImage);

                // Enhance our ball
                enhanceBall(image, lut, *classifiedImage);

                // Emit our classified image
                emit(std::move(classifiedImage));
            });

        }

        LUTClassifier::~LUTClassifier() {
            // TODO work out how to fix pimpl and fix it damnit!!
            delete quex;
        }

    }  // vision
}  // modules