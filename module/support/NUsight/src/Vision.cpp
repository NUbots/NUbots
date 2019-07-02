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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUsight.h"

#include "message/input/Image.h"
#include "message/output/CompressedImage.h"
#include "message/vision/Ball.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/Goal.h"
#include "message/vision/GreenHorizon.h"
#include "message/vision/Line.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/LookUpTableDiff.h"
#include "message/vision/Obstacle.h"
#include "message/vision/VisualMesh.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::Image;
    using message::output::CompressedImage;
    using message::vision::Balls;
    using message::vision::ClassifiedImage;
    using message::vision::Goals;
    using message::vision::GreenHorizon;
    using message::vision::Lines;
    using message::vision::LookUpTableDiff;
    using message::vision::Obstacles;
    using message::vision::VisualMesh;

    void NUsight::provideVision() {
        handles["image"].push_back(
            on<Trigger<Image>, Single, Priority::LOW>().then([this](std::shared_ptr<const Image> image) {
                // If we have never sent an image from this camera, or we
                if (last_image.count(image->camera_id) == 0
                    || (NUClear::clock::now() - last_image[image->camera_id] > max_image_duration)) {
                    powerplant.emit_shared<Scope::NETWORK>(std::move(image), "nusight", false);
                    last_image[image->camera_id] = NUClear::clock::now();
                }
            }));

        handles["compressed_image"].push_back(on<Trigger<CompressedImage>, Single, Priority::LOW>().then(
            [this](std::shared_ptr<const CompressedImage> image) {
                // If we have never sent an image from this camera, or we
                if (last_image.count(image->camera_id) == 0
                    || (NUClear::clock::now() - last_image[image->camera_id] > max_image_duration)) {
                    powerplant.emit_shared<Scope::NETWORK>(std::move(image), "nusight", false);
                    last_image[image->camera_id] = NUClear::clock::now();
                }
            }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage>, Single, Priority::LOW>().then(
            [this](std::shared_ptr<const ClassifiedImage> image) {
                if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                    return;
                }

                powerplant.emit_shared<Scope::NETWORK>(std::move(image), "nusight", false);

                last_classified_image = NUClear::clock::now();
            }));

        handles["vision_object"].push_back(
            on<Trigger<Balls>, Single, Priority::LOW>().then([this](std::shared_ptr<const Balls> balls) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(balls), "nusight", false);
            }));

        handles["vision_object"].push_back(
            on<Trigger<Goals>, Single, Priority::LOW>().then([this](std::shared_ptr<const Goals> goals) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(goals), "nusight", false);
            }));

        handles["vision_object"].push_back(
            on<Trigger<Lines>, Single, Priority::LOW>().then([this](std::shared_ptr<const Lines> lines) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(lines), "nusight", false);
            }));

        handles["vision_object"].push_back(
            on<Trigger<Obstacles>, Single, Priority::LOW>().then([this](std::shared_ptr<const Obstacles> obstacles) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(obstacles), "nusight", false);
            }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then(
            [this](std::shared_ptr<const LookUpTableDiff> tableDiff) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(tableDiff), "nusight", true);
            }));
        handles["visual_mesh"].push_back(
            on<Trigger<VisualMesh>, Single, Priority::LOW>().then([this](std::shared_ptr<const VisualMesh> vm) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(vm), "nusight", false);
            }));
        handles["green_horizon"].push_back(
            on<Trigger<GreenHorizon>, Single, Priority::LOW>().then([this](std::shared_ptr<const GreenHorizon> gh) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(gh), "nusight", false);
            }));
    }
}  // namespace support
}  // namespace module
