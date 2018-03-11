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

#include "NUbugger.h"

#include "message/input/Image.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/LookUpTableDiff.h"
#include "message/vision/VisionObjects.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::Ball;
    using message::vision::ClassifiedImage;
    using message::vision::Goal;
    using message::vision::Line;
    using message::vision::LookUpTableDiff;
    using message::vision::NUsightBalls;
    using message::vision::NUsightGoals;
    using message::vision::NUsightLines;
    using message::vision::NUsightObstacles;
    using message::vision::Obstacle;

    void NUbugger::provideVision() {
        handles["camera_parameters"].push_back(on<Every<1, Per<std::chrono::seconds>>, With<CameraParameters>>().then(
            [this](const CameraParameters& cameraParameters) {
                send(cameraParameters, 1, false, NUClear::clock::now());
            }));

        handles["image"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            // send<Image>(image, 1, false, NUClear::clock::now());
            // send<Image>(std::make_unique<Image>(image), 1, false, NUClear::clock::now());
            send(image, 1, false, NUClear::clock::now());

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(
            on<Trigger<ClassifiedImage>, Single, Priority::LOW>().then([this](const ClassifiedImage& image) {

                if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                    return;
                }

                send(image, image.image->camera_id + 1, false, NUClear::clock::now());

                last_classified_image = NUClear::clock::now();
            }));

        handles["vision_object"].push_back(
            on<Trigger<std::vector<Ball>>, Single, Priority::LOW>().then([this](const std::vector<Ball>& balls) {

                NUsightBalls nusight;

                for (const auto& ball : balls) {
                    nusight.balls.push_back(ball);
                }

                send(nusight, 1, false, NUClear::clock::now());
            }));

        handles["vision_object"].push_back(
            on<Trigger<std::vector<Goal>>, Single, Priority::LOW>().then([this](const std::vector<Goal>& goals) {

                NUsightGoals nusight;

                for (const auto& goal : goals) {
                    nusight.goals.push_back(goal);
                }

                send(nusight, 2, false, NUClear::clock::now());
            }));

        handles["vision_object"].push_back(
            on<Trigger<std::vector<Line>>, Single, Priority::LOW>().then([this](const std::vector<Line>& lines) {

                NUsightLines nusight;

                for (const auto& line : lines) {
                    nusight.lines.push_back(line);
                }

                send(nusight, 3, false, NUClear::clock::now());
            }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Obstacle>>, Single, Priority::LOW>().then(
            [this](const std::vector<Obstacle>& obstacles) {

                NUsightObstacles nusight;

                for (const auto& obstacles : obstacles) {
                    nusight.obstacles.push_back(obstacles);
                }

                send(nusight, 4, false, NUClear::clock::now());
            }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then(
            [this](const LookUpTableDiff& tableDiff) { send(tableDiff, 0, true, NUClear::clock::now()); }));
    }
}  // namespace support
}  // namespace module
