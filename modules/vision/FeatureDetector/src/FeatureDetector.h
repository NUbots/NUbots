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

#ifndef MODULES_VISION_FEATUREDETECTOR_H
#define MODULES_VISION_FEATUREDETECTOR_H

#include <nuclear>
#include <string>
#include <vector>

#include "messages/vision/ClassifiedImage.h"
#include "messages/support/Configuration.h"
#include "messages/platform/darwin/DarwinSensors.h"

#include "VisionKinematics.h"
#include "BallDetector/BallDetector.h"
#include "GoalDetector/GoalDetector_RANSAC.h"
//#include "FieldPointDetector/FieldPointDetector.h"
#include "ObstacleDetector/ObstacleDetector.h"

namespace modules {
    namespace vision {

        struct FeatureDetectorConfig{
            static constexpr const char* CONFIGURATION_PATH = "FeatureDetector.json";
        };

        struct VisionKinematicsConfig{
            static constexpr const char* CONFIGURATION_PATH = "VisionKinematics.json";
        };

        struct BallDetectorConfig{
            static constexpr const char* CONFIGURATION_PATH = "BallDetector.json";
        };

        struct GoalDetectorConfig{
            static constexpr const char* CONFIGURATION_PATH = "GoalDetector.json";
        };

        struct FieldPointDetectorConfig{
            static constexpr const char* CONFIGURATION_PATH = "FieldPointDetector.json";
        };

        struct ObstacleDetectorConfig{
            static constexpr const char* CONFIGURATION_PATH = "ObstacleDetector.json";
        };

        struct CameraConfig {
            static constexpr const char* CONFIGURATION_PATH = "LinuxCamera.json";
        };
        /**
         * Searches for interesting features in a classified image.
         *
         * @author Alex Biddulph
         */
        class FeatureDetector : public NUClear::Reactor {
        private:
            bool DETECT_LINES;
            bool DETECT_GOALS;
            bool DETECT_BALLS;
            bool DETECT_OBSTACTLES; 

            /*
            NUClear::Reactor::ReactionHandle m_detectGoals;
            NUClear::Reactor::ReactionHandle m_detectBalls;
            NUClear::Reactor::ReactionHandle m_detectLineObjects;
            NUClear::Reactor::ReactionHandle m_detectObstacles;
*/


            BallDetector m_ballDetector;

            GoalDetector_RANSAC m_goalDetector;

            //FieldPointDetector m_fieldPointDetector;

            ObstacleDetector m_obstacleDetector;

            VisionKinematics m_visionKinematics;

        public:
            explicit FeatureDetector(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_FEATUREDETECTOR_H

