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

#ifndef MODULES_LOCALISATION_MMKFROBOTLOCALISATIONENGINE_H
#define MODULES_LOCALISATION_MMKFROBOTLOCALISATIONENGINE_H

#include <nuclear>
#include <chrono>
#include "utility/localisation/LocalisationFieldObject.h"
#include "utility/localisation/FieldDescription.h"
#include "utility/localisation/FieldDescription.h"
#include "messages/support/Configuration.h"
#include "messages/vision/VisionObjects.h"
#include "MultiModalRobotModel.h"

namespace modules {
namespace localisation {
    struct MMKFRobotLocalisationEngineConfig {
        static constexpr const char* CONFIGURATION_PATH = "MMKFRobotLocalisationEngine.json";
    };

    class MMKFRobotLocalisationEngine {
        public:

        MMKFRobotLocalisationEngine() : cfg_({true, true}) {
            last_time_update_time_ = NUClear::clock::now();
        }

        void TimeUpdate(std::chrono::system_clock::time_point current_time);

        void TimeUpdate(std::chrono::system_clock::time_point current_time,
                        const messages::localisation::FakeOdometry& odom);

        std::vector<utility::localisation::LocalisationFieldObject> GetPossibleObjects(
            const messages::vision::Goal& ambiguous_object);

        void ProcessAmbiguousObjects(
            const std::vector<messages::vision::Goal>& ambiguous_objects);

        void IndividualStationaryObjectUpdate(
            const std::vector<messages::vision::Goal>& goals,
            float time_increment);

        void ProcessObjects(const std::vector<messages::vision::Goal>& goals);

        std::shared_ptr<utility::localisation::FieldDescription> field_description() {
            return field_description_;
        };
        void set_field_description(std::shared_ptr<utility::localisation::FieldDescription> desc) {
            field_description_ = desc;
        };

        void UpdateConfiguration(
            const messages::support::Configuration<MultiModalRobotModelConfig>& config) {
            robot_models_.UpdateConfiguration(config);
        };

        void UpdateConfiguration(
            const messages::support::Configuration<MMKFRobotLocalisationEngineConfig>& config) {
            cfg_.angle_between_goals_observation_enabled = config["AngleBetweenGoalsObservationEnabled"];
            cfg_.goal_pair_observation_enabled = config["GoalPairObservationEnabled"];
        };

    // private:
        /// Contains the dimensions of the field
        std::shared_ptr<utility::localisation::FieldDescription> field_description_;

        MultiModalRobotModel robot_models_;

    private:
        struct {
            bool angle_between_goals_observation_enabled;
            bool goal_pair_observation_enabled;
        } cfg_;

        std::chrono::system_clock::time_point last_time_update_time_;
    };
}
}
#endif
