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
#include "message/support/Configuration.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisionObjects.h"
#include "message/input/Sensors.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "MultiModalRobotModel.h"
#include "message/input/Sensors.h"

namespace module {
namespace localisation {

    class MMKFRobotLocalisationEngine {
        public:

        MMKFRobotLocalisationEngine() : cfg_({true, true, false, true}) {
            last_time_update_time_ = NUClear::clock::now();
        }

        void TimeUpdate(NUClear::clock::time_point current_time,
                        const message::input::Sensors& sensors);

        std::vector<utility::localisation::LocalisationFieldObject> GetPossibleObjects(
            const message::vision::Goal& ambiguous_object);

        void ProcessAmbiguousObjects(
            const std::vector<message::vision::Goal>& ambiguous_objects);

        void IndividualStationaryObjectUpdate(
            const std::vector<message::vision::Goal>& goals,
            float time_increment);

        void ProcessObjects(const std::vector<message::vision::Goal>& goals);

        // void SensorsUpdate(const message::input::Sensors& sensors);

        std::shared_ptr<message::support::FieldDescription> field_description();

        void set_field_description(std::shared_ptr<message::support::FieldDescription> desc);

        void UpdateMultiModalRobotModelConfiguration(const message::support::Configuration& config);

        void UpdateRobotLocalisationEngineConfiguration(const message::support::Configuration& config);

        bool CanEmitFieldObjects();

        void Reset(const message::localisation::ResetRobotHypotheses& reset, const message::input::Sensors& sensors);

        void OdometryMeasurementUpdate(const message::input::Sensors& sensors);
    // private:
        MultiModalRobotModel robot_models_;

        std::shared_ptr<message::support::FieldDescription> field_description_;

    private:
        struct {
            bool angle_between_goals_observation_enabled;
            bool goal_pair_observation_enabled;
            bool all_goals_are_own;
            bool emit_robot_fieldobjects;
        } cfg_;

        struct {
            utility::localisation::LocalisationFieldObject own_l;
            utility::localisation::LocalisationFieldObject own_r;
            utility::localisation::LocalisationFieldObject opp_l;
            utility::localisation::LocalisationFieldObject opp_r;
        } goalpost_lfos_;

        NUClear::clock::time_point last_time_update_time_;
    };
}
}
#endif
