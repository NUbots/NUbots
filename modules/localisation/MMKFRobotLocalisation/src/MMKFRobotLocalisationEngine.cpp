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

#include "MMKFRobotLocalisationEngine.h"
#include <chrono>
#include <algorithm>
#include "utility/time/time.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/localisation/ResetRobotHypotheses.h"
#include "messages/input/Sensors.h"

using utility::localisation::LFOId;
using utility::localisation::LocalisationFieldObject;
using utility::time::TimeDifferenceSeconds;
using messages::input::Sensors;
using messages::vision::VisionObject;
using messages::localisation::ResetRobotHypotheses;

namespace modules {
namespace localisation {

    bool MMKFRobotLocalisationEngine::CanEmitFieldObjects() {
        return cfg_.emit_robot_fieldobjects;
    }

    std::shared_ptr<messages::support::FieldDescription> MMKFRobotLocalisationEngine::field_description() {
        return field_description_;
    }

    void MMKFRobotLocalisationEngine::set_field_description(std::shared_ptr<messages::support::FieldDescription> desc) {

        field_description_ = desc;
        goalpost_lfos_.bl = {field_description_->goalpost_bl, LFOId::kGoalBL, "goalpost_blue_left"};
        goalpost_lfos_.br = {field_description_->goalpost_br, LFOId::kGoalBR, "goalpost_blue_right"};
        goalpost_lfos_.yl = {field_description_->goalpost_yl, LFOId::kGoalYL, "goalpost_yellow_left"};
        goalpost_lfos_.yr = {field_description_->goalpost_yr, LFOId::kGoalYR, "goalpost_yellow_right"};
    }

    void MMKFRobotLocalisationEngine::UpdateConfiguration(
        const messages::support::Configuration<MultiModalRobotModelConfig>& config) {

        robot_models_.UpdateConfiguration(config);
    }

    void MMKFRobotLocalisationEngine::UpdateConfiguration(
        const messages::support::Configuration<MMKFRobotLocalisationEngineConfig>& config) {

        cfg_.angle_between_goals_observation_enabled = config["AngleBetweenGoalsObservationEnabled"].as<bool>();
        cfg_.goal_pair_observation_enabled = config["GoalPairObservationEnabled"].as<bool>();
        cfg_.all_goals_are_blue = config["AllGoalsAreBlue"].as<bool>();
        cfg_.emit_robot_fieldobjects = config["EmitRobotFieldobjects"].as<bool>();
    }

    void MMKFRobotLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time,
                                                 const Sensors& sensors) {
        double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
        last_time_update_time_ = current_time;
        robot_models_.TimeUpdate(seconds, sensors);
    }

    std::vector<LocalisationFieldObject> MMKFRobotLocalisationEngine::GetPossibleObjects(
            const messages::vision::Goal& ambiguous_object) {

        std::vector<LocalisationFieldObject> possible;

        if (ambiguous_object.side == messages::vision::Goal::Side::LEFT) {
            possible.push_back(goalpost_lfos_.bl);
            if (!cfg_.all_goals_are_blue)
                possible.push_back(goalpost_lfos_.yl);
        } else if (ambiguous_object.side == messages::vision::Goal::Side::RIGHT) {
            possible.push_back(goalpost_lfos_.br);
            if (!cfg_.all_goals_are_blue)
                possible.push_back(goalpost_lfos_.yr);
        } else if (ambiguous_object.side == messages::vision::Goal::Side::UNKNOWN) {
            possible.push_back(goalpost_lfos_.bl);
            possible.push_back(goalpost_lfos_.br);
            if (!cfg_.all_goals_are_blue) {
                possible.push_back(goalpost_lfos_.yl);
                possible.push_back(goalpost_lfos_.yr);
            }
        } else {
            NUClear::log<NUClear::ERROR>(__FILE__, ",", __LINE__, ": The ambiguous_object (messages::vision::Goal) has an invalid messages::vision::Goal::Side");
        }

        return std::move(possible);
    }

    bool GoalPairObserved(
        const std::vector<messages::vision::Goal>& ambiguous_objects) {

        if (ambiguous_objects.size() != 2)
            return false;

        auto& oa = ambiguous_objects[0];
        auto& ob = ambiguous_objects[1];

        return
            (oa.side == messages::vision::Goal::Side::RIGHT &&
             ob.side == messages::vision::Goal::Side::LEFT) ||
            (oa.side == messages::vision::Goal::Side::LEFT &&
             ob.side == messages::vision::Goal::Side::RIGHT);
    }

    void MMKFRobotLocalisationEngine::ProcessAmbiguousObjects(
        const std::vector<messages::vision::Goal>& ambiguous_objects) {
        bool pair_observations_enabled =
            cfg_.goal_pair_observation_enabled ||
            cfg_.angle_between_goals_observation_enabled;

        if (pair_observations_enabled && GoalPairObserved(ambiguous_objects)) {
            std::vector<messages::vision::VisionObject> vis_objs;
            // Ensure left goal is always first.
            if (ambiguous_objects[0].side == messages::vision::Goal::Side::LEFT){
                vis_objs = { ambiguous_objects[0], ambiguous_objects[1] };
            } else {
                vis_objs = { ambiguous_objects[1], ambiguous_objects[0] };
            }

            std::vector<std::vector<LocalisationFieldObject>> objs;
            objs.push_back({goalpost_lfos_.bl, goalpost_lfos_.br});

            if (!cfg_.all_goals_are_blue)
                objs.push_back({goalpost_lfos_.yl, goalpost_lfos_.yr});

            if(cfg_.goal_pair_observation_enabled)
                robot_models_.AmbiguousMeasurementUpdate(vis_objs, objs);

            // if (cfg_.angle_between_goals_observation_enabled)
            //     robot_models_.AmbiguousMultipleMeasurementUpdate(vis_objs, objs);
        } else {
            for (auto& ambiguous_object : ambiguous_objects) {
                // Get a vector of all field objects that the observed object could
                // possibly be
                auto possible_objects = GetPossibleObjects(ambiguous_object);
                robot_models_.AmbiguousMeasurementUpdate(ambiguous_object, possible_objects);
            }
        }

        robot_models_.PruneModels();
    }

    void MMKFRobotLocalisationEngine::IndividualStationaryObjectUpdate(
        const std::vector<messages::vision::Goal>& goals,
        float) {

        for (auto& observed_object : goals) {

            LocalisationFieldObject actual_object;

            if (observed_object.side == messages::vision::Goal::Side::LEFT)
                actual_object = goalpost_lfos_.bl;

            if (observed_object.side == messages::vision::Goal::Side::RIGHT)
                actual_object = goalpost_lfos_.br;

            robot_models_.MeasurementUpdate(observed_object, actual_object);
        }

        robot_models_.NormaliseAlphas();
    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void MMKFRobotLocalisationEngine::ProcessObjects(const std::vector<messages::vision::Goal>& goals) {
        // robot_models_.robot_models_ = std::vector<std::unique_ptr<RobotHypothesis>>();
        // robot_models_.robot_models_.push_back(std::make_unique<RobotHypothesis>());

        ProcessAmbiguousObjects(goals);
    }

    void MMKFRobotLocalisationEngine::Reset(const ResetRobotHypotheses& reset, const Sensors& sensors) {
        robot_models_.robot_models_ = std::vector<std::unique_ptr<RobotHypothesis>>();
        for (const auto& reset_hyp : reset.hypotheses) {
            auto hyp = std::make_unique<RobotHypothesis>(reset_hyp, sensors);
            hyp->weight_ = 1 / double(reset.hypotheses.size());
            robot_models_.robot_models_.push_back(std::move(hyp));
        }
    }

    void MMKFRobotLocalisationEngine::OdometryMeasurementUpdate(const Sensors& sensors) {
        robot_models_.MeasurementUpdate(sensors);
    }
}
}
