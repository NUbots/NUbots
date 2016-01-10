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
#include "message/vision/VisionObjects.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"
#include "message/localisation/ResetRobotHypotheses.h"
#include "message/input/Sensors.h"

using utility::localisation::LFOId;
using utility::localisation::LocalisationFieldObject;
using utility::time::TimeDifferenceSeconds;
using message::input::Sensors;
using message::vision::VisionObject;
using message::localisation::ResetRobotHypotheses;
using message::vision::Goal;

namespace module {
namespace localisation {

    bool MMKFRobotLocalisationEngine::CanEmitFieldObjects() {
        return cfg_.emit_robot_fieldobjects;
    }

    std::shared_ptr<message::support::FieldDescription> MMKFRobotLocalisationEngine::field_description() {
        return field_description_;
    }

    void MMKFRobotLocalisationEngine::set_field_description(std::shared_ptr<message::support::FieldDescription> desc) {

        field_description_ = desc;
        goalpost_lfos_.own_l = {field_description_->goalpost_own_l, LFOId::kGoalOwnL, "goalpost_own_left"};
        goalpost_lfos_.own_r = {field_description_->goalpost_own_r, LFOId::kGoalOwnR, "goalpost_own_right"};
        goalpost_lfos_.opp_l = {field_description_->goalpost_opp_l, LFOId::kGoalOppL, "goalpost_opp_left"};
        goalpost_lfos_.opp_r = {field_description_->goalpost_opp_r, LFOId::kGoalOppR, "goalpost_opp_right"};
    }

    void MMKFRobotLocalisationEngine::UpdateMultiModalRobotModelConfiguration(const message::support::Configuration& config) {

        robot_models_.UpdateConfiguration(config);
    }

    void MMKFRobotLocalisationEngine::UpdateRobotLocalisationEngineConfiguration(const message::support::Configuration& config) {

        cfg_.angle_between_goals_observation_enabled = config["AngleBetweenGoalsObservationEnabled"].as<bool>();
        cfg_.goal_pair_observation_enabled = config["GoalPairObservationEnabled"].as<bool>();
        cfg_.all_goals_are_own = config["AllGoalsAreBlue"].as<bool>();
        cfg_.emit_robot_fieldobjects = config["EmitRobotFieldobjects"].as<bool>();
    }

    void MMKFRobotLocalisationEngine::TimeUpdate(NUClear::clock::time_point current_time,
                                                 const Sensors& sensors) {
        double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
        last_time_update_time_ = current_time;
        robot_models_.TimeUpdate(seconds, sensors);
    }

    std::vector<LocalisationFieldObject> MMKFRobotLocalisationEngine::GetPossibleObjects(
            const message::vision::Goal& obj) {
        std::vector<LocalisationFieldObject> possible;

        //BOTH UNKNOWN
        if (obj.side == Goal::Side::UNKNOWN && obj.team == Goal::Team::UNKNOWN ) {
            possible.push_back(goalpost_lfos_.own_l);
            possible.push_back(goalpost_lfos_.own_r);
            possible.push_back(goalpost_lfos_.opp_l);
            possible.push_back(goalpost_lfos_.opp_r);
            return std::move(possible);
        }

        //SIDE UNKNOWN
        if (obj.side == Goal::Side::UNKNOWN) {
            if ( obj.team == Goal::Team::OWN     ) {
                possible.push_back(goalpost_lfos_.own_l);
                possible.push_back(goalpost_lfos_.own_r);
            }
            if ( obj.team == Goal::Team::OPPONENT) {
                possible.push_back(goalpost_lfos_.opp_l);
                possible.push_back(goalpost_lfos_.opp_r);
            }
            return std::move(possible);
        }

        //TEAM UNKNOWN
        if (obj.team == Goal::Team::UNKNOWN) {
            if (obj.side == Goal::Side::LEFT ) {
                possible.push_back(goalpost_lfos_.own_l);
                possible.push_back(goalpost_lfos_.opp_l);
            }
            if (obj.side == Goal::Side::RIGHT) {
                possible.push_back(goalpost_lfos_.own_r);
                possible.push_back(goalpost_lfos_.opp_r);
            }
            return std::move(possible);
        }

        //COMPLETE KNOWLEDGE
        if (obj.side == Goal::Side::LEFT  && obj.team == Goal::Team::OWN     ) { possible.push_back(goalpost_lfos_.own_l); return std::move(possible); }
        if (obj.side == Goal::Side::RIGHT && obj.team == Goal::Team::OWN     ) { possible.push_back(goalpost_lfos_.own_r); return std::move(possible); }
        if (obj.side == Goal::Side::LEFT  && obj.team == Goal::Team::OPPONENT) { possible.push_back(goalpost_lfos_.opp_l); return std::move(possible); }
        if (obj.side == Goal::Side::RIGHT && obj.team == Goal::Team::OPPONENT) { possible.push_back(goalpost_lfos_.opp_r); return std::move(possible); }


        NUClear::log<NUClear::ERROR>(__FILE__, ",", __LINE__, ": The ambiguous_object (message::vision::Goal) has an invalid message::vision::Goal::Side");
        return std::move(possible);
    }

    bool GoalPairObserved(
        const std::vector<message::vision::Goal>& ambiguous_objects) {

        if (ambiguous_objects.size() != 2)
            return false;

        auto& oa = ambiguous_objects[0];
        auto& ob = ambiguous_objects[1];

        if (oa.team == Goal::Team::UNKNOWN ||
            ob.team == Goal::Team::UNKNOWN ||
            oa.team != ob.team)
            return false;

        return
            (oa.side == message::vision::Goal::Side::RIGHT &&
             ob.side == message::vision::Goal::Side::LEFT) ||
            (oa.side == message::vision::Goal::Side::LEFT &&
             ob.side == message::vision::Goal::Side::RIGHT);
    }

    void MMKFRobotLocalisationEngine::ProcessAmbiguousObjects(
        const std::vector<message::vision::Goal>& ambiguous_objects) {
        bool pair_observations_enabled =
            cfg_.goal_pair_observation_enabled ||
            cfg_.angle_between_goals_observation_enabled;

        if (pair_observations_enabled && GoalPairObserved(ambiguous_objects)) {

            std::vector<message::vision::Goal> vis_objs;
            // Ensure left goal is always first.
            if (ambiguous_objects[0].side == Goal::Side::LEFT){
                vis_objs = { ambiguous_objects[0], ambiguous_objects[1] };
            } else {
                vis_objs = { ambiguous_objects[1], ambiguous_objects[0] };
            }

            std::vector<std::vector<LocalisationFieldObject>> objs;

            auto own_goals = {goalpost_lfos_.own_l, goalpost_lfos_.own_r};
            auto opp_goals = {goalpost_lfos_.opp_l, goalpost_lfos_.opp_r};

            // Note: If goals are ambigous, negated tests will cause both to be added.
            if (vis_objs[0].team != Goal::Team::OPPONENT) {
                objs.push_back(own_goals);
            }
            if (vis_objs[0].team != Goal::Team::OWN && !cfg_.all_goals_are_own) {
                objs.push_back(opp_goals);
            }

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
        const std::vector<message::vision::Goal>& goals,
        float) {

        for (auto& observed_object : goals) {

            LocalisationFieldObject actual_object;

            if (observed_object.side == message::vision::Goal::Side::LEFT)
                actual_object = goalpost_lfos_.own_l;

            if (observed_object.side == message::vision::Goal::Side::RIGHT)
                actual_object = goalpost_lfos_.own_r;

            robot_models_.MeasurementUpdate(observed_object, actual_object);
        }

        robot_models_.NormaliseAlphas();
    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void MMKFRobotLocalisationEngine::ProcessObjects(const std::vector<message::vision::Goal>& goals) {
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
