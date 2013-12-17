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
            });

            reaction = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {
                /********************************************************
                 *          THIS CODE HAS NOT BEEN PORTED YET           *
                 ********************************************************/
                reaction.disable();
                // FIND GOALS                
                // ransac method
                std::vector<Goal> ransac_goals_edges = m_goal_detector_ransac_edges->run();
                //m_goal_detector_ransac_edges->relabel(ransac_goals_edges);

                m_blackboard->addGoals(ransac_goals_edges);

                // FIND FIELD POINTS
                // REMOVED FOR RC2013
                // Edit here to change whether centre circles, lines or corners are found
                //      (note lines cannot be published yet)
                m_field_point_detector->run(true, true, true);

                // FIND BALLS
                //m_blackboard->addBalls(m_ball_detector_dave->run());
                m_blackboard->addBalls(m_ball_detector_shannon->run());

                // FIND OBSTACLES
                std::vector<Obstacle> obstacles = ObstacleDetectionCH::run();
                m_blackboard->addObstacles(obstacles);

                // ADD IN LABELLING OF GOALS BASED ON KEEPER COLOUR

                // EMIT RESULTS

            });

//BEGIN Trents multi trigger:
            

            auto functionthattakesallthethingsback = [this](const blah& ball, const goal& goal, ..., int index_of_triggered){
                if(all triggered){
                    emit(collection);
                }
                reaction.enable();
            }

            on<Trigger<blah>, With<goal>, With<line>,..>(std::bind(functionthattakesallthethingsback,_1,_2,_3,0));

            on<With<blah>, Trigger<goal>, With<line>,..>(std::bind(functionthattakesallthethingsback,_1,_2,_3,1));

            on<With<blah>, With<goal>, Trigger<line>,..>(std::bind(functionthattakesallthethingsback,_1,_2,_3,2));
        }

    }  // vision
}  // modules
