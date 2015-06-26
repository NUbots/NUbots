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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "PathPlanner.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <fstream>
#include <iostream>

namespace ob = ompl::base;

namespace modules {
namespace behaviour {
namespace planning {

    using messages::localisation::Ball;
    using messages::localisation::Self;

    // Test whether the current state does not intersect an obstacle.
    // (i.e. that the robot is not intersecting the ball)
    class DarwinBallValidityChecker : public ob::StateValidityChecker
    {
    public:
        DarwinBallValidityChecker(const ob::SpaceInformationPtr& si, Ball ball) :
            ob::StateValidityChecker(si), ball_(ball) {}

        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ob::State* state) const
        {
            return this->clearance(state) > 0.0;
        }

        // Returns the distance from the given state's position to the
        // boundary of the circular obstacle.
        double clearance(const ob::State* state) const
        {
            // We know we're working with a SE2StateSpace in this
            // example, so we downcast state into the specific type.
            const ob::SE2StateSpace::StateType* state2D =
                state->as<ob::SE2StateSpace::StateType>();

            // Extract the robot's (x,y) position from its state
            double x = state2D->getX();
            double y = state2D->getY();
            double t = state2D->getYaw();

            auto bx = ball_.position(0);
            auto by = ball_.position(1);

            // TODO: Use collision between a RotatedRectangle and a circle
            // instead of just a distance test.

            // Distance formula between two points, offset by the circle's
            // radius:
            // TODO: Use a FieldDescription from the config system for the ball radius.
            return sqrt((x-bx)*(x-bx) + (y-by)*(y-by)) - 0.05;
        }

        Ball ball_;
    };

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);


    ompl::base::PathPtr PathPlanner::obstacleFreePathBetween(Transform2D start, Transform2D goal, Ball ball, double timeLimit) {
        // Construct the robot state space in which we're planning.
        ob::StateSpacePtr space(new ob::SE2StateSpace());

        // Set the bounds of space to the field area:
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-4.5);
        bounds.setHigh(4.5);
        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        // Construct a space information instance for this state space:
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new DarwinBallValidityChecker(si, ball)));
        si->setup();

        // Set the starting state:
        ob::ScopedState<> startState(space);
        startState->as<ob::SE2StateSpace::StateType>()->setX(start.x());
        startState->as<ob::SE2StateSpace::StateType>()->setY(start.y());
        startState->as<ob::SE2StateSpace::StateType>()->setYaw(start.angle());

        // Set the goal state:
        ob::ScopedState<> goalState(space);
        goalState->as<ob::SE2StateSpace::StateType>()->setX(goal.x());
        goalState->as<ob::SE2StateSpace::StateType>()->setY(goal.y());
        goalState->as<ob::SE2StateSpace::StateType>()->setYaw(goal.angle());

        // Create a problem instance:
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
        pdef->setStartAndGoalStates(startState, goalState);
        pdef->setOptimizationObjective(getPathLengthObjective(si));

        // int planStep = 0;
        // pdef->setIntermediateSolutionCallback([si, &planStep](
        //      const ob::Planner* planner,
        //      const std::vector<const ob::State*>& states,
        //      const ob::Cost cost) {
        //     std::cout << "Planning step: " << planStep << std::endl;

        //     ob::PlannerData pd(si);
        //     planner->getPlannerData(pd);

        //     std::stringstream fname;
        //     fname << "output/planning-step_" << planStep << ".gml";

        //     std::ofstream myfile;
        //     myfile.open(fname.str());
        //     pd.printGraphML(myfile);
        //     myfile.close();

        //     planStep++;
        // });

        // Construct our optimal planner using the RRTstar algorithm.
        ob::PlannerPtr optimizingPlanner(new ompl::geometric::RRTstar(si));

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // Attempt to solve the planning problem, passing in the time limit in seconds:
        ob::PlannerStatus solved = optimizingPlanner->solve(timeLimit);

        ompl::base::PathPtr solution;

        if (solved)
        {
            solution = pdef->getSolutionPath();
            // Output the length of the path found
            std::cout << "OMPLPathPlanner: Found solution of path length "
                      << solution->length() << std::endl;
        }
        else {
            std::cout << "OMPLPathPlanner: No solution found." << std::endl;
        }

        return solution;
    }


    /** Returns a structure representing the optimization objective to use
        for optimal motion planning. This method returns an objective
        which attempts to minimize the length in configuration space of
        computed paths. */
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }

    /** Returns an optimization objective which attempts to minimize path
        length that is satisfied when a path of length shorter than 1.51
        is found. */
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(1.51));
        return obj;
    }

}
}
}