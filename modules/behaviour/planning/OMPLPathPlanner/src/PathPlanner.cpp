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
#include <iterator>
#include <boost/graph/adjacency_list.hpp>

#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/Intersection.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "messages/behaviour/WalkPath.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::localisation::Ball;
    using messages::localisation::Self;
    using messages::behaviour::WalkPath;
    using utility::math::matrix::Transform2D;
    using utility::math::angle::vectorToBearing;
    using utility::math::geometry::Circle;
    using utility::math::geometry::RotatedRectangle;
    namespace intersection = utility::math::geometry::intersection;
    namespace ob = ompl::base;

    // Helper method to convert from an OMPL state to a transform2d.
    Transform2D omplState2Transform2d(const ob::State* state) {
        // We know we're working with a SE2StateSpace,
        // so we downcast state into the specific type.
        const ob::SE2StateSpace::StateType* state2D =
                state->as<ob::SE2StateSpace::StateType>();
        double x = state2D->getX();
        double y = state2D->getY();
        double t = state2D->getYaw();

        return {x, y, t};
    }

    // Test whether the current state does not intersect an obstacle.
    // (i.e. that the robot is not intersecting the ball)
    class DarwinBallValidityChecker : public ob::StateValidityChecker {
    public:
        DarwinBallValidityChecker(const ob::SpaceInformationPtr& si, arma::vec2 ballPos) :
            ob::StateValidityChecker(si), ballPos_(ballPos) {}

        bool isValid(const ob::State* state) const {

            auto pos = omplState2Transform2d(state);

            // TODO: Use a FieldDescription from the config system for the ball radius.
            Circle ballCircle(0.05, ballPos_);
            RotatedRectangle robotRect(pos, {0.12, 0.17});

            return !intersection::test(ballCircle, robotRect);

            // Distance formula between two points, offset by the circle's radius:
            // auto bx = ball_.position(0);
            // auto by = ball_.position(1);
            // return sqrt((pos.x()-bx)*(pos.x()-bx) + (pos.y()-by)*(pos.y()-by)) - 0.05;
        }

        arma::vec2 ballPos_;
    };

    class DarwinPathOptimisationObjective : public ob::OptimizationObjective {
    public:
        DarwinPathOptimisationObjective(const ob::SpaceInformationPtr &si) :
            ob::OptimizationObjective(si) {}
        virtual ob::Cost stateCost(const ob::State* s) const {
            return ob::Cost(0);
        }
        // virtual bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const; // default: c1 + eps < c2
        virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const {
            // Get the states as Transform2Ds:
            auto t1 = omplState2Transform2d(s1);
            auto t2 = omplState2Transform2d(s2);

            // Straight line distance and direction from start to goal.
            auto diff = arma::vec2(t2.xy() - t1.xy());
            auto dist = arma::norm(diff);
            auto dir = vectorToBearing(diff);

            // Heading difference between start and goal.
            auto goalTargetDiff = utility::math::angle::difference(t1.angle(), t2.angle());

            // Turning required to turn onto, then away from the straight line direction.
            auto tdiff1 = utility::math::angle::difference(t1.angle(), dir);
            auto tdiff2 = utility::math::angle::difference(t2.angle(), dir);
            auto totalTurning = tdiff1 + tdiff2;

            // The cost of turning vs walking in a straight line in m/rad.
            double turningWeight = 0.5;

            // The distance beyond which we prefer forward walking to omnidirectional walking.
            double closeDist = 0.3;

            auto closeF = std::min(dist / closeDist, 1.0); // 0 when close, 1 when far.

            double turningCost = (1 - closeF) * goalTargetDiff + // If close, prefer omnidirectional walking;
                                      closeF  * totalTurning;    // otherwise, turn onto and off the path.

            // Straight line distance + a cost for turning to and from the straight line.
            return ob::Cost(dist + turningWeight * turningCost);
        }
        // virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const; // default: (+)
        // virtual ob::Cost identityCost() const; // default: 0
        // virtual ob::Cost infiniteCost() const; // default: double value inf
    };

    /** Returns a structure representing the optimization objective to use
        for optimal motion planning. This method returns an objective
        which attempts to minimize the length in configuration space of
        computed paths. */
    ob::OptimizationObjectivePtr getOptimisationObjective(const ob::SpaceInformationPtr& si) {
        return ob::OptimizationObjectivePtr(new DarwinPathOptimisationObjective(si));
        // return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }


    std::unique_ptr<WalkPath> PathPlanner::obstacleFreePathBetween(Transform2D start, Transform2D goal, arma::vec2 ballPos, double timeLimit) {
        auto omplPath = omplPlanPath(start, goal, ballPos, timeLimit);
        auto walkPath = omplPathToWalkPath(omplPath);
        return std::move(walkPath);
    }

    std::unique_ptr<WalkPath> PathPlanner::omplPathToWalkPath(ompl::base::PathPtr omplPath) {
        auto walkPath = std::make_unique<WalkPath>();

        // Emit the new path to NUSight.
        if (omplPath != nullptr) {

            // Get the path states:
            auto pathGeom = boost::static_pointer_cast<ompl::geometric::PathGeometric>(omplPath);
            std::vector<ob::State*>& states = pathGeom->getStates();

            // Add each state as a Transform2D:
            for (auto* state : states) {
                auto stateTrans = omplState2Transform2d(state);
                walkPath->states.push_back(stateTrans);
            }
        }

        return std::move(walkPath);
    }

    ompl::base::PathPtr PathPlanner::omplPlanPath(Transform2D start, Transform2D goal, arma::vec2 ballPos, double timeLimit) {
        // Construct the robot state space in which we're planning.
        stateSpace = ob::StateSpacePtr(new ob::SE2StateSpace());

        // Set the bounds of space to the field area:
        ob::RealVectorBounds bounds(2);
        bounds.setLow (0, std::min(start.x()-1.0, goal.x()-1.0)); // x
        bounds.setHigh(0, std::max(start.x()+1.0, goal.x()+1.0));
        bounds.setLow (1, std::min(start.y()-1.0, goal.y()-1.0)); // y
        bounds.setHigh(1, std::max(start.y()+1.0, goal.y()+1.0));
        // TODO: Clip bounds to the field, if both the goal and start points are on the field.
        stateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);

        // Construct a space information instance for this state space:
        ob::SpaceInformationPtr si(new ob::SpaceInformation(stateSpace));
        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new DarwinBallValidityChecker(si, ballPos)));
        si->setup();

        // Set the starting state:
        ob::ScopedState<> startState(stateSpace);
        startState->as<ob::SE2StateSpace::StateType>()->setX(start.x());
        startState->as<ob::SE2StateSpace::StateType>()->setY(start.y());
        startState->as<ob::SE2StateSpace::StateType>()->setYaw(start.angle());

        // Set the goal state:
        ob::ScopedState<> goalState(stateSpace);
        goalState->as<ob::SE2StateSpace::StateType>()->setX(goal.x());
        goalState->as<ob::SE2StateSpace::StateType>()->setY(goal.y());
        goalState->as<ob::SE2StateSpace::StateType>()->setYaw(goal.angle());

        // Create a problem instance:
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
        pdef->setStartAndGoalStates(startState, goalState);
        pdef->setOptimizationObjective(getOptimisationObjective(si));

        // Construct our optimal planner using the RRT* algorithm.
        // TODO: Look into using CForest for faster planning.
        ob::PlannerPtr optimizingPlanner(new ompl::geometric::RRTstar(si));

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // Attempt to solve the planning problem, passing in the time limit in seconds:
        ob::PlannerStatus solved = optimizingPlanner->solve(timeLimit);

        ompl::base::PathPtr solution;
        if (solved) {
            solution = pdef->getSolutionPath();
            // Output the length of the path found
            std::cout << "OMPLPathPlanner: Found solution of path length "
                      << solution->length() << std::endl;

            // Print the entire graph:
            debugPositions.clear();
            debugParentIndices.clear();
            ob::PlannerData pd(si);
            optimizingPlanner->getPlannerData(pd);
            for (unsigned int i = 0; i < pd.numVertices(); i++) {
                auto t = omplState2Transform2d(pd.getVertex(i).getState());
                // std::cout << t.t() << std::endl;
                debugPositions.push_back(arma::vec(t.xy()));

                std::vector<unsigned int> edges;
                int num = pd.getIncomingEdges(i, edges);
                if (num > 0) {
                    debugParentIndices.push_back(edges.front());
                } else {
                    debugParentIndices.push_back(i);
                }
            }
        }
        else {
            std::cout << "OMPLPathPlanner: No solution found." << std::endl;
        }
        return solution;
    }

}
}
}
