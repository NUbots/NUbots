/*! @file NavigationLogic.h
    @brief maths and logic for navigation behaviours.

    @author Josiah Walker

 Copyright (c) 2012 Josiah Walker

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef NAVIGATIONLOGIC_H
#define NAVIGATIONLOGIC_H

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Tools/Math/General.h"



class NavigationLogic {
public:
    
    /*! @brief Returns a vector of (x,y,heading) for the robot's current location.
     */
    static vector<float> getSelfPosition() {
        vector<float> result(3,0);
        Self self = Blackboard->Objects->self;
        result[0] = self.wmX();
        result[1] = self.wmY();
        result[2] = self.Heading();
        
        return result;
    }
    
    /*! @brief Returns a vector of (x,y,0) for the object's current location.
     */
    static vector<float> getObjectPosition(Object& object) {
        vector<float> result(3,0);
        
        result[0] = object.estimatedDistance()*cos(object.estimatedBearing());
        result[1] = object.estimatedDistance()*sin(object.estimatedBearing());
        
        return result;
    }

    /*! @brief Returns a vector of (x,y,0) for the centre of the opponent goal.
     */
    static vector<float> getOpponentGoalPosition() {
        vector<float> result(3,0);
        float x,y;
        
        if (Blackboard->TeamInfo->TeamNumber == GameInformation::TEAM_BLUE) {
            x = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].X()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].X());
            y = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].Y()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].Y());
        } else {
            x = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].X()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].X());
            y = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].Y()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].Y());
        }
        
        return result;
    }
    
    /*! @brief Returns a vector of (x,y,0) for the centre of our own goal.
     */
    static vector<float> getOwnGoalPosition() {
        vector<float> result(3,0);
        float x,y;
        
        if (not Blackboard->TeamInfo->TeamNumber == GameInformation::TEAM_BLUE) {
            x = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].X()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].X());
            y = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].Y()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].Y());
        } else {
            x = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].X()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].X());
            y = 0.5f*(BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].Y()+BlackBoard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].Y());
        }
        
        return result;
    }
    
    /*! @brief Returns a vector of relative (distance,bearing,heading) for the difference betweek two (x,y,heading) vectors.
     */
    static vector<float> getPositionDifference(position1,position2) {
        vector<float> result(3,0);
        float rel_x,rel_y;
        
        rel_x = position2[0]-position1[0];
        rel_y = position2[1]-position1[1];
        
        result[0] = sqrt(rel_x*rel_x+rel_y*rel_y);
        result[1] = mathGeneral::normaliseAngle(atan2(rel_y,rel_x) - position1[2]);
        result[2] = mathGeneral::normaliseAngle(position2[2]-position1[2]);
        
        return result;
    }
    
    /*! @brief Returns a vector of visible field Objects which are identified by flags as obstacles.
     */
    static vector<Object> getVisibleObstacles() {
        vector<Object> result;
        
        //XXX: unimplemented
        
        return result;
    }
    
    /*! @brief Returns the position of the ball on the field.
     */
    static vector<float> getBallPosition() {
        return getObjectPosition((Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL]));
    }
    
    
    /*! @brief Returns a defensive support (intercepting) position on the field, facing the ball.
     */
    static vector<float> getBallDefensePosition() {
        const float supportDistance = 70.;
        const float goalBoxClearance = 40.;
        vector<float> result(3,600);
        
        //intialise positions
        vector<float> ballPos = this::getBallPosition();
        vector<float> myGoalPos = this::getOwnGoalPosition();
        vector<float> ballToGoal(2,0);
        
        //calculate values
        ballToGoal[0] = myGoalPos[0]-ballPos[0];
        ballToGoal[1] = myGoalPos[1]-ballPos[1];
        float ballToGoalDistance = MathGeneral::sqrt(ballToGoal[0]*ballToGoal[0]+ballToGoal[1]*ballToGoal[1]);
        
        float supportDistance = MathGeneral::min(supportDistance,ballToGoalDistance-goalBoxClearance);
        
        if (supportDistance > 0.) {
            //calculate position
            result[0] = ballToGoal[0]*supportDistance/(ballToGoalDistance+0.000001)+ballPos[0];
            result[1] = ballToGoal[1]*supportDistance/(ballToGoalDistance+0.000001)+ballPos[1];
            
            //calculate heading
            result[2] = atan2(-ballToGoal[1]/(ballToGoalDistance+0.000001),-ballToGoal[0]/(ballToGoalDistance+0.000001));
        }
        
        return result;
    }
    
    /*! @brief Returns an offensive support (passing) position on the field, angled to see the ball.
     */
    static vector<float> getBallOffensePosition() {
        const float supportDistance = 130.; //XXX: put this offset somewhere
        vector<float> result(3,0);
        
        //intialise positions
        vector<float> ballPos = this::getBallPosition();
        vector<float> goalPos = this::getOpponentGoalPosition();
        vector<float> ballToGoal(2,0);
        vector<float> supportDirection(2,0);
        
        //calculate values
        ballToGoal[0] = goalPos[0]-ballPos[0];
        ballToGoal[1] = goalPos[1]-ballPos[1];
        float ballToGoalDistance = MathGeneral::sqrt(ballToGoal[0]*ballToGoal[0]+ballToGoal[1]*ballToGoal[1]);
        
        //work out where we have to rotate to get support directions
        if (ballPos[0] > 0) {
            supportDirection[0] = ballToGoal[1]/ballToGoalDistance;
            supportDirection[1] = -ballToGoal[0]/ballToGoalDistance;
        } else {
            supportDirection[0] = -ballToGoal[1]/ballToGoalDistance;
            supportDirection[1] = ballToGoal[0]/ballToGoalDistance;
        }
        
        result[0] = supportDirection[0]*supportDistance;
        result[1] = supportDirection[1]*supportDistance;
        
        result[2] = atan2(ballPos[1]+ballToGoal[1]*0.75-result[1],ballPos[0]+ballToGoal[0]*0.75-result[0]); //first y, then x
        
        return result;
    }
    
     /*! @brief Returns the kickoff start position on the field.
     */
    static vector<float> getStartOffensePosition() {
        //XXX: hardcoded starting positions
        const float[][] positions={{-20,0,0}, //robot 1 - kickoff robot!
                                   {-20,90,0}, //robot 2
                                   {-50,-90,0}, //etc
                                   {20,-90,0},
                                   {-50,90,0},
                                   {-150,0,0}}
        vector<float> result(3,0);
        int teamPosition = Blackboard->GameInfo->getPlayerNumber();
        
        result[0] = positions[teamPosition][0];
        result[1] = positions[teamPosition][1];
        result[2] = positions[teamPosition][2];
        
        
        return result;
    }
    
    /*! @brief Returns the non-kickoff start position on the field.
     */
    static vector<float> getStartDefensePosition() {
        //XXX: hardcoded starting positions
        const float[][] positions={{-70,0,0}, //robot 1 - kickoff robot!
                                   {-20,90,0}, //robot 2
                                   {-50,-90,0}, //etc
                                   {20,-90,0},
                                   {-50,90,0},
                                   {-150,0,0}}
        vector<float> result(3,0);
        int teamPosition = Blackboard->GameInfo->getPlayerNumber();
        
        result[0] = positions[teamPosition][0];
        result[1] = positions[teamPosition][1];
        result[2] = positions[teamPosition][2];
        
        
        return result;
    }
    
    static vector<float> fieldLocalisationPosition() {
        vector<float> result(3,0);
        
        result[2] = 0.5; //XXX: fix this to be less lazy
        
        return result;
    }
    
    static vector<float> ballLocalisationPosition() {
        return fieldLocalisationPosition(); //XXX: make this not so lazy
    }
    
}

#endif

