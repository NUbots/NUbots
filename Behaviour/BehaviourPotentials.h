/*! @file BehaviourPotentials.h
    @brief Declaration of the behaviour motor schemas (aka potentials or vector fields)
 
    Functions in this file return a vector (trans_speed, trans_direction, rotational_speed).

    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#ifndef BEHAVIOUR_POTENTIALS_H
#define BEHAVIOUR_POTENTIALS_H

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <vector>
#include <string>
using namespace std;

class BehaviourPotentials 
{
public:
    /*! @brief Returns a vector to go to a field state 
        @param self the self field object
        @param fieldstate the absolute position on the field [x(cm), y(cm), heading(rad)]
        @param stoppeddistance the distance in cm to the target at which the robot will stop walking, ie the accurarcy required.
        @param stoppingdistance the distance in cm from the target the robot will start to slow
        @param turningdistance the distance in cm from the target the robot will start to turn to face the desired heading
     */
    static vector<float> goToFieldState(Self& self, const vector<float>& fieldstate, float stoppeddistance = 0, float stoppingdistance = 50, float turningdistance = 70)
    {
        vector<float> relativestate = self.CalculateDifferenceFromFieldState(fieldstate);
        return goToPoint(relativestate[0], relativestate[1], relativestate[2], stoppeddistance, stoppingdistance, turningdistance);
    }
    
    /*! @brief Returns a vector to go to a field state 
        @param distance to the distance to the point
        @param bearing to the point
        @param heading the desired heading at the point
        @param stoppeddistance the distance in cm to the target at which the robot will stop walking, ie the accurarcy required.
        @param stoppingdistance the distance in cm from the target the robot will start to slow
        @param turningdistance the distance in cm from the target the robot will start to turn to face the desired heading
     */
    static vector<float> goToPoint(float distance, float bearing, float heading, float stoppeddistance = 0, float stoppingdistance = 50, float turningdistance = 70)
    {
        static const float m_HYSTERESIS = 0.4;      // the fraction of hysteresis in the turning point toward the desired heading
        static double m_previous_time = 0;          // the previous time in ms
        static bool m_previous_turning = false;     // the previous turning state; used to implement hysteresis in the decision to turn around.
        
        if (Blackboard->Sensors->CurrentTime - m_previous_time > 500)
        {
            m_previous_turning = false;
        }
        
        vector<float> result(3,0);
        if (distance > stoppeddistance or fabs(heading) > 0.1)
        {
            // calculate the translational speed
            if (distance < stoppingdistance)
                result[0] = 0.95*pow(distance/stoppingdistance,2) + 0.05;
            else
                result[0] = 1;
            
            // calculate the translational direction
            result[1] = bearing;
            
            // calculate the rotational speed
            if (distance < turningdistance or (m_previous_turning and distance < (1+m_HYSTERESIS)*turningdistance))
            {   // We use a bit of hysteresis in the turning point, once the turn has started its best to stick with it
                if (fabs(heading) > 3)
                    heading = fabs(heading);
                result[2] = 0.5*heading;
                m_previous_turning = true;
            }
            else
            {
                result[2] = 0.5*bearing;
                m_previous_turning = false;
            }
        }
        m_previous_time = Blackboard->Sensors->CurrentTime;
        
        debug << distance << "," << result[0] << "," << result[1] << "," << result[2] << endl;
        return result;
    }
    
    /*! @brief Returns a vector to avoid a field state 
        @param self the self field object
        @param fieldstate the absolute position on the field [x(cm), y(cm)]
        @param objectsize the radius in cm of the object to avoid
        @param dontcaredistance the distance in cm at which I make no attempt to avoid the object
     */
    static vector<float> avoidFieldState(Self& self, vector<float>& fieldstate, float objectsize = 25, float dontcaredistance = 100)
    {
        vector<float> result(3,0);
        if (fieldstate.size() < 3)
            fieldstate.push_back(0);
        vector<float> relativestate = self.CalculateDifferenceFromFieldState(fieldstate);
        
        float distance = relativestate[0];
        float bearing = relativestate[1];
        
        if (distance > dontcaredistance)
        {   // if the object is too far away don't avoid it
            return result;
        }
        else
        {
            // calculate the translational speed --- max if inside the object and reduces to zero at dontcaredistance
            if (distance < objectsize)
                result[0] = 1;
            else
                result[0] = (distance - dontcaredistance)/(objectsize - dontcaredistance);
            // calculate the translational bearing --- away
            if (fabs(bearing) < 0.1)
                result[1] = mathGeneral::PI/2;
            else
                result[1] = bearing - mathGeneral::sign(bearing)*mathGeneral::PI/2;
            // calculate the rotational speed --- spin facing object if infront, spin away if behind
            float y = distance*sin(bearing);
            float x = distance*cos(bearing);
            if (fabs(y) < objectsize)
                result[2] = atan2(y - mathGeneral::sign(y)*objectsize, x);
            else
                result[2] = 0;
            
            return result;
        }
    }
    
    /*! @brief Returns a vector to go to a ball
     */
    static vector<float> goToBall(MobileObject& ball, Self& self, float heading, float kickingdistance = 15.0, float stoppingdistance = 65)
    {
        vector<float> ball_prediction = self.CalculateClosestInterceptToMobileObject(ball);
        if (false)//ball_prediction[0] < 4 and ball.estimatedDistance() > 30)
        {   // if the ball is moving go to where the ball will be!
            float x = ball_prediction[1];
            float y = ball_prediction[2];
            float distance = sqrt(x*x + y*y);
            float bearing = atan2(y,x);
            
            vector<float> speed = goToPoint(distance, bearing, 0, 0, 0, distance+9000);
            
            #if DEBUG_BEHAVIOUR_VERBOSITY > 1
                debug << "goToBall Predicated x:" << x << " y: " << y << " ballx: " << ball.estimatedDistance()*cos(heading) << " bally: " << ball.estimatedDistance()*sin(heading) << endl;
            #endif
            return speed;
        }
        else
        {
            float distance = ball.estimatedDistance()*cos(ball.estimatedElevation());
            float bearing = ball.estimatedBearing();

            float x = distance * cos(bearing);
            float y = distance * sin(bearing);
            
            const float offsetDistance = 5.0f;
            float left_foot_x = x + offsetDistance * cos(heading - mathGeneral::PI/2);
            float left_foot_y = y + offsetDistance * sin(heading - mathGeneral::PI/2);
            float left_foot_distance = sqrt(pow(left_foot_x,2) + pow(left_foot_y,2));
            
            float right_foot_x = x + offsetDistance * cos(heading + mathGeneral::PI/2);
            float right_foot_y = y + offsetDistance * sin(heading + mathGeneral::PI/2);
            float right_foot_distance = sqrt(pow(right_foot_x,2) + pow(right_foot_y,2));
      
            if(left_foot_distance < right_foot_distance)
            {   // if the calculated left foot position is closer, then pick that one
                x = left_foot_x;
                y = left_foot_y;
            }
            else
            {
                x = right_foot_x;
                y = right_foot_y;
            }

            distance = sqrt(x*x + y*y);
            bearing = atan2(y,x);

            // calculate the component to position the ball at the kicking distance
            float position_speed;
            float position_direction;
            float position_rotation;
            if (distance < kickingdistance)
            {   // if we are too close to the ball then we need to go backwards
                position_speed = (kickingdistance - distance)/kickingdistance;
                position_direction = mathGeneral::normaliseAngle(bearing + mathGeneral::PI);
                position_rotation = 0.5*bearing;
            }
            else if (distance < stoppingdistance)
            {   // if we are close enough to slow down
                position_speed = (distance - kickingdistance)/(stoppingdistance - kickingdistance);
                position_direction = bearing;
                position_rotation = 0.5*bearing;
            }
            else
            {   // if it is outside the stopping distance - full speed
                position_speed = 1;
                position_direction = bearing;
                position_rotation = 0.5*bearing;
            }
            
            // calculate the component to go around the ball to face the heading
            float around_speed;
            float around_direction;
            float around_rotation;
            if (distance < 0.75*stoppingdistance)
            {   // if we are close enough to worry about the heading
                const float heading_gain = 0.5;
                const float heading_threshold = mathGeneral::PI/2;
                if (fabs(heading) < heading_threshold)
                    around_speed = (heading_gain/heading_threshold)*fabs(heading);
                else
                    around_speed = heading_gain;
                if (fabs(heading) > 2.85)
                    around_direction = mathGeneral::normaliseAngle(bearing + mathGeneral::PI/2);
                else
                    around_direction = mathGeneral::normaliseAngle(bearing - mathGeneral::sign(heading)*mathGeneral::PI/2);
                
                around_rotation = -mathGeneral::sign(around_direction)*around_speed*12/distance;        // 11 is rough speed in cm/s
            }
            else
            {
                around_speed = 0;
                around_direction = 0;
                around_rotation = 0;
            }
            
            vector<float> speed(3,0);
            speed[0] = max(position_speed, around_speed);
            float xsum = position_speed*cos(position_direction) + around_speed*cos(around_direction);
            float ysum = position_speed*sin(position_direction) + around_speed*sin(around_direction);
            speed[1] = atan2(ysum, xsum);
            speed[2] = position_rotation + around_rotation;
            return speed;
        }
    }
    
    /*! @brief Returns a the vector sum of the potentials
        @param potentials a list of [trans_speed, trans_direction, rot_speed] vectors
     */
    static vector<float> sumPotentials(const vector<vector<float> >& potentials)
    {
        float xsum = 0;
        float ysum = 0;
        float yawsum = 0;
        float maxspeed = 0;
        for (size_t i=0; i<potentials.size(); i++)
        {
            if (potentials[i][0] > maxspeed)
                maxspeed = potentials[i][0];
            xsum += potentials[i][0]*cos(potentials[i][1]);
            ysum += potentials[i][0]*sin(potentials[i][1]);
            yawsum += potentials[i][2];
        }
        vector<float> result(3,0);
        result[0] = maxspeed;
        result[1] = atan2(ysum,xsum);
        result[2] = yawsum;
        return result;
    }
    
    /*! @brief Returns a vector as close to the original as possible without hitting obstacles detected by the sensors
        @param speed the desired speed as [trans_speed, trans_direction, rot_speed]
     */
    static vector<float> sensorAvoidObjects(const vector<float>& speed, NUSensorsData* sensors, float objectsize = 20, float dontcaredistance = 50)
    {
        // Get obstacle distances from the sensors
        vector<float> temp;
        float leftobstacle = 255;
        float rightobstacle = 255;
        if (sensors->get(NUSensorsData::LDistance, temp) and temp.size() > 0)
            leftobstacle = temp[0];
        if (sensors->get(NUSensorsData::RDistance, temp) and temp.size() > 0)
            rightobstacle = temp[0];
        
        if (fabs(speed[1]) > mathGeneral::PI/2)
        {   // if the speed is not in the range of the ultrasonic sensors then don't both dodging
            return speed;
        }
        else if (leftobstacle > dontcaredistance and rightobstacle > dontcaredistance)
        {   // if the obstacles are too far away don't dodge
            return speed;
        }
        else
        {   // an obstacle needs to be dodged
            vector<float> newspeed = speed;
            float obstacle = min(leftobstacle, rightobstacle);
            float dodgeangle;
            if (obstacle < objectsize)          // if we are 'inside' the object
                dodgeangle = mathGeneral::PI/2 + asin((objectsize - obstacle)/objectsize);
            else                                // if we are 'outside' the object
                dodgeangle = asin(objectsize/obstacle);
            
            if (leftobstacle <= rightobstacle)
            {   // the obstacle is on the left
                if (speed[1] > -dodgeangle)
                    newspeed[1] = -dodgeangle;
            }
            else
            {   // the obstacle is on the right
                if (speed[1] < dodgeangle)
                    newspeed[1] = dodgeangle;
            }
            return newspeed;
        }
    }
    
    /*! @brief Returns the opponent's goal */
    static StationaryObject& getOpponentGoal(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        StationaryObject& bluegoal = fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& yellowgoal = fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        if (gameinfo->getTeamColour() == GameInformation::RedTeam)
            return bluegoal;
        else
            return yellowgoal;
    }
    
    /*! @brief Returns the relative position of the opponent's goal [distance, bearing] */
    static vector<float> getOpponentGoalPosition(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        StationaryObject& opponentgoal = getOpponentGoal(fieldobjects, gameinfo);
        return fieldobjects->self.CalculateDifferenceFromGoal(opponentgoal);
    }
    
    /*! @brief Returns the bearing to the opponent's goal */
    static float getBearingToOpponentGoal(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        vector<float> position = getOpponentGoalPosition(fieldobjects, gameinfo);
        return position[1];
    }
    
    /*! @brief Returns your own goal */
    static StationaryObject& getOwnGoal(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        StationaryObject& bluegoal = fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& yellowgoal = fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        if (gameinfo->getTeamColour() == GameInformation::RedTeam)
            return yellowgoal;
        else
            return bluegoal;
    }
    
    /*! @brief Return the relative position of your own goal [distance, bearing] */
    static vector<float> getOwnGoalPosition(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        StationaryObject& owngoal = getOwnGoal(fieldobjects, gameinfo);
        return fieldobjects->self.CalculateDifferenceFromGoal(owngoal);
    }
    
    /*! @brief Returns the bearing to your own goal */
    static float getBearingToOwnGoal(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        vector<float> position = getOwnGoalPosition(fieldobjects, gameinfo);
        return position[1];
    }

    /*! @brief Returns the [x,y] of the support player position */
    static vector<float> CalculateSupportPlayerPosition(MobileObject& ball, Self& self, float distancefromball = 100)
    {
        // we calculate the position in field coordinates, then convert to local cartesian
        vector<float> targetposition(3,0);
        targetposition[0] = ball.X();
        if (fabs(targetposition[0]) > 180)          // clip the target position to 1.2m from the goal
            targetposition[0] = mathGeneral::sign(targetposition[0])*180;
        
        // I need a cost metric here that includes the current position of the robot, so that it does not cross the field unnecessarily
        // b_y > 0 probably choose right, b_y < 0 probably choose left, 
        // if s_y < b_y probably choose right s_y > b_y probably choose left
        float b_y = ball.Y(); 
        float s_y = self.wmY();
        float cost = -b_y + 1.0*(s_y - b_y);
        if (cost < 0)
            targetposition[1] = b_y - distancefromball;
        else
            targetposition[1] = b_y + distancefromball;
        
        // convert to relative coords
        vector<float> polar = self.CalculateDifferenceFromFieldLocation(targetposition);
        
        // convert to cartesian
        vector<float> cartesian(2,0);
        cartesian[0] = polar[0]*cos(polar[1]);
        cartesian[1] = polar[0]*sin(polar[1]);
        return cartesian;
    }

    /*! @brief Returns true if goal is lined up, false if it is not. */
    static bool opponentsGoalLinedUp(FieldObjects* fieldobjects, GameInformation* gameinfo)
    {
        StationaryObject* targetGoalLeftPost;
        StationaryObject* targetGoalRightPost;
        Self& self = fieldobjects->self;
        if (gameinfo->getTeamColour() == GameInformation::RedTeam)
        {
            targetGoalLeftPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST]);
            targetGoalRightPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]);
        }
        else
        {
            targetGoalLeftPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST]);
            targetGoalRightPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]);
        }
        float leftGoalBearing = self.CalculateBearingToStationaryObject(*targetGoalLeftPost);
        float rightGoalBearing = self.CalculateBearingToStationaryObject(*targetGoalRightPost);
        float middleBearing = leftGoalBearing + rightGoalBearing / 2.0f;
        return ((leftGoalBearing > 0.2f) && (rightGoalBearing < -0.2f)) || (fabs(middleBearing) < mathGeneral::PI/16.0f);
    }
};


#endif

