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
        @param distance to the distance to the point
        @param bearing to the point
        @param heading the desired heading at the point
        @param stoppeddistance the distance in cm to the target at which the robot will stop walking, ie the accurarcy required.
        @param stoppingdistance the distance in cm from the target the robot will start to slow
        @param turningdistance the distance in cm from the target the robot will start to turn to face the desired heading
     */
    static vector<float> goToPointBackwards(float distance, float bearing, float heading, float stoppeddistance = 4.f, float stoppingdistance = 50, float turningdistance = 70)
    {
        static const float m_HYSTERESIS = 0.15;      // the fraction of hysteresis in the turning point toward the desired heading
        static double m_previous_time = 0;          // the previous time in ms
        static bool m_previous_turning = false;     // the previous turning state; used to implement hysteresis in the decision to turn around.
        static bool m_turning_left = false; 
        //bearing = mathGeneral::normaliseAngle(bearing+3.1416);
        Self self = Blackboard->Objects->self;
        vector<float> result(3,0);
        
        
    
        
        
        
        if (fabs(bearing) > 0.4 or (fabs(bearing) > m_HYSTERESIS and m_previous_turning)) { //turn with hysteresis
            m_previous_turning = true;
            if (bearing > 0. and bearing < 2.6) {
                result[2] = bearing*1.0;
                m_turning_left = true;
            } else if (bearing <= 0. and bearing > -2.6) {
                result[2] = bearing*0.8;
                m_turning_left = false;
            } else if (distance < stoppeddistance) {
                result[2] = heading*0.5;
            } else if (m_turning_left) {
                result[2] = 1.f;
            } else if (not m_turning_left) {
                result[2] = -1.f;
            }
            result[0] = -0.01f;
        } else { //run forward
            m_previous_turning = false;
            if (distance > stoppingdistance) {
                result[0] = 1.0;
            } else if (distance > stoppingdistance/2.f) {
                result[0] = 0.7;
            } else if (distance > stoppeddistance) {
                result[0] = 0.25;
            } else {
                result[2] = heading*0.5;
            }
            /*if (distance < stoppingdistance) {
                result[1] = bearing;
                }*/
        }
        
        //cout << result[0] << ", " << result[1] << ", " << result[2] << endl;
        //cout << distance << ", " << bearing << ", " << heading << endl;
       /* if (Blackboard->Sensors->CurrentTime - m_previous_time > 500)
        {
            m_previous_turning = false;
        }
        
        if (distance > stoppeddistance or fabs(heading) > 0.05)
        {
            // calculate the translational speed
            if (distance < stoppingdistance)
                result[0] = distance/stoppingdistance;
            else
                result[0] = 1;
            
            // calculate the translational direction
            //result[1] = bearing;
            
            // calculate the rotational speed
            if (bearing > 0.1 or (m_previous_turning and bearing < .1+m_HYSTERESIS))
            {   // We use a bit of hysteresis in the turning point, once the turn has started its best to stick with it
                if (fabs(heading) > 3)
                    heading = fabs(heading);
                result[2] = 0.5*heading;
                result[0] = 0.;
                m_previous_turning = true;
            }
            else
            {
                result[2] = 0.f;//5*bearing;
                m_previous_turning = false;
            }
        }
        m_previous_time = Blackboard->Sensors->CurrentTime;*/
        return result;
    }

    /*! @brief Returns a vector to go to a field state 
        @param self the self field object
        @param fieldstate the absolute position on the field [x(cm), y(cm), heading(rad)]
        @param stoppeddistance the distance in cm to the target at which the robot will stop walking, ie the accurarcy required.
        @param stoppingdistance the distance in cm from the target the robot will start to slow
        @param turningdistance the distance in cm from the target the robot will start to turn to face the desired heading
     */
    static vector<float> goToFieldState(Self& self, const vector<float>& fieldstate, float stoppeddistance = 4.f, float stoppingdistance = 30, float turningdistance = 70)
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
    static vector<float> goToPoint(float distance, float bearing, float heading, float stoppeddistance = 4.f, float stoppingdistance = 50, float turningdistance = 70)
    {
        static const float m_HYSTERESIS = 0.1;      // the fraction of hysteresis in the turning point toward the desired heading
        static double m_previous_time = 0;          // the previous time in ms
        static bool m_previous_turning = false;     // the previous turning state; used to implement hysteresis in the decision to turn around.
        static bool m_turning_left = false; 
        
        Self self = Blackboard->Objects->self;
        vector<float> result(3,0);
        
        //Modify bearing based on objects in the way
        vector<AmbiguousObject> objects = Blackboard->Objects->ambiguousFieldObjects;
        float goaldistance = 80.f;

        for(unsigned int i=0; i<objects.size(); i++) { //for each object
            if (objects[i].isObjectAPossibility(FieldObjects::FO_OBSTACLE) and objects[i].measuredDistance() < goaldistance) { //if we are an obstacle
                if (objects[i].measuredBearing() > bearing and objects[i].measuredBearing()-objects[i].arc_width < bearing) { //if we are on the right and occluding
                    bearing = objects[i].measuredBearing()-objects[i].arc_width;
                } else if (objects[i].measuredBearing() < bearing and objects[i].measuredBearing()+objects[i].arc_width > bearing) { //if we are on the left and occluding
                    bearing = objects[i].measuredBearing()+objects[i].arc_width;
                }
            }
        }
        
        
        
        if (fabs(bearing) > 0.3 or (fabs(bearing) > m_HYSTERESIS and m_previous_turning and distance > stoppeddistance*1.5)) { //turn with hysteresis
            m_previous_turning = true;
            if (bearing > 0. and bearing < 2.6) {
                result[2] = bearing*0.5;
                m_turning_left = true;
            } else if (bearing <= 0. and bearing > -2.6) {
                result[2] = bearing*0.5;
                m_turning_left = false;
            } else if (distance < stoppeddistance) {
                result[2] = heading*0.5;
            } else if (m_turning_left) {
                result[2] = 1.f;
            } else if (not m_turning_left) {
                result[2] = -1.f;
            }
            result[0] = -0.1f;
        } else { //run forward
            m_previous_turning = false;
            if (distance > stoppingdistance) {
                result[0] = 1.0;
            } else if (distance > stoppingdistance/2.f) {
                result[0] = 0.65;
            } else if (distance > stoppeddistance) {
                result[0] = 0.25;
            } else {
                result[2] = heading*0.5;
                result[1] = mathGeneral::PI/2.f*mathGeneral::sign(-heading);
                result[0] = 0.1;
            }
        }
        
        //cout << result[0] << ", " << result[1] << ", " << result[2] << endl;
        //cout << distance << ", " << bearing << ", " << heading << endl;
       /* if (Blackboard->Sensors->CurrentTime - m_previous_time > 500)
        {
            m_previous_turning = false;
        }
        
        if (distance > stoppeddistance or fabs(heading) > 0.05)
        {
            // calculate the translational speed
            if (distance < stoppingdistance)
                result[0] = distance/stoppingdistance;
            else
                result[0] = 1;
            
            // calculate the translational direction
            //result[1] = bearing;
            
            // calculate the rotational speed
            if (bearing > 0.1 or (m_previous_turning and bearing < .1+m_HYSTERESIS))
            {   // We use a bit of hysteresis in the turning point, once the turn has started its best to stick with it
                if (fabs(heading) > 3)
                    heading = fabs(heading);
                result[2] = 0.5*heading;
                result[0] = 0.;
                m_previous_turning = true;
            }
            else
            {
                result[2] = 0.f;//5*bearing;
                m_previous_turning = false;
            }
        }
        m_previous_time = Blackboard->Sensors->CurrentTime;*/
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
    static vector<float> goToBallDirectWithSidewardsKick(MobileObject& ball, Self& self, float heading, float kickingdistance = 15.0, float stoppingdistance = 65)
    {
        std::vector<float> speed(3, 0.0f);  // [Magnitude, Direction, Rotation]

        float ballx, bally;
        ballx = ball.X();
        bally = ball.Y();

        // Get the target goal (x,y) position.
        float goalx, goaly;
        if (Blackboard->GameInfo->getTeamColour() == GameInformation::RedTeam)
        {
            StationaryObject& leftpost = Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
            StationaryObject& rightpost = Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
            goalx = (leftpost.X() + rightpost.X()) / 2.0f;
            goaly = (leftpost.Y() + rightpost.Y()) / 2.0f;
        }
        else
        {
            StationaryObject& leftpost = Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
            StationaryObject& rightpost = Blackboard->Objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
            goalx = (leftpost.X() + rightpost.X()) / 2.0f;
            goaly = (leftpost.Y() + rightpost.Y()) / 2.0f;
        }

        // Calculate the three possible kicking positions.
        float A[2], B[2], C[2];
        float distance_between_legs_on_2 = 10.0f / 2.0f;
        float robot_foot_size_on_2 = 0.f; //10.f/2.f;

        A[0] = goalx;
        A[1] = goaly;

        B[0] = ballx;
        B[1] = bally;

        // Forward Kick
        mathGeneral::ProjectFromAtoB(A, B, 1, C);

        float x_diff = C[0] - B[0];
        float y_diff = C[1] - B[1];

        float fwd_angle = atan2(-y_diff, -x_diff);

        float ball_bearing;
        float ball_distance;
        if(ball.isObjectVisible())
        {
            ball_bearing = ball.measuredBearing();
            ball_distance = ball.measuredDistance();
        }
        else
        {
            ball_bearing = ball.estimatedBearing();
            ball_distance = ball.estimatedDistance();
        }
        bool offset_sign = mathGeneral::sign(ball_bearing);
        
        kickingdistance = kickingdistance - robot_foot_size_on_2;
        float fwd_x = C[0] + x_diff * kickingdistance;
        float fwd_y = C[1] + y_diff * kickingdistance;


        fwd_x += -offset_sign * y_diff * distance_between_legs_on_2;
        fwd_y += offset_sign * x_diff * distance_between_legs_on_2;

        // Left side Kick (facing goal)
        float left_side_x = ballx - y_diff * kickingdistance - x_diff * distance_between_legs_on_2 * 0.5;
        float left_side_y = bally + x_diff * kickingdistance - y_diff * distance_between_legs_on_2 * 0.5;

        // Right side Kick (facing goal)
        float right_side_x = ballx + y_diff * kickingdistance - x_diff * distance_between_legs_on_2 * 0.5;
        float right_side_y = bally - x_diff * kickingdistance - y_diff * distance_between_legs_on_2 * 0.5;

        // Now get the closest kicking position

        float my_x = self.wmX();
        float my_y = self.wmY();
        float my_heading = self.Heading();

        float fwd_pos_distance_metric = fabs(my_x - fwd_x) + fabs(my_y - fwd_y)-2.;
        float left_pos_distance_metric = fabs(my_x - left_side_x) + fabs(my_y - left_side_y);
        float right_pos_distance_metric = fabs(my_x - right_side_y) + fabs(my_y - right_side_y);

        float best_kicking_pos_x;
        float best_kicking_pos_y;
        float best_kicking_orientation;
        // Forwards best
        if( (fwd_pos_distance_metric < left_pos_distance_metric) and (fwd_pos_distance_metric < right_pos_distance_metric) )
        {
            best_kicking_pos_x = fwd_x;
            best_kicking_pos_y = fwd_y;
            best_kicking_orientation = fwd_angle;
        }
        // Left best
        else if( (left_pos_distance_metric < fwd_pos_distance_metric) and (left_pos_distance_metric < right_pos_distance_metric) )
        {
            best_kicking_pos_x = left_side_x;
            best_kicking_pos_y = left_side_y;
            best_kicking_orientation = fwd_angle - 0.5*mathGeneral::PI;
        }
        // Right best
        else if( (right_pos_distance_metric < fwd_pos_distance_metric) and (right_pos_distance_metric < left_pos_distance_metric) )
        {
            best_kicking_pos_x = right_side_x;
            best_kicking_pos_y = right_side_y;
            best_kicking_orientation = fwd_angle + 0.5*mathGeneral::PI;
        }

        x_diff = best_kicking_pos_x - my_x;
        y_diff = best_kicking_pos_y - my_y;
        float dist_to_kick_pos = sqrt(pow(x_diff,2) + pow(y_diff, 2));
        float angle_to_kick_pos = mathGeneral::normaliseAngle(atan2(y_diff, x_diff) - my_heading);
        float angle_to_aim_pos = mathGeneral::normaliseAngle(best_kicking_orientation - my_heading);
        //cout << "Approach angle: " << angle_to_kick_pos << "\tKick angle: " << angle_to_aim_pos << "\tHeading: " << my_heading << endl;
        //cout << "Ball RelX: " << x_diff << "\tBall RelY: " << y_diff << "\tBall Heading: " << atan2(y_diff, x_diff) << endl;
        //cout << "Ball Goal Bearing: " << fwd_angle << "\tkick orientation: " << best_kicking_orientation << endl;
        //cout << "Goal RelX: " << goalx-my_x << "\tGoal RelY: " << goaly-my_y << "\tBall Distance: " << ball_distance << endl;

        static bool turning = false;
        static bool turningLeft = false;
        float target_heading = (ball_distance > stoppingdistance) ? angle_to_kick_pos : ball_bearing; //angle_to_aim_pos;
        
        //Bearing Alteration
        vector<AmbiguousObject> objects = Blackboard->Objects->ambiguousFieldObjects;

        for(unsigned int i=0; i<objects.size(); i++) { //for each object
            if (objects[i].isObjectAPossibility(FieldObjects::FO_OBSTACLE) and objects[i].measuredDistance() > ball_distance) { //if we are an obstacle
                if (objects[i].measuredBearing() > target_heading and objects[i].measuredBearing()-objects[i].arc_width < target_heading) { //if we are on the right and occluding
                    target_heading = objects[i].measuredBearing()-objects[i].arc_width;
                } else if (objects[i].measuredBearing() < target_heading and objects[i].measuredBearing()+objects[i].arc_width > target_heading) { //if we are on the left and occluding
                    target_heading = objects[i].measuredBearing()+objects[i].arc_width;
                }
            }
        }
        
        //hysteresis for 180 degrees out of phase
        if (target_heading > 0.f and target_heading < 3.f) {
            turningLeft = true;
        } else if (target_heading < 0.f and target_heading > -3.f) {
            turningLeft = false;
        }
        if (turningLeft and target_heading < 0. or (not turningLeft) and target_heading > 0.) {
            target_heading *= -1.f;
        }

        if(turning and 
            (fabs(target_heading) < 0.1 or fabs(target_heading) < 0.3 and ball_distance > stoppingdistance/2.))
        {
            turning = false;
        }
        else if (not turning and fabs(target_heading) > 0.5)
        {
            turning = true;
        }

        if(turning)
        {
            speed[0] = 0.03f; // 10% speed
            speed[1] = -mathGeneral::sign(target_heading)*3.1;
            speed[2] = 0.6*target_heading;
            if (ball_distance < kickingdistance * 1.5) {
                speed[2] = 0.3*target_heading;
            }
            /*
            std::cout << "Calculated heading: " << -atan2(y_diff, x_diff) << std::endl;
            std::cout << "My heading: " << my_heading << std::endl;
            std::cout << "Heading to Ball: " << ball_bearing << std::endl;
            std::cout << "Ball Position: (" << ballx << ", " << bally << ")" << std::endl;
            std::cout << "Kick Position: (" << best_kicking_pos_x << ", " << best_kicking_pos_y << ")" << std::endl;
            std::cout << angle_to_kick_pos << std::endl;
            std::cout << "Turning." << std::endl;*/
        }
        else if(ball_distance > stoppingdistance)
        {
            // Full speed ahead!
            speed[0] = 1.0f; // 100% speed.
            speed[1] = 0.0f;    // Straight
            speed[2] = 0.0f;    // Straight
//            std::cout << "Walking." << std::endl;
        } else if (ball_distance > kickingdistance) {
            speed[0] = 0.5f; // 100% speed.
            speed[1] = 0.2*ball_bearing;    // Straight
            speed[2] = 0.0f;    // Straight
        
        }
        //cout << speed[0] << ", " << speed[1] << ", " << speed[2] << endl;
        return speed;
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
            float distance = ball.estimatedDistance(); //*cos(ball.estimatedElevation());
            float bearing = ball.estimatedBearing();
            




            float x = distance * cos(bearing);
            float y = distance * sin(bearing);
            
            float dist_hysteresis = 5.f;

            
            float offsetDistance = 3.0f;
            
            if (ball.isObjectVisible() and ball.TimeSeen() > 500.) {
                bearing = ball.measuredBearing();
                distance = ball.measuredDistance();//*cos(ball.measuredElevation();
            }
            
            
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
            
            /*float approachOffset = 20.;
            if (Blackboard->GameInfo->getTeamColour() == GameInformation::BlueTeam and distance > (kickingdistance + approachOffset)) {
                x -= approachOffset-dist_hysteresis;
            } else if (distance > kickingdistance + approachOffset) {
                x += approachOffset-dist_hysteresis;
            }*/
            
            //change bearing to align to ball
            vector<float> goalPosition(3,0);
            float bearingLineUpSide = 0.3;
            float bearingLineUpFront = 0.45;
            goalPosition = self.CalculateDifferenceFromGoal(getOpponentGoal(Blackboard->Objects, Blackboard->GameInfo));
            if (goalPosition[1] > mathGeneral::PI/4.f) { //to our left > 45 degrees
                bearing -= bearingLineUpSide*(kickingdistance-2.)/distance;
            } else if (goalPosition[1] < -mathGeneral::PI/4.f) { //to our right > 45 degrees
                bearing += bearingLineUpSide*(kickingdistance-2.)/distance;
            } else {
                bearing += mathGeneral::sign(bearing)*bearingLineUpFront*(kickingdistance-2.)/distance;
            }
            

            distance = sqrt(x*x + y*y);
            bearing = atan2(y,x);

            // calculate the component to position the ball at the kicking distance
            float position_speed;
            float position_direction;
            float position_rotation;
            if (distance < kickingdistance)
            {   // if we are too close to the ball then we need to go backwards
                position_speed = 0.1;//(kickingdistance - distance)/(kickingdistance);
                position_direction = mathGeneral::normaliseAngle(bearing + mathGeneral::PI);
                
                //position_rotation = 0.5*bearing; //previous value for NAO
                if (bearing < mathGeneral::PI/8.) {
                    position_rotation = mathGeneral::sign(heading)*0.3;
                } else if (bearing > mathGeneral::PI/3.) {
                    position_rotation = mathGeneral::sign(bearing)*0.3;
                } else {
                    position_rotation = 0.;//mathGeneral::sign(bearing)*0.3;
                }
                //speed up if we're too slow at shuffling
                /*if (fabs(bearing) > 0.8f and position_speed < 0.3 and fabs() ) {
                    position_speed += 0.1;
                } else if (fabs(bearing) > 1.0f and position_speed < 0.3) {
                    position_speed += 0.15;
                } */
                
            }
            else if (distance < stoppingdistance)
            {   // if we are close enough to slow down
                
                position_speed = (distance - kickingdistance+3.)/(stoppingdistance - kickingdistance+3.);
                position_direction = bearing;
                //position_rotation = 0.5*bearing; //previous value for NAO
                position_rotation = 0.7*bearing;
                
                //added this to boost sidestepping ability when lining up
                if (position_speed < 0.3 and fabs(bearing) < mathGeneral::PI/3.) {
                    position_speed += 0.13;
                }
                
            }
            else
            {   // if it is outside the stopping distance - full speed
                if ( fabs (bearing) < 0.3 or (fabs (bearing) < 0.5) and distance > stoppingdistance) {
                    position_speed = 1.0;
                    position_direction = bearing*0.6;
                    position_rotation = 0.;//6*bearing;
                } else {
                    position_speed = 0.2;
                    position_direction = mathGeneral::normaliseAngle(bearing - mathGeneral::sign(heading)*mathGeneral::PI/2);
                    position_rotation = 0.5*bearing;
                }
                //position_rotation = 0.5*bearing; //previous value for NAO
            }
            
            // calculate the component to go around the ball to face the heading
            vector<float> speed(3,0);
            float around_speed;
            float around_direction;
            float around_rotation;
            if (distance < stoppingdistance and distance > 1.5*kickingdistance or distance < 1.15*kickingdistance)
            {   // if we are close enough to worry about the heading
                float heading_gain = 0.5;
                if (distance < 1.25*kickingdistance) {
                    heading_gain = 0.2;
                    }
                const float heading_threshold = mathGeneral::PI/3;
                if (fabs(heading) < heading_threshold)
                    around_speed = (heading_gain/heading_threshold)*fabs(heading);
                else
                    around_speed = heading_gain;
                if (fabs(heading) > 2.85)
                    around_direction = mathGeneral::normaliseAngle(bearing + mathGeneral::PI/2);
                else
                    around_direction = mathGeneral::normaliseAngle(bearing - mathGeneral::sign(heading)*mathGeneral::PI/2);
                //around_rotation = -mathGeneral::sign(around_direction)*around_speed*12/distance;        // previous value for NAO
                around_rotation = -mathGeneral::sign(around_direction)*around_speed/distance/2.;        // 11 is rough speed in cm/s
            }
            else
            {
                around_speed = 0;
                around_direction = 0;
                around_rotation = 0;
            }
            
            //vector<float> speed(3,0);
            
            
            speed[0] = max(position_speed, around_speed);
            //float xsum = position_speed*cos(position_direction) + around_speed*cos(around_direction);
            //float ysum = position_speed*sin(position_direction) + around_speed*sin(around_direction);
            speed[1] = (position_speed*position_direction + around_speed*around_direction)/(max(position_speed,0.3f)+around_speed);//atan2(ysum, xsum);
            speed[2] = (position_speed*position_rotation + around_speed*around_rotation)/(position_speed+around_speed);
            speed[2] = min(fabs(speed[2]),.3f)*mathGeneral::sign(speed[2]);
            
            /*cout << endl;
            cout << "Goal Heading: " << heading << "\tBall Distance: " << distance << endl;
            cout << "Goto Speed:   " << position_speed << "\t" << position_direction << "\t" << position_rotation << "\t" << endl;
            cout << "Around Speed: " << around_speed << "\t" << around_direction << "\t" << around_rotation << "\t" << endl;
            cout << "Final Speed:  " << speed[0] << "\t" << speed[1] << "\t" << speed[2] << "\t" << endl;*/
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

    /*! @brief Returns a vector of the left and right obstacles using either the ultrasonics
                on the NAOs or the Darwin Vision obstacles
     */
    static vector<float> getObstacleDistances(NUSensorsData* sensors)
    {
        float VIEW_ANGLE_RANGE = mathGeneral::PI/6;
        float OVERLAP_ANGLE = mathGeneral::PI/24;
        vector<float> result;
        vector<float> temp_l;
        vector<float> temp_r;
        float leftobstacle = 255;
        float rightobstacle = 255;

        // See if ultrasonic sensors are available
        if(sensors->get(NUSensorsData::LDistance, temp_l) and sensors->get(NUSensorsData::RDistance, temp_r))
        {
            //NAO
            if (temp_l.size() > 0)
                leftobstacle = temp_l[0];
            if (temp_r.size() > 0)
                rightobstacle = temp_r[0];
        }
        else
        {
            //DARWIN
            vector<AmbiguousObject> objects = Blackboard->Objects->ambiguousFieldObjects;
            AmbiguousObject tempobj;
            Vector3<float> temploc;

            for(unsigned int i=0; i<objects.size(); i++)
            {
                tempobj = objects.at(i);
                //check object was seen this frame
                if(tempobj.isObjectVisible())
                {
                    temploc = tempobj.getMeasuredRelativeLocation();
                    //check obstacle is within 120 degree cone
                    if(!(fabs(temploc.y) > VIEW_ANGLE_RANGE))
                    {
                        //check if obstacle is in front, on left or on right
                        if(fabs(temploc.y) < OVERLAP_ANGLE) {
                            //obstacle is within 15 degrees of centre - flag as left AND right obstacle
                            if(temploc.x < leftobstacle) {
                                leftobstacle = temploc.x;
                            }
                            if(temploc.x < rightobstacle) {
                                rightobstacle = temploc.x;
                            }
                        }
                        if(temploc.y > 0) {
                            //obstacle is to right
                            if(temploc.x < leftobstacle) {
                                leftobstacle = temploc.x;
                            }
                        }
                        else {
                            //obstacle is to left
                            if(temploc.x < rightobstacle) {
                                rightobstacle = temploc.x;
                            }
                        }
                    }
                }
            }
        }

        result.push_back(leftobstacle);
        result.push_back(rightobstacle);

        return result;
    }
    
    /*! @brief Returns a vector as close to the original as possible without hitting obstacles detected by the sensors
        @param speed the desired speed as [trans_speed, trans_direction, rot_speed]
     */
    static vector<float> sensorAvoidObjects(const vector<float>& speed, NUSensorsData* sensors, float objectsize = 40, float dontcaredistance = 75)
    {
        // Get obstacle distances from the sensors
        vector<float> obstacles = getObstacleDistances(sensors);
        float leftobstacle = obstacles.at(0);
        float rightobstacle = obstacles.at(1);
        
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

    /*! @brief Returns a vector as close to the original as possible without hitting obstacles detected by the sensors
                with provided obstacles - does not calculate its own obstacles
        @param speed the desired speed as [trans_speed, trans_direction, rot_speed]
     */
    static vector<float> sensorAvoidObjects(const vector<float>& speed, NUSensorsData* sensors, vector<float> obstacles, float objectsize = 40, float dontcaredistance = 75)
    {
        float leftobstacle = obstacles.at(0);
        float rightobstacle = obstacles.at(1);

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
    static vector<float> CalculateSupportPlayerPosition(MobileObject& ball, Self& self, float distancefromball = 140)
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
        std::string goalname;        
        if (gameinfo->getTeamColour() == GameInformation::RedTeam)
        {
            goalname = "Blue Goal";
            targetGoalLeftPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST]);
            targetGoalRightPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]);
        }
        else
        {
            goalname = "Yellow Goal";
            targetGoalLeftPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST]);
            targetGoalRightPost = &(fieldobjects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]);
        }
        float leftGoalBearing = self.CalculateBearingToStationaryObject(*targetGoalLeftPost);
        float rightGoalBearing = self.CalculateBearingToStationaryObject(*targetGoalRightPost);

        Vector2<float> location;
        location.x = (targetGoalLeftPost->X() + targetGoalRightPost->X()) / 2.0f;
        location.y = (targetGoalLeftPost->Y() + targetGoalRightPost->Y()) / 2.0f;
        StationaryObject middle(location);
        float middleBearing = self.CalculateBearingToStationaryObject(middle);

        bool result_posts = ((leftGoalBearing > 0.f) && (rightGoalBearing < -0.f));
        bool result_centre = (fabs(middleBearing) < mathGeneral::PI/12.0f);
        
        /*
        if(result_posts || result_centre)
        {
            debug << "%Kick lined up at time: " << Blackboard->Sensors->CurrentTime << "Goal: " << goalname << std::endl;
            debug << "%My Heading: " << self.Heading() << std::endl;
            debug << "%Left Post Bearing: " << leftGoalBearing << std::endl;
            debug << "%Right Post Bearing: " << rightGoalBearing << std::endl;
            debug << "%Middle Bearing: " << middleBearing << std::endl;
            debug << "%Post Result: " << result_posts << std::endl;
            debug << "%Centre Result: " << result_centre << std::endl;
        }
        */
        return result_posts || result_centre;
    }
};


#endif


