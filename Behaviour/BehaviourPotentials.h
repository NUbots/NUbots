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

#ifndef BEHAVIOUR_SCHEMAS_H
#define BEHAVIOUR_SCHEMAS_H

#include "Vision/FieldObjects/Self.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Tools/Math/General.h"

#include "debug.h"

#include <vector>
#include <string>
using namespace std;

namespace BehaviourPotentials 
{
    /*! @brief Returns a vector to go to a field state 
        @param self the self field object
        @param fieldstate the absolute position on the field [x(cm), y(cm), heading(rad)]
        @param stoppeddistance the distance in cm to the target at which the robot will stop walking, ie the accurarcy required.
        @param stoppingdistance the distance in cm from the target the robot will start to slow
        @param turningdistance the distance in cm from the target the robot will start to turn to face the desired heading
     */
    vector<float> goToFieldState(Self& self, const vector<float>& fieldstate, float stoppeddistance = 0, float stoppingdistance = 20, float turningdistance = 60)
    {
        vector<float> result(3,0);
        vector<float> relativestate = self.CalculateDifferenceFromFieldState(fieldstate);
        
        float distance = relativestate[0];
        float bearing = relativestate[1];
        float heading = relativestate[2];
        
        if (distance < stoppeddistance and fabs(heading) < 0.1)         // if we are close --- enough stop
            return result;
        else
        {
            // calculate the translational speed
            if (distance < stoppingdistance)
                result[0] = distance/stoppingdistance;
            else
                result[0] = 1;
            // 'calculate' the translational direction
            result[1] = bearing;
            // calculate the rotational speed
            if (distance < turningdistance)
                result[2] = 0.8*heading;
            else
                result[2] = 0.8*bearing;
            return result;
        }
    }
    
    /*! @brief Returns a vector to avoid a field state 
        @param self the self field object
        @param fieldstate the absolute position on the field [x(cm), y(cm)]
        @param objectsize the radius in cm of the object to avoid
        @param dontcaredistance the distance in cm at which I make no attempt to avoid the object
     */
    vector<float> avoidFieldState(Self& self, vector<float>& fieldstate, float objectsize = 25, float dontcaredistance = 100)
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
    
    /*! @brief Returns a the vector sum of the potentials
        @param potentials a list of [trans_speed, trans_direction, rot_speed] vectors
     */
    vector<float> sumPotentials(const vector<vector<float> >& potentials)
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
    vector<float> sensorAvoidObjects(const vector<float>& speed, NUSensorsData* sensors, float objectsize = 20, float dontcaredistance = 50)
    {
        // Get obstacle distances from the sensors
        vector<float> temp;
        float leftobstacle = 255;
        float rightobstacle = 255;
        if (sensors->getDistanceLeftValues(temp) and temp.size() > 0)
            leftobstacle = temp[0];
        if (sensors->getDistanceRightValues(temp) and temp.size() > 0)
            rightobstacle = temp[0];
        
        if (leftobstacle > dontcaredistance and rightobstacle > dontcaredistance)
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
}


#endif

