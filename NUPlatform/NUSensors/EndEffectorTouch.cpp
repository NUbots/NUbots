/*! @file EndEffectorTouch.cpp
    @brief Implementation of the EndEffectorTouch class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010, 2011 Jason Kulk
 
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
#include "EndEffectorTouch.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"

#include "debug.h"
#include "debugverbositynusensors.h"
#include "nubotdataconfig.h"

#include <math.h>
#include <limits>

/*! @brief Default constructor
 */
EndEffectorTouch::EndEffectorTouch()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "EndEffectorTouch::EndEffectorTouch()" << endl;
    #endif
    
    // Init the min and max force tracking
    m_id_offset = NUSensorsData::LArmEndEffector.Id;
    m_min_forces = vector<float>(4, numeric_limits<float>::infinity());
    m_max_forces = vector<float>(4, 0);
    
    // Load the support hulls for the end effectors
    ifstream file((CONFIG_DIR + string("Motion/SupportHull") + ".cfg").c_str());
    if (file.is_open())
    {
        vector<vector<float> > left_foot_hull;
        vector<vector<float> > right_foot_hull;
        vector<vector<float> > left_hand_hull;
        vector<vector<float> > right_hand_hull;
        file >> left_foot_hull;
        file >> right_foot_hull;
        file >> left_hand_hull;
        file >> right_hand_hull;
        #if DEBUG_NUSENSORS_VERBOSITY > 1
        	debug << "left_foot_hull: " << left_foot_hull << endl;
            debug << "right_foot_hull: " << right_foot_hull << endl;
            debug << "left_hand_hull: " << left_hand_hull << endl;
            debug << "right_hand_hull: " << right_hand_hull << endl;
        #endif
        m_hulls.push_back(left_hand_hull);
        m_hulls.push_back(right_hand_hull);
        m_hulls.push_back(left_foot_hull);
        m_hulls.push_back(right_foot_hull);
    }
    else
    {
        debug << "EndEffectorTouch::EndEffectorTouch(). WARNING: Unable to load SupportHull.cfg. This means you will have no centre of pressure and support sensors" << endl;
        errorlog << "EndEffectorTouch::EndEffectorTouch(). WARNING: Unable to load SupportHull.cfg. This means you will have no centre of pressure and support sensors" << endl;
    }
    file.close();
    
    m_Nan = numeric_limits<float>::quiet_NaN();
    m_Nan_all = vector<float>(NUSensorsData::CoPYId - NUSensorsData::ForceId, m_Nan);
}

/*! @brief Destructor
 */
EndEffectorTouch::~EndEffectorTouch()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "EndEffectorTouch::~EndEffectorTouch" << endl;
    #endif
}

/*! @brief Calculates the force, contact, centre of pressure and support sensors for each of the end effectors with touch sensors
 */
void EndEffectorTouch::calculate()
{
    calculate(NUSensorsData::LLegEndEffector);
    calculate(NUSensorsData::RLegEndEffector);
}

/*! @brief Calculates the force, contact, centre of pressure and support sensors for the given end effector
 	@param endeffector the id of the end effector to calculate
 */
void EndEffectorTouch::calculate(const NUData::id_t& endeffector)
{
    bool success = false;
    if (endeffector == NUSensorsData::LArmEndEffector)
        success = Blackboard->Sensors->get(NUSensorsData::LHandTouch, m_touch_data);
    else if (endeffector == NUSensorsData::RArmEndEffector)
        success = Blackboard->Sensors->get(NUSensorsData::RHandTouch, m_touch_data);
    else if (endeffector == NUSensorsData::LLegEndEffector)
        success = Blackboard->Sensors->get(NUSensorsData::LFootTouch, m_touch_data);
    else if (endeffector == NUSensorsData::RLegEndEffector)
        success = Blackboard->Sensors->get(NUSensorsData::RFootTouch, m_touch_data);
    else
        return;
    
    if (not success)
    {	// if there is no touch data we can't calculate anything
        invalidate(endeffector);
        return;
    }
    
    calculateForce(endeffector);
    calculateContact(endeffector);
    calculateCentreOfPressure(endeffector);
    calculateSupport(endeffector);
}

/*! @brief Calculates the total force on the end effector.
 	@param endeffector the id of the end effector the touch data is for, and where the resultant force will be stored
 	@param touchdata a vector containing the force data for the pressure sensor array
 */
void EndEffectorTouch::calculateForce(const NUData::id_t& endeffector)
{
    // sum the forces
    float result = 0;
    for (size_t i=0; i<m_touch_data.size(); i++)
        result += m_touch_data[i];
    
    // update the min and max force tracking for this end effector
    int j = endeffector.Id - m_id_offset;
    if (result < m_min_forces[j])
        m_min_forces[j] = result;
    if (result > m_max_forces[j])
        m_max_forces[j] = result;
    
    // store the result
    Blackboard->Sensors->modify(endeffector, NUSensorsData::ForceId, Blackboard->Sensors->CurrentTime, result);
}

/*! @brief Calculates whether the end effector is in contact with something
 	@param endeffector the id of the end effector
 	@param force the force in N on the end effector
 */
void EndEffectorTouch::calculateContact(const NUData::id_t& endeffector)
{
    float force;
    if (Blackboard->Sensors->getForce(endeffector, force))
    {
        int j = endeffector.Id - m_id_offset;
        float min = m_min_forces[j];
        float range = m_max_forces[j] - m_min_forces[j];
        
        bool result = false;
        if (force > min + 0.1*range)
            result = true;
        Blackboard->Sensors->modify(endeffector, NUSensorsData::ContactId, Blackboard->Sensors->CurrentTime, result);
    }
    else
        Blackboard->Sensors->modify(endeffector, NUSensorsData::ContactId, Blackboard->Sensors->CurrentTime, m_Nan);
}

/*! @brief Calculates the centre of pressure under the end effector based on the touch data
 	@param endeffector the id of the end effector 
 	@param touchdata the force data for that end effector. This should be from an anti-clockwise sensor grid
 */
void EndEffectorTouch::calculateCentreOfPressure(const NUData::id_t& endeffector)
{
    bool contact = false;
    float copx = m_Nan;
    float copy = m_Nan;
    if (Blackboard->Sensors->getContact(endeffector, contact) and contact)
    {	// centre of pressure calculation is only valid if we are in contact with an object
    	vector<vector<float> >& hull = m_hulls[endeffector.Id - m_id_offset];
    	if (hull.size() == m_touch_data.size())
        {	// centre of pressure calculation is only possible if the size of the hull specification matches that of the touch data
            float sum = 0;
            for (size_t i=0; i<m_touch_data.size(); i++)
                sum += m_touch_data[i];
            
            copx = 0;
            copy = 0;
			for (size_t i=0; i<m_touch_data.size(); i++)
            {
                copx += hull[i][0]*m_touch_data[i];
                copy += hull[i][1]*m_touch_data[i];
            }
            copx /= sum;
            copy /= sum;
        }
    }
    Blackboard->Sensors->modify(endeffector, NUSensorsData::CoPXId, Blackboard->Sensors->CurrentTime, copx);
    Blackboard->Sensors->modify(endeffector, NUSensorsData::CoPYId, Blackboard->Sensors->CurrentTime, copy);
}

/*! @brief Calculates whether the endeffector is supporting the robot. 
           Support is defined as being in contact with something, and the centre of pressure being inside the convex hull
 	@param endeffector the id of the end effector
 	@param centreofpressure the centre of pressure [x cm, y cm]
 */
void EndEffectorTouch::calculateSupport(const NUData::id_t& endeffector)
{
    bool contact = false;
	bool support = false;
    if (Blackboard->Sensors->getContact(endeffector, contact) and contact)
    {	// we can only be supporting if there is contact on the end effector
        vector<float> cop;
        if (Blackboard->Sensors->getCoP(endeffector, cop))		// if there is valid centre of pressure measurement
            if (mathGeneral::PointInsideConvexHull(cop[0], cop[1], m_hulls[endeffector.Id - m_id_offset], 0.4))		// if the cop is inside the convex hull
        		support = true;
    }
    Blackboard->Sensors->modify(endeffector, NUSensorsData::SupportId, Blackboard->Sensors->CurrentTime, support);
}

/*! @brief Invalidates all of the touch related end effector sensor values
 	@param endeffector the id of the end effector to invalidate
 */
void EndEffectorTouch::invalidate(const NUData::id_t& endeffector)
{
    Blackboard->Sensors->modify(endeffector, NUSensorsData::ForceId, Blackboard->Sensors->CurrentTime, m_Nan_all);
}
