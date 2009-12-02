/*! @file NUActionatorsData.cpp
    @brief Implementation of actionators data class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "NUActionatorsData.h"
#include "Tools/debug.h"

NUActionatorsData::joint_id_t NUActionatorsData::HeadYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::HeadPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LShoulderPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LShoulderRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LElbowYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LElbowRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RShoulderPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RShoulderRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RElbowYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RElbowRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::TorsoYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::TorsoPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipYawPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LKneePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LAnklePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LAnkleRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipYawPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RKneePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RAnklePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RAnkleRoll = NUActionatorsData::ACTIONATOR_MISSING;

NUActionatorsData::led_id_t NUActionatorsData::LEar = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::REar = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::LEye = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::REye = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::Chest = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::LFoot = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::led_id_t NUActionatorsData::RFoot = NUActionatorsData::ACTIONATOR_MISSING;

/*! @brief Default constructor for a NUActionatorsData storage class.
 */
NUActionatorsData::NUActionatorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUActionatorsData::NUActionatorsData" << endl;
#endif
    // add Joint actionators
    addActionator(&JointPositions, string("JointPositions"), actionator_t::JOINT_POSITIONS);
    addActionator(&JointVelocities, string("JointVelocities"), actionator_t::JOINT_VELOCITIES);
    addActionator(&JointTorques, string("JointTorques"), actionator_t::JOINT_TORQUES);
    
    // add Camera actionator
    addActionator(&CameraControl, string("CameraControl"), actionator_t::CAMERA_CONTROL);
    
    // add LED actionator
    addActionator(&Leds, string("Leds"), actionator_t::LEDS);
    
    // add a Sound actionator
    addActionator(&Sound, string("Sound"), actionator_t::SOUND);
}

/* Add an actionator to the NUActionatorsData.
   We construct the actionator itself and assign it to the appropriate named actionator, then append it to the list m_actionators
 */
void NUActionatorsData::addActionator(actionator_t** p_actionator, string actionatorname, actionator_t::actionator_id_t actionatorid)
{
    *p_actionator = new actionator_t(actionatorname, actionatorid);
    m_actionators.push_back(*p_actionator);
}

NUActionatorsData::~NUActionatorsData()
{
    m_actionators.clear();
    m_head_ids.clear();
    m_larm_ids.clear();
    m_rarm_ids.clear();
    m_torso_ids.clear();
    m_lleg_ids.clear();
    m_rleg_ids.clear();
}

/*! @brief Remove all of the completed actionator points
    @param currenttime all actionator points that have times before this one are assumed to have been completed, and they will be removed
 */
void NUActionatorsData::removeCompletedActions(double currenttime)
{
    for (int i=0; i<m_actionators.size(); i++)
        m_actionators[i]->removeCompleted(currenttime);
}


/******************************************************************************************************************************************
 Get Methods
 ******************************************************************************************************************************************/

/*! @brief Gets the next actionator point (ie the isvalid the positions and the gains) data
 
 @param time the variable that will be updated with the time the actionator point should be completed on in milliseconds
 @param isvalid the isvalid flags for the data and gain
 @param positions the vector of data for the actionator
 @param gains the vector of gaings for the actionator
 
 @return returns true if the actionator is available and has at least one valid data point, false otherwise
 */
bool NUActionatorsData::getJointPositions(vector<double>& time, vector<bool>& isvalid, vector<float>& positions, vector<float>& gains)
{
    getJointData(JointPositions, time, isvalid, positions, gains);
}

/*! @brief Gets the next actionator point (ie the isvalid the velocities and the gains) data
 
 @param time the variable that will be updated with the time the actionator point should be completed on in milliseconds
 @param isvalid the isvalid flags for the data and gain
 @param data the vector of data for the actionator
 @param gains the vector of gaings for the actionator
 
 @return returns true if the actionator is available and has at least one valid data point, false otherwise
 */
bool NUActionatorsData::getJointVelocities(vector<double>& time, vector<bool>& isvalid, vector<float>& velocities, vector<float>& gains)
{
    getJointData(JointVelocities, time, isvalid, velocities, gains);
}

/*! @brief Gets the next actionator point (ie the isvalid the torques and the gains) data
 
 @param time the variable that will be updated with the time the actionator point should be completed on in milliseconds
 @param isvalid the isvalid flags for the data and gain
 @param data the vector of data for the actionator
 @param gains the vector of gaings for the actionator
 
 @return returns true if the actionator is available and has at least one valid data point, false otherwise
 */
bool NUActionatorsData::getJointTorques(vector<double>& time, vector<bool>& isvalid, vector<float>& torques, vector<float>& gains)
{
    getJointData(JointTorques, time, isvalid, torques, gains);
}

/*! @brief This function does the grunt work for getting data and gains to be sent to hardware communications
 
    @param p_actionator a pointer to the actionator we put the data and gain from
    @param time the variable that will be updated with the time the actionator point should be completed on in milliseconds
    @param isvalid the isvalid flags for the data and gain
    @param data the vector of data for the actionator
    @param gains the vector of gaings for the actionator
 
    @return returns true if the actionator is available and has at least one valid data point, false otherwise
 */
bool NUActionatorsData::getJointData(actionator_t* p_actionator, vector<double>& time, vector<bool>& isvalid, vector<float>& data, vector<float>& gains)
{
    if (p_actionator->IsAvailable == false)
        return false;
    else if (p_actionator->m_points.size() == 0)
        return false;
    
    actionator_t::actionator_point_t* point = p_actionator->m_points[0];
    int numdims = point->Values.size();
    time = vector<double> (numdims, 0);
    isvalid = vector<bool> (numdims, false);
    data = vector<float> (numdims, 0);
    gains = vector<float> (numdims, 0);
    
    // for each dimension we need to find the next point that has valid data for this dimension
    for (int i=0; i<numdims; i++)
    {
        for (int j=0; j<p_actionator->m_points.size(); j++)
        {
            if (p_actionator->m_points[j]->IsValid[i] == true)
            {
                time[i] = p_actionator->m_points[j]->Time;
                isvalid[i] = true;
                data[i] = p_actionator->m_points[j]->Values[i];
                gains[i] = p_actionator->m_points[j]->Gains[i];
                break;
            }
        }
    }
}

/******************************************************************************************************************************************
 Set Methods
 ******************************************************************************************************************************************/

bool NUActionatorsData::setJointPosition(joint_id_t jointid, double time, float position)
{
    return false;
}

bool NUActionatorsData::setJointVelocity(joint_id_t jointid, double time, float velocity)
{
    return false;
}

bool NUActionatorsData::setJointStiffness(joint_id_t jointid, double time, float stiffness)
{
    return false;
}

bool NUActionatorsData::setJointTorque(joint_id_t jointid, double time, float torque)
{
    return false;
}

bool NUActionatorsData::setJointPosition(joint_id_t jointid, double time, float position, float gain)
{
    return false;
}

bool NUActionatorsData::setJointVelocity(joint_id_t jointid, double time, float velocity, float gain)
{
    return false;
}

bool NUActionatorsData::setJointTorque(joint_id_t jointid, double time, float torque, float gain)
{
    return false;
}

/*! @brief Set the joint positions for the selected body part

    @param partid the body_part_id_t being targeted
    @param time the time the position will be reached in milliseconds
    @param positions the vector of new positions
 
    @return Returns true if their are position actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointPositions(bodypart_id_t partid, double time, const vector<float>& positions)
{
    return setJointsData(JointPositions, partid, time, positions);
}

/*! @brief Set the joint velocities for the selected body part
 
    @param partid the body_part_id_t being targeted
    @param time the time the velocity will be reached in milliseconds
    @param velocities the vector of new velocities

    @return Returns true if their are velocities actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointVelocities(bodypart_id_t partid, double time, const vector<float>& velocities)
{
    return setJointsData(JointVelocities, partid, time, velocities);
}

/*! @brief Set the joint stiffnesses for the selected body part
 
    @param partid the body_part_id_t being targeted
    @param time the time the stiffness will be reached in milliseconds
    @param stiffnesses the vector of new stiffnesses

    @return Returns true if their are stiffness actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointStiffnesses(bodypart_id_t partid, double time, const vector<float>& stiffnesses)
{
    return setJointsGain(JointPositions, partid, time, stiffnesses);
}

/*! @brief Set the joint torques for the selected body part
 
 @param partid the body_part_id_t being targeted
 @param time the time the torques will be applied in milliseconds
 @param torques the vector of new torques
 
 @return Returns true if their are stiffness actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointTorques(bodypart_id_t partid, double time, const vector<float>& torques)
{
    return setJointsData(JointTorques, partid, time, torques);
}

/*! @brief Set the joint positions and gains for the selected body part
 
 @param partid the body_part_id_t being targeted
 @param time the time the position and gain will be reached in milliseconds
 @param positions the vector of new positions
 @param gains the vector of new gains
 
 @return Returns true if their are position actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& gains)
{
    return setJointsData(JointPositions, partid, time, positions, gains);
}

/*! @brief Set the joint velocities and gains for the selected body part
 
 @param partid the body_part_id_t being targeted
 @param time the time the velocities and gain will be reached in milliseconds
 @param velocities the vector of new velocities
 @param gains the vector of new gains
 
 @return Returns true if their are velocities actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointVelocities(bodypart_id_t partid, double time, const vector<float>& velocities, const vector<float>& gains)
{
    return setJointsData(JointVelocities, partid, time, velocities, gains);
}

/*! @brief Set the joint torques and gains for the selected body part
 
 @param partid the body_part_id_t being targeted
 @param time the time the torques and gain will be reached in milliseconds
 @param torques the vector of new torques
 @param gains the vector of new gains
 
 @return Returns true if their are torques actionators available, returns false otherwise
 */
bool NUActionatorsData::setJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, const vector<float>& gains)
{
    return setJointsData(JointTorques, partid, time, torques, gains);
}


bool NUActionatorsData::setJointData(actionator_t* p_actionator, joint_id_t jointid, double time, float data)
{
    //! @todo TODO: implement this function
}

bool NUActionatorsData::setJointData(actionator_t* p_actionator, joint_id_t jointid, double time, float data, float gain)
{
    //! @todo TODO: implement this function
}



/*! @brief This function does the grunt work for setting joint gains, while leaving the data alone
 
 The selected body part of the selected actionator is set with new gains. This function
 exists because it is a very similar problem to set JointPositions and JointVelocities, for example.
 
    @param p_actionator a pointer to the actionator that will have its gain updated
    @param partid the body_part_id_t being targeted
    @param time the time the action will be completed on in milliseconds
    @param gains the vector of new gains
 
    @return Returns true if the actionator is available, returns false otherwise
 */
bool NUActionatorsData::setJointsGain(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& gains)
{
    if (p_actionator->IsAvailable == false)
        return false;
    if (partid == All && m_num_joints == 0)
        return false;
    else if (partid == Body && m_num_body_joints == 0)
        return false;
    else if (partid == Head && m_num_head_joints == 0)
        return false;
    else if ((partid == LeftArm || partid == RightArm) && m_num_arm_joints == 0)
        return false;
    else if (partid == Torso && m_num_torso_joints == 0)
        return false;
    else if ((partid == LeftLeg || partid == RightLeg) && m_num_leg_joints == 0)
        return false;
    
    const vector<bool> isdatavalid (m_num_joints, false);         // for this function the data is always unused
    const vector<float> alldata (m_num_joints, 0);                // so make them invalid and set to zero
    
    if (partid == All)
    {   // if the data and gains are full, then we can do this very quickly
        vector<bool> isgainvalid (m_num_joints, true);
        addAction(p_actionator, time, isdatavalid, alldata, isgainvalid, gains);
    }
    else
    {
        if (partid == Body)
        {   // if we have all the body data and gains, it is faster to just set invalid values for the head
            vector<bool> isgainvalid (m_num_joints, true);          // set the head isvalid flags to false
            for (int i=0; i<m_head_ids.size(); i++)
                isgainvalid[m_head_ids[i]] = false;         
            
            vector<float> allgains (gains);                           // now I make alldata and allgains by inserting zeros at the begining of each for the head joints
            allgains.insert(allgains.begin(), m_head_ids.size(), 0); 
            
            addAction(p_actionator, time, isdatavalid, alldata, isgainvalid, allgains);
        }
        else
        {
            vector<bool> isgainvalid (m_num_joints, false);
            vector<float> allgains (m_num_joints, 0);
            
            if (partid == Head)
                expandGain(m_head_ids, gains, isgainvalid, allgains);
            else if (partid == LeftArm)
                expandGain(m_larm_ids, gains, isgainvalid, allgains);
            else if (partid == RightArm)
                expandGain(m_rarm_ids, gains, isgainvalid, allgains);
            else if (partid == Torso)
                expandGain(m_torso_ids, gains, isgainvalid, allgains);
            else if (partid == LeftLeg)
                expandGain(m_lleg_ids, gains, isgainvalid, allgains);
            else if (partid == RightLeg)
                expandGain(m_rleg_ids, gains, isgainvalid, allgains);
            
            addAction(p_actionator, time, isdatavalid, alldata, isgainvalid, allgains);
        }
    }
    return true;
}

/*! @brief This function does the grunt work for setting joint data, while leaving the gains alone
 
 The selected body part of the selected actionator is set with new data. This function
 exists because it is a very similar problem to set JointPositions and JointVelocities, for example.
 
    @param p_actionator a pointer to the actionator that will have its data updated
    @param partid the body_part_id_t being targeted
    @param time the time the action will be completed on in milliseconds
    @param data the vector of new data
 
    @return Returns true if the actionator is available, returns false otherwise
 */
bool NUActionatorsData::setJointsData(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& data)
{
    if (p_actionator->IsAvailable == false)
        return false;
    if (partid == All && m_num_joints == 0)
        return false;
    else if (partid == Body && m_num_body_joints == 0)
        return false;
    else if (partid == Head && m_num_head_joints == 0)
        return false;
    else if ((partid == LeftArm || partid == RightArm) && m_num_arm_joints == 0)
        return false;
    else if (partid == Torso && m_num_torso_joints == 0)
        return false;
    else if ((partid == LeftLeg || partid == RightLeg) && m_num_leg_joints == 0)
        return false;
    
    const vector<bool> isgainvalid (m_num_joints, false);         // for this function the gains are always unused
    const vector<float> allgains (m_num_joints, 0);               // so make them invalid and set to zero
    
    if (partid == All)
    {   // if the data and gains are full, then we can do this very quickly
        const vector<bool> isdatavalid (m_num_joints, true);
        addAction(p_actionator, time, isdatavalid, data, isgainvalid, allgains);
    }
    else
    {
        if (partid == Body)
        {   // if we have all the body data and gains, it is faster to just set invalid values for the head
            vector<bool> isdatavalid (m_num_joints, true);          // set the head isvalid flags to false
            for (int i=0; i<m_head_ids.size(); i++)
                isdatavalid[m_head_ids[i]] = false;         
            
            vector<float> alldata (data);                           // now I make alldata and allgains by inserting zeros at the begining of each for the head joints
            alldata.insert(alldata.begin(), m_head_ids.size(), 0); 
            
            addAction(p_actionator, time, isdatavalid, alldata, isgainvalid, allgains);
        }
        else
        {
            vector<bool> isdatavalid (m_num_joints, false);
            vector<float> alldata (m_num_joints, 0);
            
            if (partid == Head)
                expandData(m_head_ids, data, isdatavalid, alldata);
            else if (partid == LeftArm)
                expandData(m_larm_ids, data, isdatavalid, alldata);
            else if (partid == RightArm)
                expandData(m_rarm_ids, data, isdatavalid, alldata);
            else if (partid == Torso)
                expandData(m_torso_ids, data, isdatavalid, alldata);
            else if (partid == LeftLeg)
                expandData(m_lleg_ids, data, isdatavalid, alldata);
            else if (partid == RightLeg)
                expandData(m_rleg_ids, data, isdatavalid, alldata);
            
            addAction(p_actionator, time, isdatavalid, alldata, isgainvalid, allgains);
        }
    }
    return true;
}

/*! @brief This function does the grunt work for setting joint data and gains simultaneously
 
    The selected body part of the selected actionator is set with new data and gains. This function
    exists because it is a very similar problem to set JointPositions and JointVelocities, for example.
 
    @param p_actionator a pointer to the actionator that will have its data and gain updated
    @param partid the body_part_id_t being targeted
    @param time the time the action will be completed on in milliseconds
    @param data the vector of new data
    @param gains the vector of new gains
 
    @return Returns true if the actionator is available, returns false otherwise
 */
bool NUActionatorsData::setJointsData(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& data, const vector<float>& gains)
{
    if (p_actionator->IsAvailable == false)
        return false;
    if (partid == All && m_num_joints == 0)
        return false;
    else if (partid == Body && m_num_body_joints == 0)
        return false;
    else if (partid == Head && m_num_head_joints == 0)
        return false;
    else if ((partid == LeftArm || partid == RightArm) && m_num_arm_joints == 0)
        return false;
    else if (partid == Torso && m_num_torso_joints == 0)
        return false;
    else if ((partid == LeftLeg || partid == RightLeg) && m_num_leg_joints == 0)
        return false;
    
    if (partid == All)
    {   // if the positions and gains are full, then we can do this very quickly
        vector<bool> isvalid (m_num_joints, true);
        addAction(p_actionator, time, isvalid, data, gains);
    }
    else
    {
        if (partid == Body)
        {   // if we have all the body data and gains, it is faster to just set invalid values for the head
            vector<bool> isvalid (m_num_joints, true);          // set the head isvalid flags to false
            for (int i=0; i<m_head_ids.size(); i++)
                isvalid[m_head_ids[i]] = false;         
            
            vector<float> alldata (data);                       // now I make alldata and allgains by inserting zeros at the begining of each for the head joints
            vector<float> allgains (gains);
            alldata.insert(alldata.begin(), m_head_ids.size(), 0);
            allgains.insert(allgains.begin(), m_head_ids.size(), 0);     
            
            addAction(p_actionator, time, isvalid, alldata, allgains);
        }
        else
        {
            vector<bool> isvalid (m_num_joints, false);
            vector<float> alldata (m_num_joints, 0);
            vector<float> allgains (m_num_joints, 0);
            
            if (partid == Head)
                expandAction(m_head_ids, data, gains, isvalid, alldata, allgains);
            else if (partid == LeftArm)
                expandAction(m_larm_ids, data, gains, isvalid, alldata, allgains);
            else if (partid == RightArm)
                expandAction(m_rarm_ids, data, gains, isvalid, alldata, allgains);
            else if (partid == Torso)
                expandAction(m_torso_ids, data, gains, isvalid, alldata, allgains);
            else if (partid == LeftLeg)
                expandAction(m_lleg_ids, data, gains, isvalid, alldata, allgains);
            else if (partid == RightLeg)
                expandAction(m_rleg_ids, data, gains, isvalid, alldata, allgains);
            
            addAction(p_actionator, time, isvalid, alldata, allgains);
        }
    }
    return true;
}

/*! @brief Expands the vector gains into isgainvalid and allgains vectors where isgainvalid and allgains have sizes
           equal to m_all_joint_ids.
    @param ids a vector of the ids of the section being targeted by gains
    @param gains a vector of data that will be sent to a subactionator group
    @param isgainvalid will be updated to contain true values in places specified by ids
    @param allgains will be update to contain actual values in places specified by ids
 */
void NUActionatorsData::expandGain(const vector<joint_id_t>& ids, const vector<float>& gains, vector<bool>& isgainvalid, vector<float>& allgains)
{
    if (ids.size() != gains.size())
    {
        debug << "NUActionatorsData::expandGain: Your ids and gains have different lengths. All of your gains will be ignored!" << endl;
        return;
    }
    
    int id = 0;
    for (int i=0; i<ids.size(); i++)
    {
        id = ids[i];        // id is the index into the all arrays, and i is the index into the data and gains arrays
        
        isgainvalid[id] = true;
        allgains[id] = gains[i];
    }
}

/*! @brief Expands the vector data into isdatavalid and alldata vectors where isdatavalid and alldata have sizes
           equal to m_all_joint_ids.
    @param ids a vector of the ids of the section being targeted by data
    @param data a vector of data that will be sent to a subactionator group
    @param isdatavalid will be updated to contain true values in places specified by ids
    @param alldata will be update to contain actual values in places specified by ids
 */
void NUActionatorsData::expandData(const vector<joint_id_t>& ids, const vector<float>& data, vector<bool>& isdatavalid, vector<float>& alldata)
{
    if (ids.size() != data.size())
    {
        debug << "NUActionatorsData::expandData: Your ids and data have different lengths. All of your data will be ignored!" << endl;
        return;
    }
    
    int id = 0;
    for (int i=0; i<ids.size(); i++)
    {
        id = ids[i];        // id is the index into the all arrays, and i is the index into the data and gains arrays
        
        isdatavalid[id] = true;
        alldata[id] = data[i];
    }
}

/*! @brief Expand the vectors data and gains into isvalid, alldata and allgains such that
           isvalid, alldata and allgains have length equal to m_num_joints.
    @param ids a vector of the ids of the section being targeted by data and gains
    @param data a vector of data that will be sent to a subactionator group
    @param gains a vector of gain that will be sent to a subactionator group
    @param isvalid will be updated to contain true values in places specified by ids
    @param alldata will be update to contain actual values in places specified by ids
    @param allgains will be update to contain actual values in places specified by ids
 */
void NUActionatorsData::expandAction(const vector<joint_id_t>& ids, const vector<float>& data, const vector<float>& gains, vector<bool>& isvalid, vector<float>& alldata, vector<float>& allgains)
{
    if (ids.size() != data.size() || ids.size() != gains.size())
    {
        debug << "NUActionatorsData::expandAction: Your ids, data and gains have different lengths. All of your data and gains will be ignored!" << endl;
        return;
    }
    
    int id = 0;
    for (int i=0; i<ids.size(); i++)
    {
        id = ids[i];        // id is the index into the all arrays, and i is the index into the data and gains arrays
        
        isvalid[id] = true;
        alldata[id] = data[i];
        allgains[id] = gains[i];
    }
}

/*! @brief Adds an action to the specified actionator where isvalid applies to both the data and the gains
 */
void NUActionatorsData::addAction(actionator_t* p_actionator, double time, const vector<bool>& isvalid, const vector<float>& alldata, const vector<float>& allgains)
{
    addAction(p_actionator, time, isvalid, alldata, isvalid, allgains);
}

/*! @brief Adds an action to the specified actionator where separate isvalid flags are used for the data and the gains
 */
void NUActionatorsData::addAction(actionator_t* p_actionator, double time, const vector<bool>& isdatavalid, const vector<float>& alldata, const vector<bool>& isgainvalid, const vector<float>& allgains)
{
    p_actionator->addAction(time, isdatavalid, alldata, isgainvalid, allgains);
}


/*! @brief Sets each of the static joint_id_t if the joint is in the list. Also sets id lists for accessing limbs. 
 @param joints a vector of strings where each string is a name of a joint
 */
void NUActionatorsData::setAvailableJoints(const vector<string>& joints)
{
    // NOTE: This has been copied directly from NUSensorsData; so if your changing this you probably need to change that as well!
    
    // first convert everything to lower case and remove whitespace and underscores
    vector<string> simplejointnames;
    string namebuffer, currentname, currentletter;
    for (int i=0; i<joints.size(); i++)
    {
        currentname = joints[i];
        // compare each letter to a space and an underscore
        for (int j=0; j<currentname.size(); j++)
        {
            currentletter = currentname.substr(j, 1);
            if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0)     // if it is neither then add the lower case version
                namebuffer += tolower(currentletter[0]);            
        }
        simplejointnames.push_back(namebuffer);
        namebuffer.clear();
    }
    
    for (int i=0; i<simplejointnames.size(); i++) 
    {
        if (simplejointnames[i].compare("headyaw") == 0)
        {
            HeadYaw = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("headpitch") == 0)
        {
            HeadPitch = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lshoulderpitch") == 0)
        {
            LShoulderPitch = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lshoulderroll") == 0)
        {
            LShoulderRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lelbowyaw") == 0)
        {
            LElbowYaw = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lelbowroll") == 0)
        {
            LElbowRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rshoulderpitch") == 0)
        {
            RShoulderPitch = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rshoulderroll") == 0)
        {
            RShoulderRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("relbowyaw") == 0)
        {
            RElbowYaw = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("relbowroll") == 0)
        {
            RElbowRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lhipyaw") == 0)
        {
            LHipYaw = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lhipyawpitch") == 0)
        {
            LHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lhippitch") == 0)
        {
            LHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lhiproll") == 0)
        {
            LHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lkneepitch") == 0)
        {
            LKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lanklepitch") == 0)
        {
            LAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("lankleroll") == 0)
        {
            LAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rhipyaw") == 0)
        {
            RHipYaw = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rhipyawpitch") == 0)
        {
            RHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rhippitch") == 0)
        {
            RHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rhiproll") == 0)
        {
            RHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rkneepitch") == 0)
        {
            RKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("ranklepitch") == 0)
        {
            RAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].compare("rankleroll") == 0)
        {
            RAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
    }
    // add the arms, torso and legs to the body_ids
    m_body_ids.insert(m_body_ids.end(), m_larm_ids.begin(), m_larm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rarm_ids.begin(), m_rarm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_torso_ids.begin(), m_torso_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_lleg_ids.begin(), m_lleg_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rleg_ids.begin(), m_rleg_ids.end());
    // add the head and the body_ids to the all_joint_ids
    m_all_joint_ids.insert(m_all_joint_ids.end(), m_head_ids.begin(), m_head_ids.end());
    m_all_joint_ids.insert(m_all_joint_ids.end(), m_body_ids.begin(), m_body_ids.end());
    
    // set total numbers in each limb
    m_num_head_joints = m_head_ids.size();
    m_num_arm_joints = m_larm_ids.size();
    m_num_torso_joints = m_torso_ids.size();
    m_num_leg_joints = m_lleg_ids.size();
    m_num_body_joints = m_body_ids.size();
    m_num_joints = m_all_joint_ids.size();
}

/*! @brief Sets each of the static led_id_t if the led is in the list.
    @param leds a vector of strings where each string is a name of a led
 */
void NUActionatorsData::setAvailableLeds(const vector<string>& leds)
{
    // first convert everything to lower case and remove whitespace and underscores
    vector<string> simplelednames;
    string namebuffer, currentname, currentletter;
    for (int i=0; i<leds.size(); i++)
    {
        currentname = leds[i];
        // compare each letter to a space and an underscore
        for (int j=0; j<currentname.size(); j++)
        {
            currentletter = currentname.substr(j, 1);
            if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0)     // if it is neither then add the lower case version
                namebuffer += tolower(currentletter[0]);            
        }
        simplelednames.push_back(namebuffer);
        namebuffer.clear();
    }
    
    for (int i=0; i<simplelednames.size(); i++) 
    {
        if (simplelednames[i].compare("lear") == 0)
            LEar = i;
        else if (simplelednames[i].compare("rear") == 0)
            REar = i;
        else if (simplelednames[i].compare("leye") == 0)
            LEye = i;
        else if (simplelednames[i].compare("reye") == 0)
            REye = i;
        else if (simplelednames[i].compare("chest") == 0)
            Chest = i;
        else if (simplelednames[i].compare("lfoot") == 0)
            LFoot = i;
        else if (simplelednames[i].compare("rfoot") == 0)
            RFoot = i;
    }
}

/*! @brief Sets the available actionators based on the names found in the passed in strings
    
    @param actionators a vector of names for each of the available actionators.
 */
void NUActionatorsData::setAvailableActionators(const vector<string>& actionators)
{
    // first convert everything to lower case and remove whitespace and underscores
    vector<string> simpleactionatornames;
    string namebuffer, currentname, currentletter;
    for (int i=0; i<actionators.size(); i++)
    {
        currentname = actionators[i];
        // compare each letter to a space and an underscore
        for (int j=0; j<currentname.size(); j++)
        {
            currentletter = currentname.substr(j, 1);
            if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0)     // if it is neither then add the lower case version
                namebuffer += tolower(currentletter[0]);            
        }
        simpleactionatornames.push_back(namebuffer);
        namebuffer.clear();
    }
    
    for (int i=0; i<simpleactionatornames.size(); i++) 
    {
        if (simpleactionatornames[i].compare("jointpositions") == 0)
            JointPositions->IsAvailable = true;
        else if (simpleactionatornames[i].compare("jointvelocities") == 0)
            JointVelocities->IsAvailable = true;
        else if (simpleactionatornames[i].compare("jointtorques") == 0)
            JointTorques->IsAvailable = true;
        else if (simpleactionatornames[i].compare("cameracontrol") == 0 || simpleactionatornames[i].compare("camera") == 0 || simpleactionatornames[i].compare("cameraselect") == 0)
            CameraControl->IsAvailable = true;
        else if (simpleactionatornames[i].compare("leds") == 0)
            Leds->IsAvailable = true;
        else if (simpleactionatornames[i].compare("sound") == 0)
            Sound->IsAvailable = true;
    }
}

/******************************************************************************************************************************************
 Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

void NUActionatorsData::summaryTo(ostream& output)
{
    for (int i=0; i<m_actionators.size(); i++)
        m_actionators[i]->summaryTo(output);
}

void NUActionatorsData::csvTo(ostream& output)
{
    //! @todo TODO: implement this function    
}

ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor)
{
    //! @todo TODO: implement this function
}

istream& operator>> (istream& input, NUActionatorsData& p_sensor)
{
    //! @todo TODO: implement this function
}


