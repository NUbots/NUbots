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
    
    vector<bool> isvalid (3, true);
    vector<float> values (3, 0);
    vector<float> gains (3, 0);
    
    JointPositions->addAction(10, isvalid, values, gains);
    isvalid[1] = false;
    JointPositions->addAction(5, isvalid, values, gains);
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

/******************************************************************************************************************************************
 Get Methods
 ******************************************************************************************************************************************/

/******************************************************************************************************************************************
 Set Methods
 ******************************************************************************************************************************************/

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
    m_body_ids.insert(m_body_ids.end(), m_larm_ids.begin(), m_larm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rarm_ids.begin(), m_rarm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_torso_ids.begin(), m_torso_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_lleg_ids.begin(), m_lleg_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rleg_ids.begin(), m_rleg_ids.end());
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


