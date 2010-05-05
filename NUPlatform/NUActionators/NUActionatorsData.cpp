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
#include "actionator_t.h"
#include "debug.h"
#include "debugverbositynuactionators.h"

NUActionatorsData::joint_id_t NUActionatorsData::HeadPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::HeadYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LShoulderRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LShoulderPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LElbowRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LElbowYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RShoulderRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RShoulderPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RElbowRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RElbowYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::TorsoRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::TorsoPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::TorsoYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipYawPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LHipYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LKneePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LAnkleRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::LAnklePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipYawPitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RHipYaw = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RKneePitch = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RAnkleRoll = NUActionatorsData::ACTIONATOR_MISSING;
NUActionatorsData::joint_id_t NUActionatorsData::RAnklePitch = NUActionatorsData::ACTIONATOR_MISSING;

/*! @brief Default constructor for a NUActionatorsData storage class.
 */
NUActionatorsData::NUActionatorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUActionatorsData::NUActionatorsData" << endl;
#endif
    Sound = NULL;
    Teleporter = NULL;
    m_positionactionation = false;
    m_torqueactionation = false;
    
    m_num_head_joints = 0;
    m_num_arm_joints = 0;
    m_num_torso_joints = 0;
    m_num_leg_joints = 0;
    m_num_body_joints = 0;
    m_num_joints = 0;
}

/*! @brief Destroys the NUActionatorsData storage class
 */
NUActionatorsData::~NUActionatorsData()
{
    for (size_t i=0; i<m_all_actionators.size(); i++)           // Note this will delete PositionActionators, TorqueActionators, LedActionators and Teleporter
        delete m_all_actionators[i];
    
    for (size_t i=0; i<m_all_string_actionators.size(); i++)    // Note this will delete Sound
        delete m_all_string_actionators[i];
}

/******************************************************************************************************************************************
 Initialisation and Availability Setting Methods
 ******************************************************************************************************************************************/

/*! @brief Sets the available joint control methods, that is whether the joints can be position, torque or both controlled.
 @param methodnames a vector of strings where each string names a method
 */
void NUActionatorsData::setAvailableJointControlMethods(const vector<string>& methodnames)
{
    vector<string> simplemethodnames;
    simplifyNames(methodnames, simplemethodnames);
    
    for (unsigned int i=0; i<simplemethodnames.size(); i++)
    {
        if (simplemethodnames[i].find("position") != string::npos)
            m_positionactionation = true;
        else if (simplemethodnames[i].find("torque") != string::npos)
            m_torqueactionation = true;
        else
            debug << "NUActionatorsData::setAvailableJointControlMethods. You have specified an unrecognised joint control method: " << methodnames[i] << endl;
    }
}


/*! @brief Adds the joint actionators and sets each of the static joint_id_t if the joint is in the list. Also sets id lists for accessing limbs. 
 @param jointnames a vector of strings where each string is a name of a joint
 */
void NUActionatorsData::setAvailableJoints(const vector<string>& jointnames)
{
    // NOTE: This has been copied directly from NUSensorsData; so if your changing this you probably need to change that as well!
    vector<string> simplejointnames;
    simplifyNames(jointnames, simplejointnames);
    
    for (unsigned int i=0; i<simplejointnames.size(); i++) 
    {
        addJointActionator(simplejointnames[i]);
        if (simplejointnames[i].find("headyaw") != string::npos)
        {
            HeadYaw = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].find("headpitch") != string::npos)
        {
            HeadPitch = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lshoulderpitch") != string::npos)
        {
            LShoulderPitch = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lshoulderroll") != string::npos)
        {
            LShoulderRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lelbowyaw") != string::npos)
        {
            LElbowYaw = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lelbowroll") != string::npos)
        {
            LElbowRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rshoulderpitch") != string::npos)
        {
            RShoulderPitch = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rshoulderroll") != string::npos)
        {
            RShoulderRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("relbowyaw") != string::npos)
        {
            RElbowYaw = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("relbowroll") != string::npos)
        {
            RElbowRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("torsoyaw")!= string::npos)
        {
            TorsoYaw = i;
            m_torso_ids.push_back(i);
        }
        else if (simplejointnames[i].find("torsopitch") != string::npos)
        {
            TorsoPitch = i;
            m_torso_ids.push_back(i);
        }
        else if (simplejointnames[i].find("torsoroll") != string::npos)
        {
            TorsoRoll = i;
            m_torso_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhipyawpitch") != string::npos)
        {
            LHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhipyaw") != string::npos)
        {
            LHipYaw = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhippitch") != string::npos)
        {
            LHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhiproll") != string::npos)
        {
            LHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lkneepitch") != string::npos)
        {
            LKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lanklepitch") != string::npos)
        {
            LAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lankleroll") != string::npos)
        {
            LAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhipyawpitch") != string::npos)
        {
            RHipYawPitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhipyaw") != string::npos)
        {
            RHipYaw = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhippitch") != string::npos)
        {
            RHipPitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhiproll") != string::npos)
        {
            RHipRoll = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rkneepitch") != string::npos)
        {
            RKneePitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("ranklepitch") != string::npos)
        {
            RAnklePitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rankleroll") != string::npos)
        {
            RAnkleRoll = i;
            m_rleg_ids.push_back(i);
        }
        else 
        {
            debug << "NUActionatorsData::setAvailableJoints. This platform has an unrecognised joint: " << jointnames[i] << endl;
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

/*! @brief Adds the led actionators. Maintains the m_*_ids vector for led group access
    @param leds a vector of strings where each string is a name of a led
 */
void NUActionatorsData::setAvailableLeds(const vector<string>& lednames)
{
    vector<string> simplelednames;
    simplifyNames(lednames, simplelednames);
    vector<string> colourlesslednames;
    removeColours(simplelednames, colourlesslednames);
    
    for (unsigned int i=0; i<colourlesslednames.size(); i++) 
    {
        addLedActionator(colourlesslednames[i]);
        if (colourlesslednames[i].find("lear") != string::npos || colourlesslednames[i].find("earsledleft") != string::npos)
            m_lear_ids.push_back(i);
        else if (colourlesslednames[i].find("rear") != string::npos || colourlesslednames[i].find("earsledright") != string::npos)
            m_rear_ids.push_back(i);
        else if (colourlesslednames[i].find("leye") != string::npos || colourlesslednames[i].find("faceledleft") != string::npos  || colourlesslednames[i].find("faceledredleft") != string::npos)
            m_leye_ids.push_back(i);
        else if (colourlesslednames[i].find("reye") != string::npos || colourlesslednames[i].find("faceledright") != string::npos || colourlesslednames[i].find("faceledredright") != string::npos)
            m_reye_ids.push_back(i);
        else if (colourlesslednames[i].find("chest") != string::npos || colourlesslednames[i].find("chestboardled") != string::npos || colourlesslednames[i].find("chestboardredled") != string::npos)
            m_chest_ids.push_back(i);
        else if (colourlesslednames[i].find("lfoot") != string::npos || colourlesslednames[i].find("lfootled") != string::npos || colourlesslednames[i].find("lfootredled") != string::npos)
            m_lfoot_ids.push_back(i);
        else if (colourlesslednames[i].find("rfoot") != string::npos || colourlesslednames[i].find("rfootled") != string::npos || colourlesslednames[i].find("rfootredled") != string::npos)
            m_rfoot_ids.push_back(i);
    }
    
    m_all_led_ids.insert(m_all_led_ids.end(), m_lear_ids.begin(), m_lear_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_rear_ids.begin(), m_rear_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_leye_ids.begin(), m_leye_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_reye_ids.begin(), m_reye_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_chest_ids.begin(), m_chest_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_lfoot_ids.begin(), m_lfoot_ids.end());
    m_all_led_ids.insert(m_all_led_ids.end(), m_rfoot_ids.begin(), m_rfoot_ids.end());
}

/*! @brief Adds an actionator with the specified name and type
 
 @param p_actionator the actionator pointer for the new actionator
 @param actionatorname the name of the actionator to be added
 @param actionatortype the type of the actionator to be added
 */
template <> void NUActionatorsData::addActionator(actionator_t<string>*& p_actionator, string actionatorname, actionator_t<string>::actionator_type_t actionatortype)
{
    p_actionator = new actionator_t<string>(actionatorname, actionatortype);
    m_all_string_actionators.push_back(p_actionator);
}

/*! @brief Adds an actionator to the actionator group with the specified name and type
 
 @param actionatorgroup the vector of similar actionator to which this one will be added
 @param actionatorname the name of the actionator to be added
 @param actionatortype the type of the actionator to be added
 */
template <typename T> void NUActionatorsData::addActionator(vector<actionator_t<T>*>& actionatorgroup, string actionatorname, typename actionator_t<T>::actionator_type_t actionatortype)
{
    actionator_t<T>* newactionator = new actionator_t<T>(actionatorname, actionatortype);
    actionatorgroup.push_back(newactionator);
    m_all_actionators.push_back(newactionator);
}

/*! @brief Adds an actionator with the specified name and type
 
 @param p_actionator the actionator pointer for the new actionator
 @param actionatorname the name of the actionator to be added
 @param actionatortype the type of the actionator to be added
 */
template <typename T> void NUActionatorsData::addActionator(actionator_t<T>*& p_actionator, string actionatorname, typename actionator_t<T>::actionator_type_t actionatortype)
{
    p_actionator = new actionator_t<T>(actionatorname, actionatortype);
    m_all_actionators.push_back(p_actionator);
}

/*! @brief Sets the available actionators based on the names found in the passed in strings
 
 @param actionators a vector of names for each of the available actionators.
 */
void NUActionatorsData::setAvailableOtherActionators(const vector<string>& actionatornames)
{
    vector<string> simpleactionatornames;
    simplifyNames(actionatornames, simpleactionatornames);
    
    for (unsigned int i=0; i<simpleactionatornames.size(); i++) 
    {
        if (simpleactionatornames[i].find("sound") != string::npos)
            addActionator(Sound, actionatornames[i], actionator_t<string>::SOUND);
        else if (simpleactionatornames[i].find("teleporter") != string::npos || simpleactionatornames[i].find("teleportation") != string::npos)
            addActionator(Teleporter, actionatornames[i], actionator_t<float>::TELEPORTER);
        else
            debug << "NUActionatorsData::setAvailableOtherActionators. You have added an unrecognised other actionator: " << actionatornames[i] << endl;
    }
}

/*! @brief Adds a joint actionator with the specified name
 
 @param actionatorname the name of the actionator to be added
 */
void NUActionatorsData::addJointActionator(string actionatorname)
{
    if (m_positionactionation == true)
        addActionator(PositionActionators, actionatorname, actionator_t<>::JOINT_POSITION);
    if (m_torqueactionation == true)
        addActionator(TorqueActionators, actionatorname, actionator_t<>::JOINT_TORQUE);
}

/*! @brief Adds a led actionator with the specified name
 
 @param actionatorname the name of the actionator to be added
 */
void NUActionatorsData::addLedActionator(string actionatorname)
{
    addActionator(LedActionators, actionatorname, actionator_t<>::LEDS);
}

/*! @brief Simplifies a name
 
    The name is converted to lowercase.
    Spaces, underscores, forward slash, backward slash and dots are removed from the name.
 
    @param input the name to be simplified
    @return the simplified string
 */
string NUActionatorsData::simplifyName(const string& input)
{
    string namebuffer, currentletter;
    // compare each letter to a space and an underscore and a forward slash
    for (unsigned int j=0; j<input.size(); j++)
    {
        currentletter = input.substr(j, 1);
        if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0 && currentletter.compare(string("/")) != 0 && currentletter.compare(string("\\")) != 0 && currentletter.compare(string(".")) != 0)
            namebuffer += tolower(currentletter[0]);            
    }
    return namebuffer;
}

/*! @brief Simplifies a vector of strings
 
 @param input the vector of strings to be simplified
 @param ouput the vector that will be updated to contain the simplified names
 */
void NUActionatorsData::simplifyNames(const vector<string>& input, vector<string>& output)
{
    vector<string> simplifiednames;
    for (unsigned int i=0; i<input.size(); i++)
        simplifiednames.push_back(simplifyName(input[i]));
    output = simplifiednames;
}

/*! @brief Removes colours from a string. Also trys to only add a single actionator for LEDs at the same location but with different colours.
    @param input the base string to pick out names which don't have colours, or add a single version without the colour
    @param output the colourless actionatornames
*/
void NUActionatorsData::removeColours(const vector<string>& input, vector<string>& output)
{
    vector<string> outputnames;
    
    string previousname;
    for (unsigned int i=0; i<input.size(); i++)
    {
        string temp;
        unsigned int pos = 0;
        pos = input[i].find("red");
        if (pos != string::npos)
            temp = input[i].substr(0, pos) + input[i].substr(pos+3, string::npos);
            
        pos = input[i].find("green");
        if (pos != string::npos)
            temp = input[i].substr(0, pos) + input[i].substr(pos+5, string::npos);
        
        pos = input[i].find("blue");
        if (pos != string::npos)
            temp = input[i].substr(0, pos) + input[i].substr(pos+4, string::npos);
            
        if (temp.compare(previousname) != 0)        // if the colourless part matches does not match previous then add it
            outputnames.push_back(temp);
        
        if (temp.size() == 0)                       // if the name contains no colour then add it anyway
            outputnames.push_back(input[i]);

        previousname = temp;
    }
    output = outputnames;
}

/******************************************************************************************************************************************
 Get Methods
 ******************************************************************************************************************************************/
/*! @brief Pre processes the data to be ready for copying to hardware communication
 */
void NUActionatorsData::preProcess()
{
    for (unsigned int i=0; i<m_all_actionators.size(); i++)
        m_all_actionators[i]->preProcess();
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
        m_all_string_actionators[i]->preProcess();
}

/*! @brief Post processes the data after sending it to the hardware communications (Remove all of the completed actionator points)
    @param currenttime all actionator points that have times before this one are assumed to have been completed, and they will be removed
 */
void NUActionatorsData::postProcess(double currenttime)
{
    for (unsigned int i=0; i<m_all_actionators.size(); i++)
        m_all_actionators[i]->removeCompletedPoints(currenttime);
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
        m_all_string_actionators[i]->removeCompletedPoints(currenttime);
}

/*! @brief Returns the number of joints in the specified body part
    @param partid the id of the body part
    @return the number of joints
 */
int NUActionatorsData::getNumberOfJoints(bodypart_id_t partid)
{
    if (partid == AllJoints)
        return m_num_joints;
    else if (partid == BodyJoints)
        return m_num_body_joints;
    else if (partid == HeadJoints)
        return m_num_head_joints;
    else if (partid == LeftArmJoints)
        return m_num_arm_joints;
    else if (partid == RightArmJoints)
        return m_num_arm_joints;
    else if (partid == TorsoJoints)
        return m_num_torso_joints;
    else if (partid == LeftLegJoints)
        return m_num_leg_joints;
    else if (partid == RightLegJoints)
        return m_num_leg_joints;
    else
    {
        debug << "NUActionatorsData::getNumberOfJoints. UNDEFINED Body part.";
        return 0;
    }
}

/*! @brief Returns the number of joints in the specified led group
    @param groupid the id of the led group
    @return the number of lights in the group
 */
int NUActionatorsData::getNumberOfLeds(ledgroup_id_t groupid)
{
    if (groupid == AllLeds)
        return m_all_led_ids.size();
    else if (groupid == LeftEarLeds)
        return m_lear_ids.size();
    else if (groupid == RightEarLeds)
        return m_rear_ids.size();
    else if (groupid == LeftEyeLeds)
        return m_leye_ids.size();
    else if (groupid == RightEyeLeds)
         return m_reye_ids.size();
    else if (groupid == ChestLeds)
        return m_chest_ids.size();
    else if (groupid == LeftFootLeds)
        return m_lfoot_ids.size();
    else if (groupid == RightFootLeds)
        return m_rfoot_ids.size();
    else
    {
        debug << "NUActionatorsData::getNumberOfLeds. UNDEFINED led group.";
        return 0;
    }
}

/*! @brief Gets the next joint position for the specified joint
    @param id the id of the joint you want the next command
    @param time will be updated with the next command's time
    @param position will be updated with the next command's position
    @param velocity will be updated with the next command's gain
 */
bool NUActionatorsData::getNextJointPosition(joint_id_t id, double& time, float& position, float& velocity, float& gain)
{
    if (id == NUActionatorsData::ACTIONATOR_MISSING || PositionActionators[id]->isEmpty() || PositionActionators[id]->m_points[0].Data.size() != 3)
        return false;
    else 
    {
        time = PositionActionators[id]->m_points[0].Time;
        position = PositionActionators[id]->m_points[0].Data[0];
        velocity = PositionActionators[id]->m_points[0].Data[1];
        gain = PositionActionators[id]->m_points[0].Data[2]; 
        return true;
    }
}

/*! @brief Gets the *last* joint position for the specified joint
    @param id the id of the joint you want the *last* command
    @param time will be updated with the *last* command's time
    @param position will be updated with the *last* command's position
    @param velocity will be updated with the *last* command's gain
 */
bool NUActionatorsData::getLastJointPosition(joint_id_t id, double& time, float& position, float& velocity, float& gain)
{
    if (id == NUActionatorsData::ACTIONATOR_MISSING)
        return false;
    else
    {
        int lastindex = PositionActionators[id]->m_points.size() - 1;
        if (lastindex >= 0 && PositionActionators[id]->m_points[lastindex].Data.size() == 3)
        {
            time = PositionActionators[id]->m_points[lastindex].Time;
            position = PositionActionators[id]->m_points[lastindex].Data[0];
            velocity = PositionActionators[id]->m_points[lastindex].Data[1];
            gain = PositionActionators[id]->m_points[lastindex].Data[2]; 
            return true;
        }
        else
            return false;
    }
}

/*! @brief Gets the next position control point
    
    @param isvalid a vector of bools that indicates whether there is a new target for each joint.
    @param time the time each position control should be completed will be put in this vector
    @param positions the target position for each joint will be put in this vector
    @param velocities the target velocities for each joint will be put in this vector
    @param gains the target gains for each joint will be put in this vector
 */
bool NUActionatorsData::getNextJointPositions(vector<bool>& isvalid, vector<double>& time, vector<float>& positions, vector<float>& velocities, vector<float>& gains)
{
    static int l_num_joints = PositionActionators.size();
    static vector<bool> l_isvalid(l_num_joints, false);
    static vector<double> l_time(l_num_joints, 0);
    static vector<float> l_positions(l_num_joints, 0);
    static vector<float> l_velocities(l_num_joints, 0);
    static vector<float> l_gains(l_num_joints, 0);
    
    // loop through each actionator in PostionActionators looking for non-empty actionators with the right datalength
    for (int i=0; i<l_num_joints; i++)
    {
        if(PositionActionators[i]->isEmpty() || PositionActionators[i]->m_points[0].Data.size() != 3)
        {
            l_isvalid[i] = false;
        }
        else
        {
            l_isvalid[i] = true;
            l_time[i] = PositionActionators[i]->m_points[0].Time;
            l_positions[i] = PositionActionators[i]->m_points[0].Data[0];
            l_velocities[i] = PositionActionators[i]->m_points[0].Data[1];
            l_gains[i] = PositionActionators[i]->m_points[0].Data[2];
        }
    }
    
    // now copy the results to the output vectors
    isvalid = l_isvalid;
    time = l_time;
    positions = l_positions;
    velocities = l_velocities;
    gains = l_gains;
    
    if (l_num_joints > 0)
        return true;
    else
        return false;
}

/*! @brief Gets the next torque control point
 
 @param isvalid a vector of bools that indicates whether there is a new target for each joint.
 @param time the time each torque control should be completed will be put in this vector
 @param torques the target torques for each joint will be put in this vector
 @param gains the target gains for each joint will be put in this vector
 */
bool NUActionatorsData::getNextJointTorques(vector<bool>& isvalid, vector<double>& time, vector<float>& torques, vector<float>& gains)
{
    static int l_num_joints = TorqueActionators.size();
    static vector<bool> l_isvalid(l_num_joints, false);
    static vector<double> l_time(l_num_joints, 0);
    static vector<float> l_torques(l_num_joints, 0);
    static vector<float> l_gains(l_num_joints, 0);
    
    // loop through each actionator in TorqueActionators looking for non-empty actionators with the right datalength
    for (int i=0; i<l_num_joints; i++)
    {
        if(TorqueActionators[i]->isEmpty() || TorqueActionators[i]->m_points[0].Data.size() != 2)
        {
            l_isvalid[i] = false;
        }
        else
        {
            l_isvalid[i] = true;
            l_time[i] = TorqueActionators[i]->m_points[0].Time;
            l_torques[i] = TorqueActionators[i]->m_points[0].Data[0];
            l_gains[i] = TorqueActionators[i]->m_points[0].Data[1];
        }
    }
    
    // now copy the results to the output vectors
    isvalid = l_isvalid;
    time = l_time;
    torques = l_torques;
    gains = l_gains;
    
    if (l_num_joints > 0)
        return true;
    else
        return false;
}

/*! @brief Gets the next led point
 
    @param isvalid a vector of bools that indicates whether there is a new target for each led.
    @param time the time each led should be completed will be put in this vector
    @param redvalues the target red value for each led will be put in this vector
    @param greenvalues the target green value for each led will be put in this vector
    @param bluevalues the target blue value for each led will be put in this vector
 
    @return return false if there are no leds on this platform
 */
bool NUActionatorsData::getNextLeds(vector<bool>& isvalid, vector<double>& time, vector<float>& redvalues, vector<float>& greenvalues, vector<float>& bluevalues)
{
    static int l_num_leds = LedActionators.size();
    static vector<bool> l_isvalid(l_num_leds, false);
    static vector<double> l_time(l_num_leds, 0);
    static vector<float> l_redvalues(l_num_leds, 0);
    static vector<float> l_greenvalues(l_num_leds, 0);
    static vector<float> l_bluevalues(l_num_leds, 0);
    
    // loop through each actionator in LedActionators looking for non-empty actionators with the right datalength
    for (int i=0; i<l_num_leds; i++)
    {
        if(LedActionators[i]->isEmpty() || LedActionators[i]->m_points[0].Data.size() != 3)
        {
            l_isvalid[i] = false;
        }
        else
        {
            l_isvalid[i] = true;
            l_time[i] = LedActionators[i]->m_points[0].Time;
            l_redvalues[i] = LedActionators[i]->m_points[0].Data[0];
            l_greenvalues[i] = LedActionators[i]->m_points[0].Data[1];
            l_bluevalues[i] = LedActionators[i]->m_points[0].Data[2];
        }
    }
    
    // now copy the results to the output vectors
    isvalid = l_isvalid;
    time = l_time;
    redvalues = l_redvalues;
    greenvalues = l_greenvalues;
    bluevalues = l_bluevalues;
    
    if (l_num_leds > 0)
        return true;
    else
        return false;
}

/*! @brief Gets the next sound
 
    @param isvalid true if the data is valid, false otherwise
    @param time the time in milliseconds the sound should be played
    @param sound the filename of the sound to be played
    
    @return false if there is no next sound, true if there is a next sound
 */
bool NUActionatorsData::getNextSounds(bool& isvalid, double& time, vector<string>& sounds)
{
    if (Sound == NULL || Sound->isEmpty() || Sound->m_points[0].Data.size() == 0)
        return false;
    else 
    {
        isvalid = true;
        time = Sound->m_points[0].Time;
        sounds = Sound->m_points[0].Data;
        return true;
    }

}

/*! @brief Gets the next teleporation command
 
    @param isvalid true if the data is valid, false otherwise
    @param time the time in milliseconds the teleportation will take place :D
    @param data the target teleportation position [x (cm), y(cm), bearing (rad)] :D
    
 */
bool NUActionatorsData::getNextTeleportation(bool& isvalid, double& time, vector<float>& data)
{
    if (Teleporter == NULL || Teleporter->isEmpty() || Teleporter->m_points[0].Data.size() != 3)
        return false;
    else 
    {
        isvalid = true;
        time = Teleporter->m_points[0].Time;
        data = Teleporter->m_points[0].Data;
        return true;
    }
}


/******************************************************************************************************************************************
 Set Methods
 ******************************************************************************************************************************************/

/*! @brief Adds a single joint position control point
    @param jointid the id of the joint you want to control
    @param time the time at which you want the joint to reach its target (in milliseconds)
    @param position the target position (in radians)
    @param velocity the target velocity (in rad/s)
    @param gain the target gain (in Percent)
 */
bool NUActionatorsData::addJointPosition(joint_id_t jointid, double time, float position, float velocity, float gain)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else 
    {
        static vector<float> data (3, 0);
        data[0] = position;
        data[1] = velocity;
        data[2] = gain;
        PositionActionators[jointid]->addPoint(time, data);
        return true;
    }
}

/*! @brief Adds a position/velocity curve for a single joint
    @param jointid the id of the joint you want to control
    @param times the times for each of the points in the specified motion curve (in milliseconds)
    @param positions the target positions (in radians)
    @param velocities the target velocities (in rad/s)
    @param gain the target gain used for the entire curve (in Percent)
 */
bool NUActionatorsData::addJointPositions(joint_id_t jointid, const vector<double>& times, const vector<float>& positions, const vector<float>& velocities, float gain)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else
    {
        unsigned int timelength = times.size();
        unsigned int positionlength = positions.size();
        unsigned int velocitylength = velocities.size();
        
        if (positionlength < timelength || velocitylength < timelength)
            return false;
        else
        {
            for (unsigned int i=0; i<timelength; i++)
                addJointPosition(jointid, times[i], positions[i], velocities[i], gain);
        }
    }
    return true;
}

/*! @brief Adds a position/velocity curve for a single joint
    @param jointid the id of the joint you want to control
    @param times the times for each of the points in the specified motion curve (in milliseconds)
    @param positions the target positions (in radians)
    @param velocities the target velocities (in rad/s)
    @param gains the target gains (in Percent)
 */
bool NUActionatorsData::addJointPositions(joint_id_t jointid, const vector<double>& times, const vector<float>& positions, const vector<float>& velocities, const vector<float>& gains)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else
    {
        unsigned int timelength = times.size();
        unsigned int positionlength = positions.size();
        unsigned int velocitylength = velocities.size();
        unsigned int gainlength = gains.size();
        
        if (positionlength < timelength || velocitylength < timelength || gainlength < timelength)
            return false;
        else
        {
            for (unsigned int i=0; i<timelength; i++)
                addJointPosition(jointid, times[i], positions[i], velocities[i], gains[i]);
        }
    }
    return true;
}

/*! @brief Adds a single joint torque control point
    @param jointid the id of the joint you want to control
    @param time the time at which you want the joint to reach its target  (in milliseconds)
    @param torque the target torque (in Nm)
    @param gain the target gain (in Percent)
 */
bool NUActionatorsData::addJointTorque(joint_id_t jointid, double time, float torque, float gain)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else 
    {
        static vector<float> data (2, 0);
        data[0] = torque;
        data[1] = gain;
        TorqueActionators[jointid]->addPoint(time, data);
        return true;
    }
}

/*! @brief Adds a torque curve for a single joint
 @param jointid the id of the joint you want to control
 @param times the times for each of the points in the specified motion curve (in milliseconds)
 @param positions the target positions (in radians)
 @param gain the target gain used for the entire curve (in Percent)
 */
bool NUActionatorsData::addJointTorques(joint_id_t jointid, const vector<double>& times, const vector<float>& torques, float gain)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else
    {
        unsigned int timelength = times.size();
        unsigned int torquelength = torques.size();
        
        if (torquelength < timelength)
            return false;
        else
        {
            for (unsigned int i=0; i<timelength; i++)
                addJointTorque(jointid, times[i], torques[i], gain);
        }
    }
    return true;
}

/*! @brief Adds a torque curve for a single joint
    @param jointid the id of the joint you want to control
    @param times the times for each of the points in the specified motion curve (in milliseconds)
    @param positions the target positions (in radians)
    @param gains the target gains (in Percent)
 */
bool NUActionatorsData::addJointTorques(joint_id_t jointid, const vector<double>& times, const vector<float>& torques, const vector<float>& gains)
{
    if (jointid == ACTIONATOR_MISSING)
        return false;
    else
    {
        unsigned int timelength = times.size();
        unsigned int torquelength = torques.size();
        unsigned int gainlength = gains.size();
        
        if (torquelength < timelength || gainlength < timelength)
            return false;
        else
        {
            for (unsigned int i=0; i<timelength; i++)
                addJointTorque(jointid, times[i], torques[i], gains[i]);
        }
    }
    return true;
}

/*! @brief Returns the selected joint ids in the given bodypart
    @param partid the body part id you want the joint ids for
    @return a vector of the joint_id_t's in the body part
 */
vector<NUActionatorsData::joint_id_t> NUActionatorsData::getSelectedJoints(bodypart_id_t partid)
{
    vector<joint_id_t> selectedjoints;
    if (partid != ACTIONATOR_MISSING)
    {
        if (partid == AllJoints)
            selectedjoints = m_all_joint_ids;
        else if (partid == BodyJoints)
            selectedjoints = m_body_ids;
        else if (partid == HeadJoints)
            selectedjoints = m_head_ids;
        else if (partid == LeftArmJoints)
            selectedjoints = m_larm_ids;
        else if (partid == RightArmJoints)
            selectedjoints = m_rarm_ids;
        else if (partid == TorsoJoints)
            selectedjoints = m_torso_ids;
        else if (partid == LeftLegJoints)
            selectedjoints = m_lleg_ids;
        else if (partid == RightLegJoints)
            selectedjoints = m_rleg_ids;
        else
            debug << "NUActionatorsData::getSelectedJoints(). UNDEFINED Body part.";
    }
    return selectedjoints;
}

/*! @brief Adds a single joint position control point for each joint in the selected body part (the body part could be 'All' to set all joints at once)
    @param partid the id of the body part you want to control
    @param time the time at which you want the part will reach its target (in milliseconds)
    @param positions the target position (in radians)
    @param velocities the target velocities (in rad/s)
    @param gain the target gain for every joint in the body part (in Percent)
 */
bool NUActionatorsData::addJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& velocities, float gain)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    if (selectedjoints.size() < positions.size())
    {
        debug << "NUActionatorsData::addJointPositions. Specified positions are not the correct length. They are " << positions.size() << " and they should be " << selectedjoints.size() << endl;
        return false;
    }
    else 
    {
        for (unsigned int i=0; i<selectedjoints.size(); i++)
            addJointPosition(selectedjoints[i], time, positions[i], velocities[i], gain);
        return true;
    }
}

/*! @brief Adds a single joint position control point for each joint in the selected body part (the body part could be 'All' to set all joints at once)
    @param partid the id of the body part you want to control
    @param time the time at which you want the part will reach its target (in milliseconds)
    @param positions the target position (in radians)
    @param velocities the target velocities (in rad/s)
    @param gains the target gains (in Percent)
 */
bool NUActionatorsData::addJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& velocities, const vector<float>& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    if (selectedjoints.size() < positions.size())
    {
        debug << "NUActionatorsData::addJointPositions. Specified positions are not the correct length. They are " << positions.size() << " and they should be " << selectedjoints.size() << endl;
        return false;
    }
    else 
    {
        for (unsigned int i=0; i<selectedjoints.size(); i++)
            addJointPosition(selectedjoints[i], time, positions[i], velocities[i], gains[i]);
        return true;
    }
}

/*! @brief Adds joint torque control points for a body part (the body part could be 'All' to set all joints at once)
    @param partid the id of the body part you want to control
    @param time the time at which you want the joint to reach its target  (in milliseconds)
    @param torques the target torque (in Nm)
    @param gain the target gain for all joints in the body part (in Percent)
 */
bool NUActionatorsData::addJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, float gain)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    if (selectedjoints.size() < torques.size())
    {
        debug << "NUActionatorsData::addJointTorques. Specified torques are not the correct length. They are " << torques.size() << " and they should be " << selectedjoints.size() << endl;
        return false;
    }
    else 
    {
        for (unsigned int i=0; i<selectedjoints.size(); i++)
            addJointTorque(selectedjoints[i], time, torques[i], gain);
        return true;
    }
}

/*! @brief Adds joint torque control points for a body part (the body part could be 'All' to set all joints at once)
    @param partid the id of the body part you want to control
    @param time the time at which you want the joint to reach its target  (in milliseconds)
    @param torques the target torque (in Nm)
    @param gains the target gain (in Percent)
 */
bool NUActionatorsData::addJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, const vector<float>& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    if (selectedjoints.size() < torques.size())
    {
        debug << "NUActionatorsData::addJointTorques. Specified torques are not the correct length. They are " << torques.size() << " and they should be " << selectedjoints.size() << endl;
        return false;
    }
    else 
    {
        for (unsigned int i=0; i<selectedjoints.size(); i++)
            addJointTorque(selectedjoints[i], time, torques[i], gains[i]);
        return true;
    }
}

/*! @brief Adds a a motion curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ... , [posi0, posi1, ...]_N]
 
    @param partid the id of the body part you want to control
    @param time the time at which you want the part will reach its target (in milliseconds)
    @param positions the target position (in radians)
    @param velocities the target velocities (in rad/s)
    @param gain the target gain for every joint for the entire curve (in Percent)
 */
bool NUActionatorsData::addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, float gain)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || positions.size() < numjoints || velocities.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointPositions(selectedjoints[i], times[i], positions[i], velocities[i], gain); 
    
    return true;
}

/*! @brief Adds a a motion curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ... , [posi0, posi1, ...]_N]

    @param partid the id of the body part you want to control
    @param time the time at which you want the part will reach its target (in milliseconds)
    @param positions the target position (in radians)
    @param velocities the target velocities (in rad/s)
    @param gain the target gain for each joint for the entire curve (in Percent)
 */
bool NUActionatorsData::addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, const vector<float>& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || positions.size() < numjoints || velocities.size() < numjoints || gains.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointPositions(selectedjoints[i], times[i], positions[i], velocities[i], gains[i]); 
    
    return true;
}

/*! @brief Adds a a motion curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ... , [posi0, posi1, ...]_N]
 
    @param partid the id of the body part you want to control
    @param time the time at which you want the part will reach its target (in milliseconds)
    @param positions the target position (in radians)
    @param velocities the target velocities (in rad/s)
    @param gains the target gains (in Percent)
 */
bool NUActionatorsData::addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, const vector<vector<float> >& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || positions.size() < numjoints || velocities.size() < numjoints || gains.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointPositions(selectedjoints[i], times[i], positions[i], velocities[i], gains[i]); 
    
    return true;
}

/*! @brief Adds a torque curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[torq0, torq1, ...]_0, [torq0, torq1, ...]_1, ... , [torq0, torq1, ...]_N]

    @param partid the id of the body part you want to control
    @param times the time for each torque point (in milliseconds)
    @param torques the target torque (in Nm)
    @param gain the target gain for every joint of the entire curve (in Percent)
 */
bool NUActionatorsData::addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, float gain)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || torques.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointTorques(selectedjoints[i], times[i], torques[i], gain); 
    
    return true;
}

/*! @brief Adds a torque curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[torq0, torq1, ...]_0, [torq0, torq1, ...]_1, ... , [torq0, torq1, ...]_N]

    @param partid the id of the body part you want to control
    @param times the time for each torque point (in milliseconds)
    @param torques the target torque (in Nm)
    @param gains the target gain for each joint of the entire curve (in Percent)
 */
bool NUActionatorsData::addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, const vector<float>& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || torques.size() < numjoints || gains.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointTorques(selectedjoints[i], times[i], torques[i], gains[i]); 
    
    return true;
}

/*! @brief Adds a torque curve for each joint in the given body part
 
    For a body part with N joints the data needs to take the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ... , [time0, time1, ...]_N]
    [[torq0, torq1, ...]_0, [torq0, torq1, ...]_1, ... , [torq0, torq1, ...]_N]

    @param partid the id of the body part you want to control
    @param times the time for each torque point (in milliseconds)
    @param torques the target torque (in Nm)
    @param gains the target gains (in Percent)
 */
bool NUActionatorsData::addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, const vector<vector<float> >& gains)
{
    vector<joint_id_t> selectedjoints = getSelectedJoints(partid);
    
    // check that there is enough joint data in the given vectors
    unsigned int numjoints = selectedjoints.size();
    if (times.size() < numjoints || torques.size() < numjoints || gains.size() < numjoints)
    { 
        debug << "NUActionatorsData::addJointPositions. Specfied curve does not have enough joints; has " << times.size() << " needs " << numjoints << endl;
        return false;
    }
    for (unsigned int i=0; i<numjoints; i++)
        addJointTorques(selectedjoints[i], times[i], torques[i], gains[i]); 
    
    return true;
}

/*! @brief Adds a single led control point
    @param ledid the id of the led you want to control
    @param time the time at which you want the led to reach its target (in milliseconds)
    @param redvalue the target red value (0 to 1, or 0 to 255)
    @param greenvalue the target red value (0 to 1, or 0 to 255)
    @param bluevalue the target red value (0 to 1, or 0 to 255)
 */
bool NUActionatorsData::addLeds(ledgroup_id_t ledgroup, double time, const vector<vector<float> >& values)
{
    vector<joint_id_t> selectedleds;
    if (LedActionators.size() == 0)
        return false;                       
    
    if (ledgroup == AllLeds)
        selectedleds = m_all_led_ids;
    else if (ledgroup == LeftEarLeds)
        selectedleds = m_lear_ids;
    else if (ledgroup == RightEarLeds)
        selectedleds = m_rear_ids;
    else if (ledgroup == LeftEyeLeds)
        selectedleds = m_leye_ids;
    else if (ledgroup == RightEyeLeds)
        selectedleds = m_reye_ids;
    else if (ledgroup == ChestLeds)
        selectedleds = m_chest_ids;
    else if (ledgroup == LeftFootLeds)
        selectedleds = m_lfoot_ids;
    else if (ledgroup == RightFootLeds)
        selectedleds = m_rfoot_ids;
    else
    {
        debug << "NUActionatorsData::addLeds. UNDEFINED led group.";
        return false;
    }
    
    if (values.size() == 1)
    {   // if the size of the values is one, then set that value to all in the group
        static vector<float> data(3,0);
        if (values[0].size() < 3)
        {
            data[0] = values[0][0];
            data[1] = values[0][0];
            data[2] = values[0][0];
        }
        else 
        {
            data[0] = values[0][0];
            data[1] = values[0][1];
            data[2] = values[0][2];
        }

        for (unsigned int i=0; i<selectedleds.size(); i++)
        {
            LedActionators[selectedleds[i]]->addPoint(time, data);
        }
    }
    else
    {   // otherwise set each led individually.
        int numpoints = std::min(selectedleds.size(), values.size());
        
        static vector<float> data (3, 0);
        for (int i=0; i<numpoints; i++)
        {
            if (values[i].size() < 3)
            {
                data[0] = values[i][0];
                data[1] = values[i][0];
                data[2] = values[i][0];
            }
            else
            {
                data[0] = values[i][0];
                data[1] = values[i][1];
                data[2] = values[i][2];
            }
            LedActionators[selectedleds[i]]->addPoint(time, data);
        }
    }
    return true;
}

/*! @brief Adds a single sound
    @param time the time in milliseconds to play the sound
    @param sound the filename of the sound to be played
 
    @return true if the sound is successfully added, false otherwise
 */
bool NUActionatorsData::addSound(double time, string sound)
{
    static vector<string> data (1, "");
    if (Sound == NULL)
        return false;
    else 
    {
        data[0] = sound;
        Sound->addPoint(time, data);
        return true;
    }
}

/*! @brief Adds a several sounds to be played
    @param time the time in milliseconds to play the sound
    @param sounds the filenames of the sounds to be played

    @return true if the sounds are successfully added, false otherwise
 */
bool NUActionatorsData::addSounds(double time, vector<string> sounds)
{
    if (Sound == NULL)
        return false;
    else 
    {
        Sound->addPoint(time, sounds);
        return true;
    }
}

/*! @brief Adds a single teleportation command
 
    @param time the time in milliseconds to teleport
    @param x the x position (cm)
    @param y the y position (cm)
    @param bearing the bearing (rad)
 
    @return true if the command is successfully added, false otherwise
 */
bool NUActionatorsData::addTeleportation(double time, float x, float y, float bearing)
{
    static vector<float> data (3, 0);
    if (Teleporter == NULL)
        return false;
    else
    {
        data[0] = x;
        data[1] = y;
        data[2] = bearing;
        Teleporter->addPoint(time, data);
        return true;
    }
}

/******************************************************************************************************************************************
 Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

void NUActionatorsData::summaryTo(ostream& output)
{
    if (m_all_actionators.size() == 0 && m_all_string_actionators.size() == 0)
        output << "NONE!" << endl;
    for (unsigned int i=0; i<m_all_actionators.size(); i++)
        m_all_actionators[i]->summaryTo(output);
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
        m_all_string_actionators[i]->summaryTo(output);
}

void NUActionatorsData::csvTo(ostream& output)
{
    //! @todo TODO: implement this function    
}

ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor)
{
    //! @todo TODO: implement this function
    return output;
}

istream& operator>> (istream& input, NUActionatorsData& p_sensor)
{
    //! @todo TODO: implement this function
    return input;
}


