/*! @file NUData.cpp
    @brief Implementation of an abstract NUData class
    @author Jason Kulk
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

#include "NUData.h"
#include <algorithm>
#include <cctype>

#include "debug.h"
#include "debugverbositynuactionators.h"
#include "debugverbositynusensors.h"

int curr_id = 0;
vector<NUData::id_t*> NUData::m_common_ids;
const NUData::id_t NUData::All(curr_id++, "All", NUData::m_common_ids);
const NUData::id_t NUData::Head(curr_id++, "Head", NUData::m_common_ids);
const NUData::id_t NUData::Body(curr_id++, "Body", NUData::m_common_ids);
const NUData::id_t NUData::LArm(curr_id++, "LArm", NUData::m_common_ids);
const NUData::id_t NUData::RArm(curr_id++, "RArm", NUData::m_common_ids);
const NUData::id_t NUData::Torso(curr_id++, "Torso", NUData::m_common_ids);
const NUData::id_t NUData::LLeg(curr_id++, "LLeg", NUData::m_common_ids);
const NUData::id_t NUData::RLeg(curr_id++, "RLeg", NUData::m_common_ids);
const NUData::id_t NUData::NumCommonGroupIds(curr_id++, "NumCommonGroupIds", NUData::m_common_ids);

const NUData::id_t NUData::HeadRoll(curr_id++, "HeadRoll", NUData::m_common_ids);
const NUData::id_t NUData::HeadPitch(curr_id++, "HeadPitch", NUData::m_common_ids);
const NUData::id_t NUData::HeadYaw(curr_id++, "HeadYaw", NUData::m_common_ids);
const NUData::id_t NUData::NeckRoll(curr_id++, "NeckRoll", NUData::m_common_ids);
const NUData::id_t NUData::NeckPitch(curr_id++, "NeckPitch", NUData::m_common_ids);
const NUData::id_t NUData::NeckYaw(curr_id++, "NeckYaw", NUData::m_common_ids);
const NUData::id_t NUData::LShoulderRoll(curr_id++, "LShoulderRoll", NUData::m_common_ids);
const NUData::id_t NUData::LShoulderPitch(curr_id++, "LShoulderPitch", NUData::m_common_ids);
const NUData::id_t NUData::LShoulderYaw(curr_id++, "LShoulderYaw", NUData::m_common_ids);
const NUData::id_t NUData::LElbowRoll(curr_id++, "LElbowRoll", NUData::m_common_ids);
const NUData::id_t NUData::LElbowPitch(curr_id++, "LElbowPitch", NUData::m_common_ids);
const NUData::id_t NUData::LElbowYaw(curr_id++, "LElbowYaw", NUData::m_common_ids);
const NUData::id_t NUData::RShoulderRoll(curr_id++, "RShoulderRoll", NUData::m_common_ids);
const NUData::id_t NUData::RShoulderPitch(curr_id++, "RShoulderPitch", NUData::m_common_ids);
const NUData::id_t NUData::RShoulderYaw(curr_id++, "RShoulderYaw", NUData::m_common_ids);
const NUData::id_t NUData::RElbowRoll(curr_id++, "RElbowRoll", NUData::m_common_ids);
const NUData::id_t NUData::RElbowPitch(curr_id++, "RElbowPitch", NUData::m_common_ids);
const NUData::id_t NUData::RElbowYaw(curr_id++, "RElbowYaw", NUData::m_common_ids);
const NUData::id_t NUData::TorsoRoll(curr_id++, "TorsoRoll", NUData::m_common_ids);
const NUData::id_t NUData::TorsoPitch(curr_id++, "TorsoPitch", NUData::m_common_ids);
const NUData::id_t NUData::TorsoYaw(curr_id++, "TorsoYaw", NUData::m_common_ids);
const NUData::id_t NUData::LHipRoll(curr_id++, "LHipRoll", NUData::m_common_ids);
const NUData::id_t NUData::LHipPitch(curr_id++, "LHipPitch", NUData::m_common_ids);
const NUData::id_t NUData::LHipYaw(curr_id++, "LHipYaw", NUData::m_common_ids);
const NUData::id_t NUData::LHipYawPitch(curr_id++, "LHipYawPitch", NUData::m_common_ids);
const NUData::id_t NUData::LKneePitch(curr_id++, "LKneePitch", NUData::m_common_ids);
const NUData::id_t NUData::LAnkleRoll(curr_id++, "LAnkleRoll", NUData::m_common_ids);
const NUData::id_t NUData::LAnklePitch(curr_id++, "LAnklePitch", NUData::m_common_ids);
const NUData::id_t NUData::RHipRoll(curr_id++, "RHipRoll", NUData::m_common_ids);
const NUData::id_t NUData::RHipPitch(curr_id++, "RHipPitch", NUData::m_common_ids);
const NUData::id_t NUData::RHipYaw(curr_id++, "RHipYaw", NUData::m_common_ids);
const NUData::id_t NUData::RHipYawPitch(curr_id++, "RHipYawPitch", NUData::m_common_ids);
const NUData::id_t NUData::RKneePitch(curr_id++, "RKneePitch", NUData::m_common_ids);
const NUData::id_t NUData::RAnkleRoll(curr_id++, "RAnkleRoll", NUData::m_common_ids);
const NUData::id_t NUData::RAnklePitch(curr_id++, "RAnklePitch", NUData::m_common_ids);
const NUData::id_t NUData::NumJointIds(curr_id++, "NumJointIds", NUData::m_common_ids);

const NUData::id_t NUData::NumCommonIds(curr_id++, "NumCommonIds", NUData::m_common_ids);


void NUData::addDevices(const vector<string>& hardwarenames)
{
    vector<string> names = standardiseNames(hardwarenames);
    vector<id_t*>& ids = m_ids_copy;
    for (size_t i=0; i<names.size(); i++)
    {	// for each name compare it to the name of every id
        for (size_t j=0; j<ids.size(); j++)
        {
            id_t& id = *(ids[j]);
            if (id == names[i])
            {   // if the name matches the id, then add the actionator to m_available_ids and update the map
                if (find(m_available_ids.begin(), m_available_ids.end(), id.Id) == m_available_ids.end())
                    m_available_ids.push_back(id.Id);
                if (find(m_id_to_indices[id.Id].begin(), m_id_to_indices[id.Id].end(), id.Id) == m_id_to_indices[id.Id].end())
                    m_id_to_indices[id.Id].push_back(id.Id);
            }
        }
    }
    
    for (size_t i=0; i<ids.size(); i++)
    {	// fill in the groups
        for (size_t j=0; j<ids.size(); j++)
        {
            if (not m_id_to_indices[j].empty() and belongsToGroup(*ids[j], *ids[i]))
            {
                if (find(m_id_to_indices[i].begin(), m_id_to_indices[i].end(), j) == m_id_to_indices[i].end())
                    m_id_to_indices[i].push_back(j);
            }
        }
    }
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0 or DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "NUData::addDevices:" << endl;
        printMap(debug);
    #endif
}

/*! @brief Returns a vector containing the standardised versions of the vector containing hardware names 
    @param hardwarenames a list of hardwarenames
    @return a vector with the simplified names
 */
vector<string> NUData::standardiseNames(const vector<string>& hardwarenames)
{
    vector<string> simplenames;
    for (size_t i=0; i<hardwarenames.size(); i++)
    {
        string simplename = getStandardName(hardwarenames[i]);
        if (simplenames.empty())
            simplenames.push_back(simplename);    
        else if (simplename.compare(simplenames.back()) != 0)
            simplenames.push_back(simplename);
    }
    return simplenames;
}

/*! @brief Returns a simplified version of the hardwarename, formatting is removed.
    @param hardwarename the string to simplify
    @return the simplename
*/
string NUData::getStandardName(const string& hardwarename)
{
    string simplename, currentletter;
    // compare each letter to a space, an underscore, a forward slash, a backward slash and a period
    for (unsigned int j=0; j<hardwarename.size(); j++)
    {
        currentletter = hardwarename.substr(j, 1);
        if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0 && currentletter.compare(string("/")) != 0 && currentletter.compare(string("\\")) != 0 && currentletter.compare(string(".")) != 0)
            simplename += currentletter[0];            
    }
    
    // Replace "Left"/"Right" with L/R and move to front of name
    int Left = simplename.find("Left");
    int Right = simplename.find("Right");
    if (Left != string::npos)
    {
        simplename.erase(Left, 4);
        simplename.insert(0, "L");
    }
    if (Right != string::npos)
    {
        simplename.erase(Right, 5);
        simplename.insert(0, "R");
    }
    
    // Replace plurals (ears, eyes)
    int Ears = simplename.find("Ears");
    int Eyes = simplename.find("Eyes");
    if (Ears != string::npos)
        simplename.replace(Ears, 4, "Ear");
    if (Eyes != string::npos)
        simplename.replace(Ears, 4, "Eye");
    
    // Replace ChestBoard with Chest
    int ChestBoard = simplename.find("ChestBoard");
    if (ChestBoard != string::npos)
        simplename.replace(ChestBoard, 10, "Chest");
    
    // Replace LFace with LEye and RFace with REye
    int LFace = simplename.find("LFace");
    int RFace = simplename.find("RFace");
    if (LFace != string::npos)
    	simplename.replace(LFace, 5, "LEye");
    if (RFace != string::npos)
    	simplename.replace(RFace, 5, "REye");
    
    // Remove colours
    int Red = simplename.find("Red");
    if (Red != string::npos)
        simplename.erase(Red, 3);
    int Green = simplename.find("Green");
    if (Green != string::npos)
        simplename.erase(Green, 5);
    int Blue = simplename.find("Blue");
    if (Blue != string::npos)
        simplename.erase(Blue, 4);
    
    // Remove everything after a number
    int index = -1;
    for (size_t i=simplename.size()-1; i>0; i--)
    {
        if (isdigit(simplename[i]) and not isdigit(simplename[i-1]))
        {
            index = i;
            break;
        }
    }
    if (index >= 0)
        simplename.erase(index);
    
    return simplename;
}

/*! @brief Returns true if member belongs to group 
 	@param member the single id
 	@param group the group id
 	@return true if member belongs to group 
 */
bool NUData::belongsToGroup(const id_t& member, const id_t& group)
{
    if (group == All)
    {
        for (size_t i=RLeg.Id+1; i<m_common_ids.size()-1; i++)
            if (member == *m_common_ids[i])
                return true;
        return false;
    }
    else if (group == Head)
    {
        if (member == HeadRoll or member == HeadPitch or member == HeadYaw or member == NeckRoll or member == NeckPitch or member == NeckYaw)
            return true;
        else
            return false;
    }
    else if (group == Body)
    {
        if (belongsToGroup(member, All) and not belongsToGroup(member, Head))
            return true;
        else
            return false;
    }
    else if (group == LArm)
    {
        if (member == LShoulderRoll or member == LShoulderPitch or member == LShoulderYaw or member == LElbowRoll or member == LElbowPitch or member == LElbowYaw)
            return true;
        else 
            return false;
    }
    else if (group == RArm)
    {
        if (member == RShoulderRoll or member == RShoulderPitch or member == RShoulderYaw or member == RElbowRoll or member == RElbowPitch or member == RElbowYaw)
            return true;
        else 
            return false;
    }
    else if (group == LLeg)
    {
        if (member == LHipRoll or member == LHipPitch or member == LHipYaw or member == LHipYawPitch or member == LKneePitch or member == LAnkleRoll or member == LAnklePitch)
            return true;
        else
            return false;
    }
    else if (group == RLeg)
    {
        if (member == RHipRoll or member == RHipPitch or member == RHipYaw or member == RHipYawPitch or member == RKneePitch or member == RAnkleRoll or member == RAnklePitch)
            return true;
        else
            return false;
    }
    else
        return false;
}

/*! @brief Returns a list of indices into m_sensors/m_actionators so that the sensors/actionators under id can be accessed
    @param id the id of the sensor/actionator(s) to get the indicies for
 */
vector<int>& NUData::mapIdToIndices(const id_t& id)
{
    return m_id_to_indices[id.Id];
}

void NUData::printMap(ostream& output)
{
    for (size_t j=0; j<m_id_to_indices.size(); j++)
    {
        output << m_ids_copy[j]->Name << "->[";
        for (size_t k=0; k<m_id_to_indices[j].size(); k++)
            output << m_ids_copy[m_id_to_indices[j][k]]->Name << " ";
        output << "]" << endl;
    }
}

