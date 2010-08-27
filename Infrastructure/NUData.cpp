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

NUData::id_t NUData::All = 0;
NUData::id_t NUData::Head = 1;
NUData::id_t NUData::Body = 3;
NUData::id_t NUData::LArm = 4;
NUData::id_t NUData::RArm = 5;
NUData::id_t NUData::Torso = 6;
NUData::id_t NUData::LLeg = 7;
NUData::id_t NUData::RLeg = 8;

NUData::id_t NUData::HeadRoll = 9;
NUData::id_t NUData::HeadPitch = 10;
NUData::id_t NUData::HeadYaw = 11;
NUData::id_t NUData::NeckRoll = 12;
NUData::id_t NUData::NeckPitch = 13;
NUData::id_t NUData::NeckYaw = 14;
NUData::id_t NUData::LShoulderRoll = 15;
NUData::id_t NUData::LShoulderPitch = 16;
NUData::id_t NUData::LShoulderYaw = 17;
NUData::id_t NUData::LElbowRoll = 18;
NUData::id_t NUData::LElbowPitch = 19;
NUData::id_t NUData::LElbowYaw = 20;
NUData::id_t NUData::RShoulderRoll = 21;
NUData::id_t NUData::RShoulderPitch = 22;
NUData::id_t NUData::RShoulderYaw = 23;
NUData::id_t NUData::RElbowRoll = 24;
NUData::id_t NUData::RElbowPitch = 25;
NUData::id_t NUData::RElbowYaw = 26;
NUData::id_t NUData::TorsoRoll = 27;
NUData::id_t NUData::TorsoPitch = 28;
NUData::id_t NUData::TorsoYaw = 29;
NUData::id_t NUData::LHipRoll = 30;
NUData::id_t NUData::LHipPitch = 31;
NUData::id_t NUData::LHipYaw = 32;
NUData::id_t NUData::LHipYawPitch = 33;
NUData::id_t NUData::LKneePitch = 34;
NUData::id_t NUData::LAnkleRoll = 35;
NUData::id_t NUData::LAnklePitch = 36;
NUData::id_t NUData::RHipRoll = 37;
NUData::id_t NUData::RHipPitch = 38;
NUData::id_t NUData::RHipYaw = 39;
NUData::id_t NUData::RHipYawPitch = 40;
NUData::id_t NUData::RKneePitch = 41;
NUData::id_t NUData::RAnkleRoll = 42;
NUData::id_t NUData::RAnklePitch = 43;
NUData::id_t NUData::NumCommonIds = 44;

/*! @brief Returns a vector containing the simplified versions of the vector containing hardware names 
    @param hardwarenames a list of hardwarenames
    @return a vector with the simplified names
 */
vector<string> NUData::simplifyNames(const vector<string>& hardwarenames)
{
    vector<string> simplenames;
    for (size_t i=0; i<hardwarenames.size(); i++)
        simplenames.push_back(getSimpleName(hardwarenames[i]));
    return simplenames;
}

/*! @brief Returns a simplified version of the hardwarename, formatting is removed and the name is converted to lower case
    @param hardwarename the string to simplify
    @return the simplename
*/
string NUData::getSimpleName(const string& hardwarename)
{
    string simplename, currentletter;
    // compare each letter to a space, an underscore, a forward slash, a backward slash and a period
    for (unsigned int j=0; j<hardwarename.size(); j++)
    {
        currentletter = hardwarename.substr(j, 1);
        if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0 && currentletter.compare(string("/")) != 0 && currentletter.compare(string("\\")) != 0 && currentletter.compare(string(".")) != 0)
            simplename += tolower(currentletter[0]);            
    }
    return simplename;
}

