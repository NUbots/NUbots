/*! @file NUData.h
    @brief Declaration of an abstract NUData class
    @author Jason Kulk
 
    @class NUData
    @brief An abstract NUData: a place to put things that are common to both the NUSensorsData and the NUActionatorsData
 
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

#ifndef NUDATA_H
#define NUDATA_H

#include <string>
#include <vector>
using namespace std;

class NUData
{
public:
    struct id_t 
    {
        id_t(int id, const string& name, vector<NUData::id_t*>& list = NUData::Ids) 
        {
            Id = id; 
            Name = name;
            list.push_back(this);
        }
        int Id;
        string Name;
        bool operator==(const id_t& other)
        {
            return Id == other.Id;
        }
        bool operator==(const int& other)
        {
            return Id == other;
        }
        bool operator==(const string& other)
        {
            return Name.find(other) != string::npos;
        }
    };
    
    // Common aliases
    const static id_t All;
    const static id_t Head;
    const static id_t Body;
    const static id_t LArm;
    const static id_t RArm;
    const static id_t Torso;
    const static id_t LLeg;
    const static id_t RLeg;
    
    // define const static members that are both sensors and actionators
    const static id_t HeadRoll;
    const static id_t HeadPitch;
    const static id_t HeadYaw;
    const static id_t NeckRoll;
    const static id_t NeckPitch;
    const static id_t NeckYaw;
    const static id_t LShoulderRoll;
    const static id_t LShoulderPitch;
    const static id_t LShoulderYaw;
    const static id_t LElbowRoll;
    const static id_t LElbowPitch;
    const static id_t LElbowYaw;
    const static id_t RShoulderRoll;
    const static id_t RShoulderPitch;
    const static id_t RShoulderYaw;
    const static id_t RElbowRoll;
    const static id_t RElbowPitch;
    const static id_t RElbowYaw;
    const static id_t TorsoRoll;
    const static id_t TorsoPitch;
    const static id_t TorsoYaw;
    const static id_t LHipRoll;
    const static id_t LHipPitch;
    const static id_t LHipYaw;
    const static id_t LHipYawPitch;
    const static id_t LKneePitch;
    const static id_t LAnkleRoll;
    const static id_t LAnklePitch;
    const static id_t RHipRoll;
    const static id_t RHipPitch;
    const static id_t RHipYaw;
    const static id_t RHipYawPitch;
    const static id_t RKneePitch;
    const static id_t RAnkleRoll;
    const static id_t RAnklePitch;
    const static id_t NumCommonIds;
    
    double CurrentTime;    
    
protected:
    static vector<id_t*> Ids;
    vector<string> standardiseNames(const vector<string>& hardwarenames);
private:
    string getStandardName(const string& hardwarename);
};

#endif

