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
        id_t()
        {
            Id = -1;
            Name = "";
        }
        id_t(int id, const string& name, vector<NUData::id_t*>& list) 
        {
            Id = id; 
            Name = name;
            list.push_back(this);
        }
        int Id;
        string Name;
        bool operator==(const id_t& other) const
        {
            return Id == other.Id;
        }
        bool operator==(const int& other) const
        {
            return Id == other;
        }
        bool operator==(const string& other) const
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
    const static id_t NumCommonGroupIds;                //!< internal use only
    
    // define const static ids for the joints which are both sensors and actionators
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
    const static id_t NumJointIds;                      //!< internal use only
    
    // define const static ids for devices that are both sensors and actionators
    const static id_t NumCommonIds;                     //!< internal use only
    
    double CurrentTime;    
    
protected:
    static vector<id_t*> m_common_ids;
    vector<id_t*> m_ids_copy;                       //!< this is a non-static copy of the ids. It is non-static so that it is not shared between derived classes (ie sensors and actionators have different ids)
    vector<vector<int> > m_id_to_indices;
    vector<int> m_available_ids;
    
protected:
    void addDevices(const vector<string>& hardwarenames);
    
    virtual bool belongsToGroup(const id_t& member, const id_t& group);
    
    vector<int>& mapIdToIndices(const id_t& id);
    
    // debug tools
    void printMap(ostream& output);
private:
    string getStandardName(const string& hardwarename);
    vector<string> standardiseNames(const vector<string>& hardwarenames);
};

#endif

