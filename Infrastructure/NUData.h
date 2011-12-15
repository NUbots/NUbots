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
    /*! @brief A structure to represent an id for a sensor and/or actionator 
     */
    struct id_t 
    {
        /*! @brief Default constructor for id_t */
        id_t()
        {
            Id = -1;
            Name = "";
        }
        /*! @brief Constructor for an id_t
         	@param id the unique integral id for this sensor/actionator
         	@param name a unigue name for the sensor/actionator
         	@param list the list of id_t's to add this new id_t to
         */
        id_t(int id, const string& name, vector<NUData::id_t*>& list) 
        {
            Id = id; 
            Name = name;
            list.push_back(this);
        }
        int Id;				//!< unique id
        string Name;		//!< unique name
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
            return other.find(Name) != string::npos;
        }
        bool operator!=(const id_t& other) const
        {
            return Id != other.Id;
        }
        bool operator!=(const int& other) const
        {
            return Id != other;
        }
        bool operator<(const id_t& other) const
        {
            return Id < other.Id;
        }
        bool operator>(const id_t& other) const
        {
            return Id > other.Id;
        }
        /*! @brief Returns true if this id_t is for a left sensor/actionator */
        bool isLeft() const
        {
            return Name[0] == 'L';
        }
        /*! @brief Returns true if this id_t is for a right sensor/actionator */
        bool isRight() const
        {
            return Name[0] == 'R';
        }
        
        friend ostream& operator <<(ostream& output, const id_t* id);
        friend istream& operator >>(istream& input, id_t* id);
    };
        
    // Common aliases
    const static id_t All;							//!< alias for 'all', can be used in a number of ways for getting all sensors or actionators in a group
    const static id_t Head;							//!< alias for 'head', can be used to get sensors or actionators in the head
    const static id_t Body;							//!< alias for 'body', this is always 'all' - 'head'
    const static id_t LArm;							//!< alias for the left arm
    const static id_t RArm;							//!< alias for the right arm
    const static id_t LHand;						//!< alias for the left hand
    const static id_t RHand;						//!< alias for the right hand
    const static id_t Torso;						//!< alias for the torso
    const static id_t LLeg;							//!< alias for the left leg
    const static id_t RLeg;							//!< alias for the right leg
    const static id_t LFoot;						//!< alias for the left foot
    const static id_t RFoot;						//!< alias for the right foot
    const static id_t NumCommonGroupIds;            //!< internal use only
    
    // define const static ids for the joints which are both sensors and actionators
    const static id_t HeadRoll;						//!< the head roll joint
    const static id_t HeadPitch;					//!< the head pitch joint
    const static id_t HeadYaw;						//!< the head yaw joint
    const static id_t NeckRoll;						//!< the neck roll joint
    const static id_t NeckPitch;					//!< the neck pitch joint
    const static id_t NeckYaw;						//!< the neck yaw joint
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
    const static id_t LHipYawPitch;
    const static id_t LHipYaw;
    const static id_t LKneePitch;
    const static id_t LAnkleRoll;
    const static id_t LAnklePitch;
    const static id_t RHipRoll;
    const static id_t RHipPitch;
    const static id_t RHipYawPitch;
    const static id_t RHipYaw;
    const static id_t RKneePitch;
    const static id_t RAnkleRoll;
    const static id_t RAnklePitch;
    const static id_t NumJointIds;                 	//!< internal use only
    
    // define const static ids for devices that are both sensors and actionators
    const static id_t NumCommonIds;                 //!< internal use only
    const static int m_num_common_ids = 49;			//!< this *MUST* be manually updated to match NumCommonIds.Id
    
    double CurrentTime;    
    double PreviousTime;
    
    vector<id_t*> mapIdToIds(const id_t& id);
    NUData::id_t* getId(const std::string& name);
protected:
    static vector<id_t*> m_common_ids;
    vector<id_t*> m_ids_copy;                       //!< this is a non-static copy of the ids. It is non-static so that it is not shared between derived classes (ie sensors and actionators have different ids)
    vector<vector<int> > m_id_to_indices;
    vector<int> m_available_ids;
    
protected:
    void addDevices(const vector<string>& hardwarenames);
    
    virtual bool belongsToGroup(const id_t& member, const id_t& group);
    virtual bool belongsToGroup(const string& name, const id_t& group);
    
    const vector<int>& mapIdToIndices(const id_t& id) const;
    
    // debug tools
    void printMap(ostream& output);
private:
    string getStandardName(const string& hardwarename);
    vector<string> standardiseNames(const vector<string>& hardwarenames);
    template<typename T> bool t_belongsToGroup(const T& member, const id_t& group);
};


#endif

