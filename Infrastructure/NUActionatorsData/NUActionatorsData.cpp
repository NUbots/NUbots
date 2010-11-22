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
#include "Actionator.h"

#include "Infrastructure/NUBlackboard.h"			// need the blackboard and the sensors to do the interpolation
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include <sstream>

#include "Tools/Math/StlVector.h"

#include "debug.h"
#include "debugverbositynuactionators.h"

int a_curr_id = NUData::NumCommonIds.Id+1;
vector<NUActionatorsData::id_t*> NUActionatorsData::m_ids;
// Led actionators
const NUActionatorsData::id_t NUActionatorsData::LEarLed(a_curr_id++, "LEarLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::REarLed(a_curr_id++, "REarLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::LEyeLed(a_curr_id++, "LEyeLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::REyeLed(a_curr_id++, "REyeLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::ChestLed(a_curr_id++, "ChestLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::LFootLed(a_curr_id++, "LFootLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::RFootLed(a_curr_id++, "RFootLed", NUActionatorsData::m_ids);
// Led groups
const NUActionatorsData::id_t NUActionatorsData::FaceLeds(a_curr_id++, "FaceLeds", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::FeetLeds(a_curr_id++, "FeetLeds", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::AllLeds(a_curr_id++, "AllLeds", NUActionatorsData::m_ids);
// Other actionators
const NUActionatorsData::id_t NUActionatorsData::Sound(a_curr_id++, "Sound", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::Teleporter(a_curr_id++, "Teleporter", NUActionatorsData::m_ids);

/*! @brief Default constructor for a NUActionatorsData storage class.
 */
NUActionatorsData::NUActionatorsData()
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "NUActionatorsData::NUActionatorsData" << endl;
    #endif
    CurrentTime = 0;
    PreviousTime = 0;
    
    m_ids.insert(m_ids.begin(), NUData::m_common_ids.begin(), NUData::m_common_ids.end());
    m_ids_copy = m_ids;
    m_id_to_indices = vector<vector<int> >(m_ids.size(), vector<int>());
    
    for (size_t i=0; i<m_ids.size(); i++)
        m_actionators.push_back(Actionator(m_ids[i]->Name));
}

/*! @brief Destroys the NUActionatorsData storage class
 */
NUActionatorsData::~NUActionatorsData()
{
}

/******************************************************************************************************************************************
 Initialisation and Availability Setting Methods
 ******************************************************************************************************************************************/
/*! @brief Adds the actionators to the NUActionatorsData listed in hardwarenames
    @param hardwarenames a vector containing every actionator available on the robotic platform
 */
void NUActionatorsData::addActionators(const vector<string>& hardwarenames)
{
    addDevices(hardwarenames);
}

/*! @brief Returns true if member belongs to group (this function extends the NUData::belongsToGroup)
    @param member the single id
    @param group the group id
    @return true if member belongs to group 
 */
bool NUActionatorsData::belongsToGroup(const id_t& member, const id_t& group)
{
	if (member == group or FaceLeds == member or FeetLeds == member or AllLeds == member)			// we careful not to add ids for groups to groups
        return false;
    else if (NUData::belongsToGroup(member, group))
        return true;
    else
    {
        if (FaceLeds == group)
        {
            if (member.Name.find("Led") != string::npos and (member.Name.find("Eye") != string::npos or member.Name.find("Mouth") != string::npos))
                return true;
            else
                return false;
        }
        else if (FeetLeds == group)
        {
            if (member.Name.find("Led") != string::npos and member.Name.find("Foot") != string::npos)
                return true;
            else
                return false;
        }
        else if (AllLeds == group)
        {
            if (member.Name.find("Led") != string::npos)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}

/*! @brief Returns true if member belongs to group (this function extends the NUData::belongsToGroup)
    @param member the name of single id. The name is case sensitive
    @param group the group id
    @return true if member belongs to group 
 */
bool NUActionatorsData::belongsToGroup(const string& member, const id_t& group)
{
	if (group == member or FaceLeds == member or FeetLeds == member or AllLeds == member)			// we careful not to add ids for groups to groups
        return false;
    else if (NUData::belongsToGroup(member, group))
        return true;
    else
    {
        if (FaceLeds == group)
        {
            if (member.find("Led") != string::npos and (member.find("Eye") != string::npos or member.find("Mouth") != string::npos))
                return true;
            else
                return false;
        }
        else if (FeetLeds == group)
        {
            if (member.find("Led") != string::npos and member.find("Foot") != string::npos)
                return true;
            else
                return false;
        }
        else if (AllLeds == group)
        {
            if (member.find("Led") != string::npos)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}

/******************************************************************************************************************************************
 Get Methods
 ******************************************************************************************************************************************/
/*! @brief Pre processes the data to be ready for copying to hardware communication
 */
void NUActionatorsData::preProcess(double currenttime)
{
    PreviousTime = CurrentTime;
    CurrentTime = currenttime;
    for (size_t i=0; i<m_available_ids.size(); i++)
        m_actionators[m_available_ids[i]].preProcess();
}

/*! @brief Post processes the data after sending it to the hardware communications (Remove all of the completed actionator points)
    @param currenttime all actionator points that have times before this one are assumed to have been completed, and they will be removed
 */
void NUActionatorsData::postProcess()
{
    for (size_t i=0; i<m_available_ids.size(); i++)
        m_actionators[m_available_ids[i]].postProcess(CurrentTime);
}

/*! @brief Gets the target positions and gains for the servos.
 	@param positions will be updated with the positions for the next cycle
 	@param gains will be updated with the gains for the next cycle
 */
void NUActionatorsData::getNextServos(vector<float>& positions, vector<float>& gains)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "NUActionatorsData::getNextServos" << endl;
    #endif
    // get the sensor positions and gains
    vector<float> positions_current, gains_current;
    Blackboard->Sensors->getPosition(All, positions_current);
    Blackboard->Sensors->getStiffness(All, gains_current);
    
    // check that positions and gains are of the correct size; if they are not then 'resize' them
    if (positions.size() != positions_current.size())
        positions = positions_current;
    if (gains.size() != gains_current.size())
        gains = gains_current;

    // now do the interpolation for each joint that has new data
    vector<int>& ids = mapIdToIndices(All); 
    for (int i=0; i<ids.size(); i++)
    {
        Actionator& a = m_actionators[ids[i]];
        double time;
        float position;
        if (not a.empty())
        {
            if (a.get(time, position))
                positions[i] = interpolate(time, positions_current[i], position);
            else
            {
                vector<float> positiongain;
                if (a.get(time, positiongain))
                {
                    position = positiongain[0];
                    positions[i] = interpolate(time, positions_current[i], position);
                    gains[i] = interpolate(time, gains_current[i], positiongain[1]);
                }
            }
        }
        #if DEBUG_NUACTIONATORS_VERBOSITY > 0
            debug << a.Name << " [" << positions[i] << "," << gains[i] << "] target: [" << time - CurrentTime << "," << position << "]" << endl;
        #endif
    }
}

/*! @brief Gets the target [rgb] values for each led
 	@param leds will be updated with the [rgb] values for the next cycle
 */
void NUActionatorsData::getNextLeds(vector<vector<float> >& leds)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "NUActionatorsData::getNextLeds" << endl;
    #endif
	// since we have no led 'sensors' we just assume they are exactly what I set them to be
    
    // check that leds is the right size; if not 'resize' them
    vector<int>& ids = mapIdToIndices(AllLeds);
	if (leds.size() != ids.size())
        leds = vector<vector<float> >(ids.size(), vector<float>(3,0));
    
    for (int i=0; i<ids.size(); i++)
    {
        Actionator& a = m_actionators[ids[i]];
        if (not a.empty())
        {
            double time;
            float luminance;
            if (a.get(time, luminance))
            {
                float l = interpolate(time, leds[i][0], luminance);
                leds[i][0] = l;			// if the actionator stores a float; assume we want each colour to be set to that value
                leds[i][1] = l;
                leds[i][2] = l;
            }
            else
            {
                vector<float> rgb;
                if (a.get(time, rgb))
                {
                    leds[i][0] = interpolate(time, leds[i][0], rgb[0]);
                    leds[i][1] = interpolate(time, leds[i][1], rgb[1]);
                    leds[i][2] = interpolate(time, leds[i][2], rgb[2]);
                }
            }
        }
        #if DEBUG_NUACTIONATORS_VERBOSITY > 0
            debug << a.Name << " " << leds[i] << endl;
        #endif
    }
}

/*! @brief Interpolates the target value, producing the target for the next time cycle
 	@param time the timestamp (ms) to reach the target value
 	@param current the current value of the actionator
 	@param target the target actionator value
 	@return the interpolate actionator value for the next time step
 */
float NUActionatorsData::interpolate(const double& time, const float& current, const float& target)
{
    if (time - CurrentTime > CurrentTime - PreviousTime and CurrentTime - PreviousTime != 0)
        return current + ((target - current)/(time - CurrentTime))*(CurrentTime - PreviousTime);
    else
        return target;
}

/*! @brief Returns true if a member named name belongs to the group
 	@param name the name of the potential member. The name is case sensitive
 	@param group the id_t of the group
 	@return true if it is a member of group, false otherwise
 */
bool NUActionatorsData::isMemberOfGroup(const string& name, const id_t& group)
{
    return belongsToGroup(name, group);
}

/*! @brief Returns the number of actionators in actionatorid. Useful for determining, for example, the number of joints in a leg */
size_t NUActionatorsData::getSize(const id_t& actionatorid)
{
    return m_id_to_indices[actionatorid.Id].size();;
}

/******************************************************************************************************************************************
 Set Methods
 ******************************************************************************************************************************************/

/*! @brief Adds a single [time, data] point to the actionatorid
    @param actionatorid the id of the actionator to add the data
    @param time the time in ms associated with the data
    @param data the data
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, float data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    for (size_t i=0; i<ids.size(); i++)
        m_actionators[ids[i]].add(time, data);
}

/*! @brief Adds a [time, [data,gain]] tuple to the actionatorid
    @param actionatorid the id of the actionator to add the data
    @param time the time in ms associated with the data,gain point
    @param data the data
    @param gain the gain
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, float data, float gain)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << ")" << endl;
    #endif
    // combine the data and gain into a vector
    vector<float> d(2,data);
    d[1] = gain;
    // then just add it
    add(actionatorid, time, d);
}

/*! @brief Adds the data to the actionatorid with a single time. 
 
           If the size of the data matches the number of actionators in actionatorid
           then each element is given to each actionator.
           If the size does not match, or there is only a single actionator under this actionatorid
           then the actionator(s) are given the entire data.
 
    @param actionatorid the id of the targeted actionator(s)
    @param time the time in ms associated with the data
    @param data the data
*/
void NUActionatorsData::add(const id_t& actionatorid, double time, const vector<float>& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {	// if actionatorid is a single
        m_actionators[ids[0]].add(time, data);
    }
    else
    {   // if actionatorid is a group
        if (data.size() == numids)
        {	// if the data size matches the group size then assume we want each row of data given to each actionator
            for (size_t i=0; i<numids; i++) 
                m_actionators[ids[i]].add(time, data[i]);
        }
        else
        {	// if the data size differs then assume we want a copy of the entire vector given to each actionator
            for (size_t i=0; i<numids; i++) 
                m_actionators[ids[i]].add(time, data);
        }
    }
}

/*! @brief Adds the data and gain to the actionatorid with a single time. The size of the data must match the number
           of actionators under the id, and the same gain value is used for each.
 
    @param actionatorid the id of the targeted actionators
    @param time the time in ms associated with the data,gain
    @param data the data
    @param gain the gain
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, const vector<float>& data, float gain)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids > 1 and numids == data.size())
    {	// as we are including a gain, we must be assigning a single value from data to each actionator in a group
        vector<float> d(2,gain);
        for (size_t i=0; i<numids; i++)
        {
            d[0] = data[i];
            m_actionators[ids[i]].add(time, d);
        }
    }
    else
    {
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). The data is incorrectly formatted. ";
        debug << "data.size():" << data.size() << " must be ids.size()" << numids << endl;
    }
}

/*! @brief Adds the data and gain to the actionatorid with a single time. The size of the data and gains must match the number
           of actionators under the id
 
    @param actionatorid the id of the targeted actionators
    @param time the time in ms associated with the data,gain
    @param data the data
    @param gain the gain
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, const vector<float>& data, const vector<float>& gain)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids > 1 and numids == data.size() and numids == gain.size())
    {	// as we are including gains, we must assign a single data,gain pair to each actionator in a group
        vector<float> d(2,0);
        for (size_t i=0; i<numids; i++)
        {
            d[0] = data[i];
            d[1] = gain[i];
            m_actionators[ids[i]].add(time, d);
        }
    }
    else
    {
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). The data is incorrectly formatted. ";
        debug << "data.size():" << data.size() << " and gain.size():" << gain.size() << " must be ids.size()" << numids << endl;
    }
}

/*! @brief Adds the data to the actionatorid with a single time. 
 
            If the size of the data matches the number of actionators in actionatorid
            then each element is given to each actionator.
            If the size does not match, or there is only a single actionator under this actionatorid
            then the actionator(s) are given the entire data.
 
    @param actionatorid the id of the targeted actionator(s)
    @param time the time in ms associated with the data
    @param data the data
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, const vector<vector<float> >& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {	// if actionatorid is a single
        m_actionators[ids[0]].add(time, data);
    }
    else
    {   // if actionatorid is a group
        if (data.size() == numids)
        {	// if the data size matches the group size then assume we want each row of data given to each actionator
            for (size_t i=0; i<numids; i++)
                m_actionators[ids[i]].add(time, data[i]);
        }
        else
        {	// if the data size differs then assume we want a copy of the entire vector given to each actionator
            for (size_t i=0; i<numids; i++)
                m_actionators[ids[i]].add(time, data);
        }
    }
}

/*! @brief Adds the data to the actionatorid with a single time. 
 
             If the size of the data matches the number of actionators in actionatorid
             then each element is given to each actionator.
             If the size does not match, or there is only a single actionator under this actionatorid
             then the actionator(s) are given the entire data.
 
    @param actionatorid the id of the targeted actionator(s)
    @param time the time in ms associated with the data
    @param data the data
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, const vector<vector<vector<float> > >& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {	// if actionatorid is a single
        m_actionators[ids[0]].add(time, data);
    }
    else
    {   // if actionatorid is a group
        if (data.size() == numids)
        {	// if the data size matches the group size then assume we want each row of data given to each actionator
            for (size_t i=0; i<numids; i++)
                m_actionators[ids[i]].add(time, data[i]);
        }
        else
        {	// if the data size differs then assume we want a copy of the entire vector given to each actionator
            for (size_t i=0; i<numids; i++)
                m_actionators[ids[i]].add(time, data);
        }
    }
}

/*! @brief Adds a single [time, data] point to the actionatorid
    @param actionatorid the id of the actionator to add the data
    @param time the time in ms associated with the data
    @param data the data
 */
void NUActionatorsData::add(const id_t& actionatorid, double time, const string& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    for (size_t i=0; i<ids.size(); i++)
        m_actionators[ids[i]].add(time, data);
}

/*! @brief Adds a sequence of points to an actionatorid
        
        For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN].
        For a group actionator the data needs to be formatted as either 
            [id0, id1, ... , idN] [time0, time1, ... , timeN] [data0, data1, ... , dataN]
            [id0, id1, ... , idN] [time0, time1, ... , timeN] data
 
    @param actionatorid the id of the targetted actionator(s)
    @param time the sequence of times in ms
    @param data the sequence of data points
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();

    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        if (time.size() == data.size())
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        else
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). time and data need to have the same length when targeting a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids)
        {   // if there is a time entry for each member of the group
            if (data.size() == numids)
            {   // if the data size matches the group size, then each member gets a single element
                for (size_t i=0; i<numids; i++)
                    add(*m_ids[ids[i]], time[i], data[i]);
            }
            else
            {   // if the data size doesn't match, then each member gets the entire vector
                for (size_t i=0; i<numids; i++)
                    add(*m_ids[ids[i]], time[i], data);
            }
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << "time.size():" << time.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of points to an actionatorid with a gain. 
           The size of the time and data must match, and in the case actionatorid is a group, they must also match the group size.
 
     For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN], gain
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN] [data0, data1, ... , dataN], gain
     where in both cases gain is added to every point.
 
    @param actionatorid the id of the targetted actionator(s)
    @param time the sequence of times in ms
    @param data the sequence of data points
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data, float gain)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i], gain);
        }
        else
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). time and data need to have the same length when targeting a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i], gain);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << "time.size():" << time.size() << " and " << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [time,data,gain] points to an actionatorid
           The size of time, data and gain must be the same, in the case that actionatorid is a group, they must also match the group size.
    
     For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN], [gain0, gain1, ... , gainN]
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN] [data0, data1, ... , dataN], [gain0, gain1, ... , gainN]
 
    @param actionatorid the id of the targetted actionator(s)
    @param time the sequence of times in ms
    @param data the sequence of data points
    @param gain the sequence of gain points (%)
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data, const vector<float>& gain)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        if (time.size() == data.size() and time.size() == gain.size())
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i], gain[i]);
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << " time.size():" << time.size() << ", data.size():" << data.size() << "and gain.size():" << gain.size() << " must be the same." << endl;
        }
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids and gain.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i], gain[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << "time.size():" << time.size() << ", data.size():" << data.size() << " and gain.size():" << gain.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [time, vector] pairs to an actionatorid
 
    For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN], where each data is a vector.
    For a group actionator the data needs to
        id [time0, time1, ... , timeN], [data0, data1, ... , dataN], so each data element is given to the group with a single time
        [id0, id1, ... , idN] time [data0, data1, ... , dataN], so each actionator in the group receives a single data element at a sequence of times
        [id0, id1, ... , idN] [time0, time1, ... , timeN] data, so each actionator in the group receives the same data matrix at different times 
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<vector<float> >& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single
        if (time.size() == data.size())
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << " time.size():" << time.size() << " and data.size():" << data.size() << " must be the same." << endl;
        }
    }
    else
    {   // if actionatorid is a group
        if (time.size() == data.size())
        {	
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else if (numids == data.size())
        {	
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time, data[i]);
        }
        else if (time.size() == numids)
        {
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). lolwut?";
        }
    }
}

/*! @brief Adds a sequence of [time, matrix] pairs to an actionatorid
 
         For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN], where each data is a vector.
         For a group actionator the data needs to
             id [time0, time1, ... , timeN], [data0, data1, ... , dataN], so each data element is given to the group with a single time
             [id0, id1, ... , idN] time [data0, data1, ... , dataN], so each actionator in the group receives a single data element at a sequence of times
             [id0, id1, ... , idN] [time0, time1, ... , timeN] data, so each actionator in the group receives the same data matrix at different times 
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<vector<vector<float> > >& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << " time.size():" << time.size() << " and data.size():" << data.size() << " must be the same." << endl;
        }
    }
    else
    {   // if actionatorid is a group
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else if (numids == data.size())
        {
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time, data[i]);
        }
        else if (time.size() == numids)
        {
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). lolwut?" << endl;
        }
    }
}

/*! @brief Adds a sequence of [time, matrix] pairs to an actionatorid
 
     For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [data0, data1, ... , dataN], where each data is a vector.
     For a group actionator the data needs to
         id [time0, time1, ... , timeN], [data0, data1, ... , dataN], so each data element is given to the group with a single time
         [id0, id1, ... , idN] time [data0, data1, ... , dataN], so each actionator in the group receives a single data element at a sequence of times
         [id0, id1, ... , idN] [time0, time1, ... , timeN] data, so each actionator in the group receives the same data matrix at different times 
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<vector<vector<vector<float> > > >& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << " time.size():" << time.size() << " and data.size():" << data.size() << " must be the same." << endl;
        }
    }
    else
    {   // if actionatorid is a group
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else if (numids == data.size())
        {
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time, data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). lolwut?" << endl;
        }
    }
}

/*! @brief Adds a sequence of strings to an actionatorid
 
         For a single actionator the data needs to be formatted as [time0, time1, ... , timeN, [string0, string1, ... , stringN].
         For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN] [string0, string1, ... , stringN].
 
 @param actionatorid the id of the targetted actionator(s)
 @param time the sequence of times in ms
 @param data the sequence of strings
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<double>& time, const vector<string>& data)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << ")" << endl;
    #endif
    
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        if (time.size() == data.size())
        {
            for (size_t i=0; i<time.size(); i++)
                add(actionatorid, time[i], data[i]);
        }
        else
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). time and data need to have the same length when targeting a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, matrix] pairs to an actionatorid
 
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN], 
     so each data vector is given to the group with a single time vector
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, matrix] pairs with a single gain value to an actionatorid
 
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN], 
     so each data vector is given to the group with a single time vector
     
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
    @param gain the single gain value added to each data
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, float gain)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i], gain);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, matrix] pairs with a single gain value to an actionatorid
 
        For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN] [gain0, gain1, ..., gainN], 
        so each data vector is given to the group with a single time vector
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
    @param gain the sequence of gains added to each data
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<float>& gain)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids and gain.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i], gain[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " and gain.size():" << gain.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, matrix] pairs with a single gain value to an actionatorid
 
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN] [gain0, gain1, ..., gainN], 
     so each data vector is given to the group with a single time vector
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of vectors to give to the actionator
    @param gain the sequence of gains added to each data
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<vector<float> >& gain)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids and gain.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i], gain[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "," << gain << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " and gain.size():" << gain.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, vector<matrix>] pairs with a single gain value to an actionatorid
 
     For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN]
     so each data matrix is given to the group with a single time vector
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of matricies to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<vector<float> > >& data)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/*! @brief Adds a sequence of [vector<time>, vector<matrix>] pairs with a single gain value to an actionatorid
 
 For a group actionator the data needs to be formatted as [id0, id1, ... , idN] [time0, time1, ... , timeN], [data0, data1, ... , dataN]
 so each data matrix is given to the group with a single time vector
 
    @param actionatorid the id of the targetted actionator
    @param time the sequence of times in ms
    @param data the sequence of matricies to give to the actionator
 */
void NUActionatorsData::add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<vector<vector<float> > > >& data)
{
    vector<int>& ids = mapIdToIndices(actionatorid);
    size_t numids = ids.size();
    
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if actionatorid is a single actionator
        debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). Can not be used to add data to a single actionator." << endl;
    }
    else
    {   // if actionatorid is a group
        if (time.size() == numids and data.size() == numids)
        {   // if there is an entry for each member of the group
            for (size_t i=0; i<numids; i++)
                add(*m_ids[ids[i]], time[i], data[i]);
        }
        else
        {
            debug << "NUActionatorsData::add(" << actionatorid.Name << "," << time << "," << data << "). ";
            debug << "time.size():" << time.size() << " and data.size():" << data.size() << " must be ids.size():" << ids.size() << endl;
        }
    }
}

/******************************************************************************************************************************************
 Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

void NUActionatorsData::summaryTo(ostream& output)
{
    for (unsigned int i=0; i<m_available_ids.size(); i++)
        m_actionators[m_available_ids[i]].summaryTo(output);
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

