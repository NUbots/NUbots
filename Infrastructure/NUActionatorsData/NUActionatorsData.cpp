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

#include "debug.h"
#include "debugverbositynuactionators.h"

int a_curr_id = NUData::NumCommonIds.Id+1;
vector<NUActionatorsData::id_t*> NUActionatorsData::m_ids;
// Led actionators
const NUActionatorsData::id_t NUActionatorsData::ChestLed(a_curr_id++, "ChestLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::LFootLed(a_curr_id++, "LFootLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::RFootLed(a_curr_id++, "RFootLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::LEyeLed(a_curr_id++, "LEyeLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::REyeLed(a_curr_id++, "REyeLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::LEarLed(a_curr_id++, "LEarLed", NUActionatorsData::m_ids);
const NUActionatorsData::id_t NUActionatorsData::REarLed(a_curr_id++, "REarLed", NUActionatorsData::m_ids);
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
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUActionatorsData::NUActionatorsData" << endl;
#endif
    CurrentTime = 0;
    
    m_ids.insert(m_ids.begin(), NUData::Ids.begin(), NUData::Ids.end());
    
    for (size_t i=0; i<m_ids.size(); i++)
        m_actionators.push_back(Actionator(m_ids[i]->Name));
    m_id_to_indices = vector<vector<int> >(m_ids.size(), vector<int>());
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
    vector<string> names = standardiseNames(hardwarenames);
    for (size_t i=0; i<names.size(); i++)
    {	// for each name compare it to the name of that actionator
        for (size_t j=0; j<m_ids.size(); j++)
        {
            if (*(m_ids[j]) == names[i])
                m_id_to_indices[j].push_back(j);
        }
    }
    
    for (size_t i=0; i<m_ids.size(); i++)
    {	// fill in the groups
        for (size_t j=0; j<m_ids.size(); j++)
        {
            if (not m_id_to_indices[j].empty() and belongsToGroup(*m_ids[j], *m_ids[i]))
                m_id_to_indices[i].push_back(j);
        }
    }
    
    for (size_t j=0; j<m_id_to_indices.size(); j++)
    {
        debug << m_ids[j]->Name << "->[";
        for (size_t k=0; k<m_id_to_indices[j].size(); k++)
            debug << m_ids[m_id_to_indices[j][k]]->Name << " ";
        debug << "]" << endl;
    }
    // now I need to 
    //      - add the actionator to m_actionators
    //      - update m_id_to_indices
    
}

/*! @brief Returns true if member belongs to group 
    @param member the single id
    @param group the group id
    @return true if member belongs to group 
 */
bool NUActionatorsData::belongsToGroup(const id_t& member, const id_t& group)
{
    bool belongstonudata = NUData::belongsToGroup(member, group);
    if (belongstonudata)
        return true;
    else
    {
        if (group == FaceLeds)
        {
            if (member.Name.find("Led") != string::npos and (member.Name.find("Eye") != string::npos  or member.Name.find("Mouth") != string::npos))
                return true;
            else
                return false;
        }
        else if (group == FeetLeds)
        {
            if (member.Name.find("Led") != string::npos and member.Name.find("Foot") != string::npos)
                return true;
            else
                return false;
        }
        else if (group == AllLeds)
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

/******************************************************************************************************************************************
 Get Methods
 ******************************************************************************************************************************************/
/*! @brief Pre processes the data to be ready for copying to hardware communication
 */
void NUActionatorsData::preProcess(double currenttime)
{
    CurrentTime = currenttime;
    /*for (unsigned int i=0; i<m_all_actionators.size(); i++)
        m_all_actionators[i]->preProcess();
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
        m_all_string_actionators[i]->preProcess();*/
}

/*! @brief Post processes the data after sending it to the hardware communications (Remove all of the completed actionator points)
    @param currenttime all actionator points that have times before this one are assumed to have been completed, and they will be removed
 */
void NUActionatorsData::postProcess()
{
    /*for (unsigned int i=0; i<m_all_actionators.size(); i++)
        m_all_actionators[i]->postProcess(currenttime);
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
        m_all_string_actionators[i]->postProcess(currenttime);*/
}

vector<int>& NUActionatorsData::getIndices(id_t actionatorid)
{
    return m_id_to_indices[0];
}

size_t NUActionatorsData::getSize(id_t actionatorid)
{
    return 0;
}

/******************************************************************************************************************************************
 Set Methods
 ******************************************************************************************************************************************/
/* A single time and a single data.
    for each actionator in actionator id 
        add [time,data] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, float data)
{
}

/* A single time and a single data.
    for each actionator in actionator id 
        add [time,[data,gain]] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, float data, float gain)
{
}

/* A single time, and a vector of data 
    if actionatorid is a group
        if data.size matches group.size
            add(actionatorid[i], time, data[i])
        else
            add(actionatorid[i], time, data)
    else
        add [time,data] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, const vector<float>& data)
{
}

/* */
void NUActionatorsData::add(id_t actionatorid, double time, const vector<float>& data, float gain)
{
}

void NUActionatorsData::add(id_t actionatorid, double time, const vector<float>& data, const vector<float>& gain)
{
}

/* A single time, and a vector<vector> of data
    if actionator is a group
        if data.size matches group.size
            add(actionatorid[i], time, data[i])
        else
            add(actionatorid[i], time, data)
    else
        add [time,data] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, const vector<vector<float> >& data)
{
}

/* A single time, and a vector<vector<vector>> of data
    if actionator is a group
        if data.size matches group.size
            add(actionatorid[i], time, data[i])
        else
            add(actionatorid[i], time, data)
    else
        add [time,data] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, const vector<vector<vector<float> > >& data)
{
}

/* A single time, and a string of data
    for each actionator in actionator id 
        add [time,data] to actionator
 */
void NUActionatorsData::add(id_t actionatorid, double time, const string& data)
{
}


// ------------------------------------------------------------------------------------------- Vector time functions
/* A vector of time and a vector of data
    if actionator is a group
        if time.size matches group.size
            if group.size matches data.size
                add(actionatorid[i], time[i], data[i])
            else
                add(actionatorid[i], time[i], data)
        else 
            makes no sense
    else
        if time.size matches data.size
            add(actionator, time[i], data[i])
        else
            makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<float>& data)
{
}

void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<float>& data, float gain)
{
}

/* A vector of time and a vector of data
     if actionator is a group
         if time.size matches group.size
             if group.size matches data.size matches gain.size
                 add(actionatorid[i], time[i], data[i], gain[i])
             else 
                 makes no sense
     else
         if time.size matches data.size matches gain.size
             add(actionator, time[i], data[i], gain[i])
         else
             makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<float>& data, const vector<float>& gain)
{
}

/* A vector of time and a vector<vector> of data
    if actionator is a group
        if times.size matches data.size
            add(actionator (group), time[i], data[i])
        else if group.size matches data.size
            add(actionator[i], time, data[i])
        else if time.size matches group.size
            add(actionator[i], time[i], data)
        else
            makes no sense
    else
        if time.size matches data.size
            add(actionator, time[i], data[i]
        else
            makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<vector<float> >& data)
{
}

/* A vector of time and a vector<vector<vector>> of data
    if actionator is a group
        if times.size matches data.size
            add(actionator (group), time[i], data[i])
        else if group.size matches data.size
            add(actionator[i], time, data[i])
        else if time.size matches group.size
            add(actionator[i], time[i], data)
        else
            makes no sense
    else
        if time.size matches data.size
            add(actionator, time[i], data[i])
        else
            makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<vector<vector<float> > >& data)
{
}

/*! A vector of time and a vector<vector<vector<vector>>> of data
     if actionator is a group
         if times.size matches data.size
             add(actionator (group), time[i], data[i])
         else if group.size matches data.size
             add(actionator[i], time, data[i])
         else if time.size matches group.size
             add(actionator[i], time[i], data)
         else
             makes no sense
     else
         if time.size matches data.size
             add(actionator, time[i], data[i])
         else
             makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<vector<vector<vector<float> > > >& data)
{
}

/* A vector of time and a vector of strings
    if actionator is a group
        if time.size matches group.size matches data.size
            add(actionatorid[i], time[i] , data[i]]
        else
            makes no sense
    else
        if time.size matches data.size
            add(actionator, time[i], data[i])
        else
            makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<double>& time, const vector<string>& data)
{
}

// ------------------------------------------------------------------------------------------- Matrix time functions
/* A matrix of time and a matrix of data
    if actionator is a group:
        if time.size matches group.size matches data.size
            add(actionator[i], time[i], data[i])
        else
            makes no sense
    else
        makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data)
{
}

void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<float>& gain)
{
}

void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, float gain)
{
}

/* A matrix of time and a matrix of data
     if actionator is a group:
         if time.size matches group.size matches data.size matches gain.size
             add(actionator[i], time[i], data[i], gain[i])
         else
             makes no sense
     else
         makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<vector<float> >& gain)
{
}


/* A matrix of time and a vector<vector<vector>>
    if actionator is a group:
        if time.size matches group.size matches data.size
            add(actionator[i], time[i], data[i])
        else
            makes no sense
    else
        makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<vector<float> > >& data)
{
}

/* A matrix of time and a vector<vector<vector<vector>>>
     if actionator is a group:
         if time.size matches group.size matches data.size
             add(actionator[i], time[i], data[i])
         else
             makes no sense
     else
         makes no sense
 */
void NUActionatorsData::add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<vector<vector<float> > > >& data)
{
}

/******************************************************************************************************************************************
 Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

void NUActionatorsData::summaryTo(ostream& output)
{
    /*if (m_all_actionators.size() == 0 && m_all_string_actionators.size() == 0)
        output << "NONE!" << endl;
    for (unsigned int i=0; i<m_all_actionators.size(); i++)
    {
        if (not m_all_actionators[i]->isEmpty()) 
            m_all_actionators[i]->summaryTo(output);
    }
    for (unsigned int i=0; i<m_all_string_actionators.size(); i++)
    {
        if (not m_all_string_actionators[i]->isEmpty())
            m_all_string_actionators[i]->summaryTo(output);
    }*/
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


