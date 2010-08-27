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

// Led actionators
NUActionatorsData::id_t NUActionatorsData::ChestLed = NUData::NumCommonIds + 0;
NUActionatorsData::id_t NUActionatorsData::LeftFootLed = NUData::NumCommonIds + 1;
NUActionatorsData::id_t NUActionatorsData::RightFootLed = NUData::NumCommonIds + 2;
NUActionatorsData::id_t NUActionatorsData::LeftEyeLed = NUData::NumCommonIds + 3;
NUActionatorsData::id_t NUActionatorsData::RightEyeLed = NUData::NumCommonIds + 4;
NUActionatorsData::id_t NUActionatorsData::LeftEarLed = NUData::NumCommonIds + 5;
NUActionatorsData::id_t NUActionatorsData::RightEarLed = NUData::NumCommonIds + 6;
// Led groups
NUActionatorsData::id_t NUActionatorsData::FaceLeds = NUData::NumCommonIds + 7;
NUActionatorsData::id_t NUActionatorsData::FeetLeds = NUData::NumCommonIds + 8;
NUActionatorsData::id_t NUActionatorsData::AllLeds = NUData::NumCommonIds + 9;
// Other actionators
NUActionatorsData::id_t NUActionatorsData::Sound = NUData::NumCommonIds + 10;
NUActionatorsData::id_t NUActionatorsData::Teleporter = NUData::NumCommonIds + 11;

/*! @brief Default constructor for a NUActionatorsData storage class.
 */
NUActionatorsData::NUActionatorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUActionatorsData::NUActionatorsData" << endl;
#endif
    CurrentTime = 0;
    m_id_to_indices.push_back(vector<int>(20,0));
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
    vector<string> names = simplifyNames(hardwarenames);
    // now I need to 
    //      - add the actionator to m_actionators
    //      - update m_id_to_indices
    
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


