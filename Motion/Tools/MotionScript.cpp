/*! @file MotionCurves.cpp
    @brief Implementation of motion curve calculation class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "MotionScript.h"
#include "MotionFileTools.h"
#include "MotionCurves.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"

#include <sstream>
#include <cmath>
using namespace std;

MotionScript::MotionScript()
{
    m_is_valid = false;
}

MotionScript::MotionScript(string filename)
{
    m_name = filename;
    m_is_valid = load();
    m_uses_set = false;         // we can't set the m_uses_* because we don't have access to data or actions here
    m_play_start_time = 0;
}

string& MotionScript::getName()
{
    return m_name;
}

MotionScript::~MotionScript()
{
    
}

/*! @brief Returns the time when the script will be completed */
double MotionScript::timeFinished()
{
    return m_uses_last;
}

/*! @brief Returns true if the script uses the head */
bool MotionScript::usesHead()
{
    return m_uses_head;
}

/*! @brief Returns the time when the script finishes with the head */
double MotionScript::timeFinishedWithHead()
{
    if (not m_uses_head)
        return 0;
    else
        return m_uses_last_head;
}

/*! @brief Returns true if the script uses the left arm */
bool MotionScript::usesLArm()
{
    return m_uses_larm;
}

/*! @brief Returns the time when the script finishes with the left arm */
double MotionScript::timeFinishedWithLArm()
{
    if (not m_uses_larm)
        return 0;
    else
        return m_uses_last_larm;
}

/*! @brief Returns true if the script uses the right arm */
bool MotionScript::usesRArm()
{
    return m_uses_rarm;
}

/*! @brief Returns the time when the script finishes with the right arm */
double MotionScript::timeFinishedWithRArm()
{
    if (not m_uses_rarm)
        return 0;
    else
        return m_uses_last_rarm;
}

/*! @brief Returns true if the script ueses the left leg */
bool MotionScript::usesLLeg()
{
    return m_uses_lleg;
}

/*! @brief Returns the time the script finishes with the left leg */
double MotionScript::timeFinishedWithLLeg()
{
    if (not m_uses_lleg)
        return 0;
    else
        return m_uses_last_lleg;
}

/*! @brief Returns true if script uses the right leg */
bool MotionScript::usesRLeg()
{
    return m_uses_rleg;
}

/*! @brief Returns the time the script finishes with the right leg */
double MotionScript::timeFinishedWithRLeg()
{
    if (not m_uses_rleg)
        return 0;
    else
        return m_uses_last_rleg;
}

/*! @brief Sets the play speed
    @param speed the play speed (1 = normal speed, 0.1 = 10 times slower, 10 = 10 times faster)
 */
void MotionScript::setPlaySpeed(float speed)
{
    if (speed < 0.01)
        speed = 0.01;
    else if (speed > 100)
        speed = 100;
    m_playspeed = speed;
}

void MotionScript::play(NUSensorsData* data, NUActionatorsData* actions)
{
    if (not m_is_valid)
        return;
    if (not m_uses_set)
        setUses(actions);
    
    m_play_start_time = data->CurrentTime + 100;        // I add 100ms here because it can take upto 100ms to calculate a long and detailed motion curve
    vector<vector<double> > times = m_times;
    for (size_t i=0; i<times.size(); i++)
        for (size_t j=0; j<times[i].size(); j++)
            times[i][j] = times[i][j]/m_playspeed + m_play_start_time;
    
    vector<float> sensorpositions;
    data->getPosition(NUSensorsData::All, sensorpositions);
    if (m_return_to_start)
        appendReturnToStart(actions, times, m_positions, sensorpositions);
    
    updateLastUses(actions, times);
    
    MotionCurves::calculate(m_play_start_time, times, sensorpositions, m_positions, m_gains, m_smoothness, 10, m_curvetimes, m_curvepositions, m_curvevelocities, m_curvegains);
    actions->add(NUActionatorsData::All, m_curvetimes, m_curvepositions, m_curvegains);
    
    #if DEBUG_NUMOTION_VERBOSITY > 0
        debug << "MotionScript::play. Playing " << m_name << ". It uses ";
        if (m_uses_head)
            debug << "Head until " << timeFinishedWithHead() << ", ";
        if (m_uses_larm)
            debug << "LArm until " << timeFinishedWithLArm() << ", ";
        if (m_uses_rarm)
            debug << "RArm until " << timeFinishedWithRArm() << ", ";
        if (m_uses_lleg)
            debug << "LLeg until " << timeFinishedWithLLeg() << ", ";
        if (m_uses_rleg)
            debug << "RLeg until " << timeFinishedWithRLeg() << ", ";
        debug << "runs from " << m_play_start_time << " to " << timeFinished() << endl;
    #endif
    
    #if DEBUG_NUMOTION_VERBOSITY > 1
        debug << MotionFileTools::fromMatrix(m_curvetimes) << endl;
        debug << MotionFileTools::fromMatrix(m_curvepositions) << endl;
        debug << MotionFileTools::fromMatrix(m_curvevelocities) << endl;
        debug << MotionFileTools::fromMatrix(m_curvegains) << endl;
    #endif
}

bool MotionScript::load()
{
    ifstream file((CONFIG_DIR + "Motion/Scripts/" + m_name + ".num").c_str());
    if (!file.is_open())
    {
        errorlog << "MotionScript::load(). Unable to open " << m_name << endl;
        return false;
    }
    else
    {
        m_smoothness = MotionFileTools::toFloat(file);
        m_return_to_start = MotionFileTools::toBool(file);
        m_labels = MotionFileTools::toStringVector(file);
        if (m_labels.empty())
        {
            errorlog << "MotionScript::load(). Unable to load " << m_name << " the file labels are invalid " << endl;
            return false;
        }
        m_playspeed = 1.0;
        
        size_t numjoints = m_labels.size() - 1;
        m_times = vector<vector<double> >(numjoints, vector<double>());
        m_positions = vector<vector<float> >(numjoints, vector<float>());
        m_gains = vector<vector<float> >(numjoints, vector<float>());
        
        float time;
        vector<vector<float> > row;
        
        while (!file.eof())
        {
            MotionFileTools::toFloatWithMatrix(file, time, row);
            if (row.size() >= numjoints)
            {   // discard rows that don't have enough joints
                for (size_t i=0; i<numjoints; i++)
                {
                    if (row[i].size() > 0)
                    {   // if there is an entry then the first must be a position
                        m_times[i].push_back(1000*time);
                        m_positions[i].push_back(row[i][0]);
                        
                        // because the way the joint actionators are designed we must also specify a gain
                        if (row[i].size() > 1)
                        {   // if there is a second entry then it is the gain
                            m_gains[i].push_back(row[i][1]);
                        }
                        else
                        {   // however if there is no second entry we reuse the previous entry or use 100% if this is the first one
                            if (m_gains[i].empty())
                                m_gains[i].push_back(100.0);
                            else
                                m_gains[i].push_back(m_gains[i].back());
                        }
                        
                    }
                }
            }
            row.clear();
        }
        file.close();
        
        if (m_return_to_start)
        {   // Now a bit of hackery. When we want to return to start we need to add the placeholders for the return move
            for (size_t i=0; i<m_times.size(); i++)
            {
                if (not m_times[i].empty())
                {   // only add the placeholders if the joint is non empty
                    m_times[i].push_back(m_times[i].back());
                    m_positions[i].push_back(m_positions[i].back());
                    m_gains[i].push_back(m_gains[i].back());
                }
            }
        }
        return true;
    }
}


void MotionScript::setUses(NUActionatorsData* actions)
{
    m_uses_head = checkIfUses(actions->getIndices(NUActionatorsData::Head)); 
    m_uses_larm = checkIfUses(actions->getIndices(NUActionatorsData::LArm));
    m_uses_rarm = checkIfUses(actions->getIndices(NUActionatorsData::RArm));
    m_uses_lleg = checkIfUses(actions->getIndices(NUActionatorsData::LLeg));
    m_uses_rleg = checkIfUses(actions->getIndices(NUActionatorsData::RLeg));
    updateLastUses(actions, m_times);
    
    m_uses_set = true;
}

bool MotionScript::checkIfUses(const vector<int>& ids)
{
    for (size_t i=0; i<ids.size(); i++)
    {
        if (not m_times[ids[i]].empty())
            return true;
    }
    return false;
}

double MotionScript::findLastUse(const vector<int>& ids, const vector<vector<double> >& times)
{
    double lastuse = 0;
    for (size_t i=0; i<ids.size(); i++)
    {
        if (not times[ids[i]].empty() and times[ids[i]].back() > lastuse)
            lastuse = times[ids[i]].back();
    }
    return lastuse;
}


void MotionScript::updateLastUses(NUActionatorsData* actions, const vector<vector<double> >& times)
{
    m_uses_last_head = findLastUse(actions->getIndices(NUActionatorsData::Head), times);
    m_uses_last_larm = findLastUse(actions->getIndices(NUActionatorsData::LArm), times);
    m_uses_last_rarm = findLastUse(actions->getIndices(NUActionatorsData::RArm), times);
    m_uses_last_lleg = findLastUse(actions->getIndices(NUActionatorsData::LLeg), times);
    m_uses_last_rleg = findLastUse(actions->getIndices(NUActionatorsData::RLeg), times);
    
    m_uses_last = m_uses_last_head;
    if (m_uses_last_larm > m_uses_last)
        m_uses_last = m_uses_last_larm;
    if (m_uses_last_rarm > m_uses_last)
        m_uses_last = m_uses_last_rarm;
    if (m_uses_last_lleg > m_uses_last)
        m_uses_last = m_uses_last_lleg;
    if (m_uses_last_rleg > m_uses_last)
        m_uses_last = m_uses_last_rleg;
}


void MotionScript::appendReturnToStart(NUActionatorsData* actions, vector<vector<double> >& times, vector<vector<float> >& positions, const vector<float>& sensorpositions)
{
    if (m_uses_head)
    {
        vector<int>& headids = actions->getIndices(NUActionatorsData::Head);
        appendReturnLimbToStart(headids, times, positions, sensorpositions);
    }
    
    if (m_uses_larm)
    {
        vector<int>& larmids = actions->getIndices(NUActionatorsData::LArm);
        appendReturnLimbToStart(larmids, times, positions, sensorpositions);
    }
    
    if (m_uses_rarm)
    {
        vector<int>& rarmids = actions->getIndices(NUActionatorsData::RArm);
        appendReturnLimbToStart(rarmids, times, positions, sensorpositions);
    }
    
    if (m_uses_lleg)
    {
        vector<int>& llegids = actions->getIndices(NUActionatorsData::LLeg);
        appendReturnLimbToStart(llegids, times, positions, sensorpositions);
    }
    
    if (m_uses_rleg)
    {
        vector<int>& rlegids = actions->getIndices(NUActionatorsData::RLeg);
        appendReturnLimbToStart(rlegids, times, positions, sensorpositions);
    }
}

void MotionScript::appendReturnLimbToStart(const vector<int>& ids, vector<vector<double> >& times, vector<vector<float> >& positions, const vector<float>& sensorpositions)
{
    double maxtime = 0;
    size_t numids = ids.size();
    for (size_t i=0; i<numids; i++)
    {
        int id = ids[i];
        if (not times[id].empty())
        {
            double t = 1000*fabs(positions[id].back() - sensorpositions[id])/1.5;
            positions[id].back() = sensorpositions[id];
            if (t > maxtime)
                maxtime = t;
        }
    }
    
    for (size_t i=0; i<numids; i++)
    {
        int id = ids[i];
        if (not times[id].empty())
            times[id].back() = times[id][times[id].size()-2] + maxtime;
    }
}

ostream& operator<< (ostream& output, const MotionScript& p_script)
{
    return output;
}

ostream& operator<< (ostream& output, const MotionScript* p_script)
{
    return output;
}

istream& operator>> (istream& input, MotionScript& p_script)
{
    return input;
}

istream& operator>> (istream& input, MotionScript* p_script)
{
    return input;
}


