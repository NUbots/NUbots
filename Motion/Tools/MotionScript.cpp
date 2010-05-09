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

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUSystem.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"

#include <sstream>
using namespace std;

MotionScript::MotionScript()
{
    m_is_valid = false;
}

MotionScript::MotionScript(string filename)
{
    m_name = filename;
    m_is_valid = load();
    m_playspeed = 1;
    if (m_is_valid)
        calculateCurve();
}

string& MotionScript::getName()
{
    return m_name;
}

MotionScript::~MotionScript()
{
    
}

void MotionScript::play(NUSensorsData* data, NUActionatorsData* actions)
{
    cout << "playing at " << NUSystem::getThreadTime() << endl;
    
    vector<float> sensorpositions;
    data->getJointPositions(NUSensorsData::AllJoints, sensorpositions);
    vector<vector<double> > times = m_times;
    for (size_t i=0; i<times.size(); i++)
        for (size_t j=0; j<times[i].size(); j++)
            times[i][j] += data->CurrentTime;
    MotionCurves::calculate(data->CurrentTime, times, sensorpositions, m_positions, m_gains, m_smoothness, 10, m_curvetimes, m_curvepositions, m_curvevelocities, m_curvegains);
    actions->addJointPositions(NUActionatorsData::AllJoints, m_curvetimes, m_curvepositions, m_curvevelocities, m_curvegains);
    cout << "finished at " << NUSystem::getThreadTime() << endl;
    
    for (size_t i=0; i<m_curvetimes.size(); i++)
    {
        if (m_curvetimes[i].size() > 0)
        {
            stringstream filename;
            filename << "joint" << i << ".csv";
            ofstream file(filename.str().c_str()); 
            for (size_t j=0; j<m_curvetimes[i].size(); j++)
                file << m_curvetimes[i][j] << ", " << m_curvepositions[i][j] << ", " << 1000*m_curvevelocities[i][j] << endl;
        }
        cout << m_curvetimes[i].size() << " " << MotionFileTools::fromVector(m_curvetimes[i]) << endl;
        cout << m_curvepositions[i].size() << " " << MotionFileTools::fromVector(m_curvepositions[i]) << endl;
        cout << m_curvevelocities[i].size() << " " << MotionFileTools::fromVector(m_curvevelocities[i]) << endl;
        cout << m_curvegains[i].size() << " " << MotionFileTools::fromVector(m_curvegains[i]) << endl;
    }
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
                                m_gains[i].push_back(m_gains[i].front());
                        }
                        
                    }
                }
            }
            row.clear();
        }
        return true;
    }
}

void MotionScript::calculateCurve()
{
    vector<float> zero(m_times.size(), 0);
    MotionCurves::calculate(0, m_times, zero, m_positions, m_gains, m_smoothness, 10, m_curvetimes, m_curvepositions, m_curvevelocities, m_curvegains);
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


