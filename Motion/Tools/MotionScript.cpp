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

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"

using namespace std;

MotionScript::MotionScript(string filename)
{
    m_name = filename;
    m_is_valid = load();
}

MotionScript::~MotionScript()
{
    
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
        vector<string> labels = MotionFileTools::toStringVector(file);
        vector<float> times;
        vector<vector<vector<float> > > zomg;
        while (!file.eof())
        {
            float time;
            vector<vector<float> > positions;
            MotionFileTools::toFloatWithMatrix(file, time, positions);
            if (positions.size() > 0)
            {
                times.push_back(time);
                zomg.push_back(positions);
            }
        }
        
        for (unsigned int i=0; i<labels.size(); i++)
            cout << labels[i] << ",";
        cout << endl;
        
        for (unsigned int i=0; i<times.size(); i++)
            cout << times[i] << ":" << MotionFileTools::fromMatrix(zomg[i]) << endl;
    }
}
