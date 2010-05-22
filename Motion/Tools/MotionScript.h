/*! @file MotionScript.h
    @brief Declaration of class to hold a motion script
 
    @class MotionScript
    @brief A class to hold a motion script
 
    Features:
        1. Both positions and gains can be specified in the .num file
        2. The smoothness and whether the script should return to the position from which
           It began can be specified in a file. A smoothness of 0 means only the linear
           interpolation of the hardware layer is used.
        3. Joints that have no entries in the .num file can be used by other modules/scripts
        4. The play speed can be specified online with setPlaySpeed
 
    TODO:
        1. 'Conditions'. In particular premature exit of the script
 
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

#ifndef MOTIONSCRIPT_H
#define MOTIONSCRIPT_H

#include "NUPlatform/NUActionators/NUActionatorsData.h"
class NUSensorsData;

#include <string>
#include <vector>
using namespace std;

class MotionScript
{
public:
    MotionScript();
    MotionScript(std::string filename);
    ~MotionScript();
    
    void play(NUSensorsData* data, NUActionatorsData* actions);
    void setPlaySpeed(float speed);
    
    string& getName();
    
    bool usesHead();
    double timeFinishedWithHead();
    bool usesLArm();
    double timeFinishedWithLArm();
    bool usesRArm();
    double timeFinishedWithRArm();
    bool usesLLeg();
    double timeFinishedWithLLeg();
    bool usesRLeg();
    double timeFinishedWithRLeg();
    
    friend ostream& operator<< (ostream& output, const MotionScript& p_script);
    friend ostream& operator<< (ostream& output, const MotionScript* p_script);
    friend istream& operator>> (istream& input, MotionScript& p_script);
    friend istream& operator>> (istream& input, MotionScript* p_script);
protected:
    bool load();
    void setUses(NUActionatorsData* actions);
    bool checkIfUses(const vector<NUActionatorsData::joint_id_t>& ids);
    void updateLastUses(NUActionatorsData* actions, const vector<vector<double> >& times);
    double findLastUse(const vector<NUActionatorsData::joint_id_t>& ids, const vector<vector<double> >& times);
    
    void appendReturnToStart(NUActionatorsData* actions, vector<vector<double> >& times, vector<vector<float> >& positions, const vector<float>& sensorpositions);
    void appendReturnLimbToStart(const vector<NUActionatorsData::joint_id_t>& ids, vector<vector<double> >& times, vector<vector<float> >& positions, const vector<float>& sensorpositions);
protected:
    string m_name;                      //!< the name of the script
    bool m_is_valid;                    //!< true if the motion script file was loaded without error
    
    bool m_uses_set;                    //!< true if the m_uses_* have been set
    bool m_uses_head;                   //!< true if the script uses any of the head joints
    double m_uses_last_head;            //!< the time in ms from the begining of the script that the script uses the head
    bool m_uses_larm;                   //!< true if the script uses any of the left arm joints
    double m_uses_last_larm;            //!< the time in ms from the begining of the script that the script uses the left arm
    bool m_uses_rarm;                   //!< true if the script uses any of the right arm joints
    double m_uses_last_rarm;            //!< the time in ms from the begining of the script that the script uses the right arm
    bool m_uses_lleg;                   //!< true if the script uses any of the left leg joints
    double m_uses_last_lleg;            //!< the time in ms from the begining of the script that the script uses the left leg
    bool m_uses_rleg;                   //!< true if the script uses any of the right leg joints
    double m_uses_last_rleg;            //!< the time in ms from the begining of the script that the script uses the right leg

    float m_playspeed;
    double m_play_start_time;
    
    // original script data
    vector<string> m_labels;             //!< the labels for each row
    float m_smoothness;                  //!< the smoothness loaded from the script file
    bool m_return_to_start;              //!< a flag to specify whether the script should return to the position when the script started playing
    vector<vector<double> > m_times;     //!< the times read in from the script file
    vector<vector<float> > m_positions;  //!< the positions read in from the script file
    vector<vector<float> > m_gains;      //!< the gains read in from the script file
    
    // smoothed script data (these curves are only valid for a playspeed of 1)
    vector<vector<double> > m_curvetimes;
    vector<vector<float> > m_curvepositions;
    vector<vector<float> > m_curvevelocities;
    vector<vector<float> > m_curvegains;
};

#endif

