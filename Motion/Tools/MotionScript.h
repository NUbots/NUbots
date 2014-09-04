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

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
class NUSensorsData;

#include <string>
#include <vector>

class MotionScript
{
public:
    MotionScript();
    MotionScript(std::string filename);
    ~MotionScript();
    
    void play(NUSensorsData* data, NUActionatorsData* actions);
    void setPlaySpeed(float speed);
    
    bool isValid();
    std::string& getName();
    
    double timeFinished();
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
    
    friend std::ostream& operator<< (std::ostream& output, const MotionScript& p_script);
    friend std::ostream& operator<< (std::ostream& output, const MotionScript* p_script);
    friend std::istream& operator>> (std::istream& input, MotionScript& p_script);
    friend std::istream& operator>> (std::istream& input, MotionScript* p_script);
    std::vector<std::vector<double> > m_times;     		//!< the times read in from the script file
    std::vector<std::vector<float> > m_positions;  		//!< the positions read in from the script file
    std::vector<std::vector<float> > m_gains;      		//!< the gains read in from the script file
protected:
    bool load();
    void setUses();
    bool checkIfUses(const std::vector<int>& ids);
    void updateLastUses(const std::vector<std::vector<double> >& times);
    double findLastUse(const std::vector<int>& ids, const std::vector<std::vector<double> >& times);
    
    void appendReturnToStart(std::vector<std::vector<double> >& times, std::vector<std::vector<float> >& positions, const std::vector<float>& sensorpositions);
    void appendReturnLimbToStart(const std::vector<int>& ids, std::vector<std::vector<double> >& times, std::vector<std::vector<float> >& positions, const std::vector<float>& sensorpositions);
protected:
    std::string m_name;                      		//!< the name of the script
    bool m_is_valid;                    		//!< true if the motion script file was loaded without error
    
    // a bunch of variables to keep track of when a script uses each limb
    double m_uses_last;                 		//!< the time in ms from the begining of the script that the script stops using all joints
    
    std::vector<int> m_head_indices;					//!< a list containing columns corresponding to head joints in the position matrix
    bool m_uses_head;                   		//!< true if the script uses any of the head joints
    double m_uses_last_head;            		//!< the time in ms from the begining of the script that the script uses the head
    
    std::vector<int> m_larm_indices;					//!< a list containing columns corresponding to larm joints in the position matrix
    bool m_uses_larm;                   		//!< true if the script uses any of the left arm joints
    double m_uses_last_larm;            		//!< the time in ms from the begining of the script that the script uses the left arm
    
    std::vector<int> m_rarm_indices;					//!< a list containing columns corresponding to rarm joints in the position matrix
    bool m_uses_rarm;                   		//!< true if the script uses any of the right arm joints
    double m_uses_last_rarm;            		//!< the time in ms from the begining of the script that the script uses the right arm
	
    std::vector<int> m_lleg_indices;					//!< a list containing columns corresponding to lleg joints in the position matrix
    bool m_uses_lleg;                   		//!< true if the script uses any of the left leg joints
    double m_uses_last_lleg;            		//!< the time in ms from the begining of the script that the script uses the left leg
    
    std::vector<int> m_rleg_indices;					//!< a list containing columns corresponding to rleg joints in the position matrix
    bool m_uses_rleg;                   		//!< true if the script uses any of the right leg joints
    double m_uses_last_rleg;            		//!< the time in ms from the begining of the script that the script uses the right leg

    // play speed
    float m_playspeed;
    double m_play_start_time;
    
    // original script data
    std::vector<std::string> m_labels;             		//!< the labels for each row
    float m_smoothness;                  		//!< the smoothness loaded from the script file
    bool m_return_to_start;              		//!< a flag to specify whether the script should return to the position when the script started playing
    
    // smoothed script data
    std::vector<std::vector<double> > m_curvetimes;		//!< the times to be given to the actionators
    std::vector<std::vector<float> > m_curvepositions;	//!< the positions to be given to the actionators
    std::vector<std::vector<float> > m_curvevelocities;	//!< the velocities
    std::vector<std::vector<float> > m_curvegains;		//!< the gains to be given to the actionators
};

#endif

