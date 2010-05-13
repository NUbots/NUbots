/*! @file AbstractBehaviour.h
    @brief Declaration of an abstract behaviour class for other behaviours to inherit from
 
    @class AbstractBehaviour
    @brief Declaration of an abstract behaviour class

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

#ifndef ABSTRACTBEHAVIOUR_H
#define ABSTRACTBEHAVIOUR_H

#include <boost/circular_buffer.hpp>

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

class AbstractBehaviour
{
public:
    virtual ~AbstractBehaviour();
    
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
protected:
    AbstractBehaviour();
    bool preProcess(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
    virtual void doBehaviour() = 0;
    virtual void postProcess();
    
    void swapBehaviour(AbstractBehaviour* newbehaviour);
    
    bool longChestClick();
    bool singleChestClick();
    bool doubleChestClick();
    
    bool longLeftBumperClick();
    bool singleLeftBumperClick();
    bool doubleLeftBumperClick();
    
    bool longRightBumperClick();
    bool singleRightBumperClick();
    bool doubleRightBumperClick();
    
private:
    bool tripleChestClick();
    void removeStiffness();
    bool quadChestClick();
    void restartSoftware();
    
    void updateButtonValues();
    bool longClick(boost::circular_buffer<float> times, boost::circular_buffer<float> durations, float& previoustime);
    bool nClick(unsigned int n, boost::circular_buffer<float> times, boost::circular_buffer<float> durations, float& previoustime);

protected:
    double m_current_time;
    double m_previous_time;
    
    JobList* m_jobs;
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    FieldObjects* m_field_objects;
    GameInformation* m_game_info;
    TeamInformation* m_team_info;
    
    AbstractBehaviour* m_behaviour;         //!< a pointer to the current behaviour
    AbstractBehaviour* m_parent_behaviour;  //!< a pointer to the this behaviour's parent behaviour (that will be returned to if 4 clicks are detected, or this behaviour finishes)

private:
    
    // Private variables for button click detection
    float m_chest_state;
    float m_chest_previous_state;
    boost::circular_buffer<float> m_chest_times;
    boost::circular_buffer<float> m_chest_durations;
    
    float m_left_state;
    float m_left_previous_state;
    boost::circular_buffer<float> m_left_times;
    boost::circular_buffer<float> m_left_durations;
    
    float m_right_state;
    float m_right_previous_state;
    boost::circular_buffer<float> m_right_times;
    boost::circular_buffer<float> m_right_durations;
    
    float m_previous_long_chest_click;
    float m_previous_single_chest_click;
    float m_previous_double_chest_click;
    float m_previous_triple_chest_click;
    float m_previous_quad_chest_click;
    
    float m_previous_long_left_click;
    float m_previous_single_left_click;
    float m_previous_double_left_click;
    
    float m_previous_long_right_click;
    float m_previous_single_right_click;
    float m_previous_double_right_click;
};


#endif

