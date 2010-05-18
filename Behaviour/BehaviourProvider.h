/*! @file BehaviourProvider.h
    @brief Declaration of an abstract behaviour provider class for other behaviours to inherit from
 
    @class BehaviourProvider
    @brief Declaration of an abstract behaviour provider class

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

#ifndef BEHAVIOURPROVIDER_H
#define BEHAVIOURPROVIDER_H

class Behaviour;

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <boost/circular_buffer.hpp>

class BehaviourProvider
{
public:
    virtual ~BehaviourProvider();
    
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
protected:
    BehaviourProvider(Behaviour* manager);
    bool preProcess(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
    virtual void doBehaviour() = 0;
    virtual void postProcess();
    
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
    void restartBehaviour();
    
    void updateButtonValues();
    bool longClick(boost::circular_buffer<float> times, boost::circular_buffer<float> durations, float& previoustime);
    bool nClick(unsigned int n, boost::circular_buffer<float> times, boost::circular_buffer<float> durations, float& previoustime);

protected:
    double m_current_time;
    double m_previous_time;
    
    Behaviour* m_manager;                   //!< a pointer to the behaviour manager
    
    JobList* m_jobs;                        //!< a local copy of the pointer to the public JobList
    NUSensorsData* m_data;                  //!< a local copy of the pointer to the public SensorData
    NUActionatorsData* m_actions;           //!< a local copy of the pointer to the public ActionatorsData
    FieldObjects* m_field_objects;          //!< a local copy of the pointer to the public world model
    GameInformation* m_game_info;           //!< a local copy of the pointer to the public GameInfo
    TeamInformation* m_team_info;           //!< a local copy of the pointer to the public TeamInfo

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

