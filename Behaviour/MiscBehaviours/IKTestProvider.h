/*! @file IKTestProvider.h
    @brief Declaration of a behaviour provider for inverse kinematics testing purposes
 
    @class IKTestProvider
    @brief A special behaviour for developing walk engine's in
 

    @author Steven Nicklin
 
  Copyright (c) 2012 Steven Nicklin
 
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

#ifndef IKTESTPROVIDER_H
#define IKTESTPROVIDER_H

#include <vector>
#include "../BehaviourProvider.h"
class Ik_motion;
class NUInverseKinematics;

class IKTestProvider : public BehaviourProvider
{
public:
    IKTestProvider(Behaviour* manager);
    ~IKTestProvider();
protected:
    void doBehaviour();
    bool inPosition();
    bool m_moving_to_position;
    Ik_motion* m_current_ik_motion;
    NUInverseKinematics* m_ik;
    unsigned int m_sequence_count;
    double m_prev_time;
    std::vector<float> m_initial_left_arm;
    std::vector<float> m_initial_right_arm;
    std::vector<float> m_initial_left_leg;
    std::vector<float> m_initial_right_leg;

private:
};


#endif

