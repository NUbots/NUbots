/*! @file QSCatch.h
    @brief Declaration of the catching state in the QSBallisticController
 
    @class QSCatch
    @brief The catchin state in the QSBallisticController

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef QSCatch_H
#define QSCatch_H

#include "Behaviour/BehaviourState.h"
#include "Infrastructure/NUData.h"
class QSBallisticController;

class QSCatch : public BehaviourState
{
public:
    QSCatch(const NUData::id_t& joint, const QSBallisticController* parent);
    ~QSCatch();
protected:
    void doState();
    BehaviourState* nextState();
    float normalDistribution(float mean, float sigma);
private:
    NUData::id_t m_joint;
    const QSBallisticController* m_parent;
    
    bool m_catch_issued;                  //!< A flag to indicate that the catch script has been issued
    float m_finish_time;                  //!< The time at which the catching finishes
    
    // Parameters of the catching phase
    float m_strength;                     //!< A scalar which determines the strength of the torque pulse to catch a micro-fall
    float m_catch_duration;               //!< The duration of the torque pulse for catching a microfall
    float m_tonic_duration;               //!< The duration of the static component of the torque pulse
    float m_catch_duration_variance;      //!< The variance in the duration of the catch
};


#endif

