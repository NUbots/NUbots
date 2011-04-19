/*! @file QSBallisticController.h
    @brief A simple ballistic controller

        .-- Ballistic Controller ----------.
        |    .---------.      .---------.  |
        |    |  Relax  | ---> |  Catch  |  |
        |    .---------.      .---------.  |
        |          ^               |       |
        |          |               |       |
        |          .---------------.       |
        |    (on catch-action completion)  |
        .----------------------------------.
 
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

#ifndef QSBallisticController_H
#define QSBallisticController_H

#include "Behaviour/BehaviourFSMState.h"
#include "Infrastructure/NUData.h"

class QSDelay;
class QSRelax;
class QSCatch;

#include <boost/circular_buffer.hpp>

class QSBallisticController : public BehaviourFSMState
{
public:
    static const float VelocityThreshold = 0.009;   // the velocity threshold for which a catch is triggered
    static const float Mass = 2.8;                  // the mass of the cycloid in kg
    static const float Height = 0.25;               // the height of the COM of the cycloid when standing
    static const float FrictionConstant = 3.3;      // the coefficient of friction (as determined through experiment)
public:
    QSBallisticController(const NUData::id_t& joint);
    ~QSBallisticController();
    
    QSRelax* getRelax() const;
    QSCatch* getCatch() const;
    
    bool relaxed() const;
    float getTargetEstimate() const;
    float getPosition() const;
    float getVelocity() const;
    float getAcceleration() const;
    float getTorque() const;
    
    void updateTargetEstimate();
protected:
    void doStateCommons();
private:
    NUData::id_t m_joint;
    QSDelay* m_delay;
    QSRelax* m_relax;
    QSCatch* m_catch;
    
    bool m_initialised;         // a flag to indicate whether the position and velocity have been initialised
    float m_position;           // filtered angular position of the joint under control
    float m_velocity;           // filtered angular velocity of the joint under control
    float m_acceleration;       // filtered angular acceleration of the joint under control
    
    boost::circular_buffer<float> m_target_estimate_buffer;
    float m_target_estimate;
};

#endif

