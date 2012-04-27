/*! @file ik_motion.h
    @brief Declaration of a parent class for cyclic inverse kinematic motions.

    @class Ik_motion
    @brief A parent class used to make cyclic imnverse kinematics motions.


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

#ifndef IK_MOTION_H
#define IK_MOTION_H

#include "Tools/Math/Matrix.h"

class Ik_motion
{
public:
    enum position
    {
        start,
        end
    };

    Ik_motion(){}

    /*!
      Assignment constructor. This sets up the timing information for the cyclic motion.
      @param current_time_ms The current time.
      @param period_ms The time period of the cyclic motion.
      @param num_cycle The number of cycles to be completed until the motion is finished.
      */
    Ik_motion(double current_time_ms, double period_ms, unsigned int num_cycle)
    {
        initialiseTiming(current_time_ms, period_ms, num_cycle);
    }

    /*!
      Function to initialise the timing information for the cyclic motion.
      @param current_time_ms The current time.
      @param period_ms The time period of the cyclic motion.
      @param num_cycle The number of cycles to be completed until the motion is finished.
      */
    void initialiseTiming(double current_time_ms, double period_ms, unsigned int num_cycle)
    {
        m_current_time_ms = current_time_ms;
        m_cycle_start_time_ms = current_time_ms;
        m_period_ms = period_ms;
        m_remaining_cycles = num_cycle;
        if(m_remaining_cycles == 0)
            m_continuous = true;
        else
            m_continuous = false;
    }

    /*!
      Overloaded seek function. This function seeks to a pre-defined position in the sequence.
      Either start, or end.
      @param pos The target seek position.
      @return False if the motion is complete. True if there remains motions to be performed.
      */
    bool seek(position pos)
    {
        if(pos = start)
            m_current_time_ms = m_cycle_start_time_ms;
        else
        {
            m_current_time_ms = m_cycle_start_time_ms + m_period_ms;
            m_remaining_cycles = 0;
        }
        return !complete();
    }

    /*!
      Overloaded seek function. This function seeks forwars in the sequence by the amount of time specified.
      @param deltaTime_ms The amount of time that the sequence will be moved forward.
      @return False if the motion is complete. True if there remains motions to be performed.
      */
    bool seek(double deltaTime_ms)
    {
        if(complete()) return false;
        double next_cycle_start = m_cycle_start_time_ms + m_period_ms;
        m_current_time_ms += deltaTime_ms;
        if(m_current_time_ms >= next_cycle_start)
        {
            if(!m_continuous) --m_remaining_cycles;
            m_cycle_start_time_ms = next_cycle_start;
        }
        return !complete();
    }

    /*!
      Determines if the current motion has been completed.
      @return True if the motion is complete. False if there remains motions to be performed.
      */
    bool complete()
    {
        return (!m_continuous && m_remaining_cycles == 0);
    }

    /*!
      Gives the end time of the current motion.
      @return The time that the motion will end.
      */
    double endTime()
    {
        if(m_continuous) return 0;
        return m_cycle_start_time_ms + m_remaining_cycles * m_period_ms;
    }

    /*!
      Gives the amount of time remaining in the current motion.
      @return The time remaining for the motion.
      */
    double remainingTime()
    {
        if(m_continuous) return 0;
        return m_cycle_start_time_ms + m_remaining_cycles * m_period_ms - m_current_time_ms;
    }

    // Functions to get pose for each of the limbs.
    /*!
      Virtual function used to detemine the torso relative pose for the head.
      @return A matrix defining the 3-dimensional pose of the head relative to the torso.
      */
    virtual Matrix headPose() = 0;

    /*!
      Virtual function used to detemine the torso relative pose for the left foot.
      @return A matrix defining the 3-dimensional pose of the left foot relative to the torso.
      */
    virtual Matrix leftFootPose() = 0;

    /*!
      Virtual function used to detemine the torso relative pose for the right foot.
      @return A matrix defining the 3-dimensional pose of the right foot relative to the torso.
      */
    virtual Matrix rightFootPose() = 0;

    /*!
      Virtual function used to detemine the torso relative pose for the left hand.
      @return A matrix defining the 3-dimensional pose of the left hand relative to the torso.
      */
    virtual Matrix leftHandPose() = 0;

    /*!
      Virtual function used to detemine the torso relative pose for the right hand.
      @return A matrix defining the 3-dimensional pose of the right hand relative to the torso.
      */
    virtual Matrix rightHandPose() = 0;

protected:
    double m_cycle_start_time_ms;       //! Starting time.
    double m_current_time_ms;           //! Current time.
    double m_period_ms;                 //! Period of motion cycle.
    bool m_continuous;                  //! True if motion continues indefinately. False if there are a fixed number of cycles.
    unsigned int m_remaining_cycles;    //! Total remaining cycles to perform.

};

#endif // IK_MOTION_H
