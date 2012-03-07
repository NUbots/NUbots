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

    Ik_motion(double current_time_ms, double period_ms, unsigned int num_cycle)
    {
        initialiseTiming(current_time_ms, period_ms, num_cycle);
    }

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

    bool complete()
    {
        return (!m_continuous && m_remaining_cycles == 0);
    }

    double endTime()
    {
        if(m_continuous) return 0;
        return m_cycle_start_time_ms + m_remaining_cycles * m_period_ms;
    }

    double remainingTime()
    {
        if(m_continuous) return 0;
        return m_cycle_start_time_ms + m_remaining_cycles * m_period_ms - m_current_time_ms;
    }

    // Functions to get pose for each of the limbs.
    virtual Matrix headPose() = 0;
    virtual Matrix leftFootPose() = 0;
    virtual Matrix rightFootPose() = 0;
    virtual Matrix leftHandPose() = 0;
    virtual Matrix rightHandPose() = 0;

protected:
    double m_cycle_start_time_ms;
    double m_current_time_ms;
    double m_period_ms;
    bool m_continuous;
    unsigned int m_remaining_cycles;

};

#endif // IK_MOTION_H
