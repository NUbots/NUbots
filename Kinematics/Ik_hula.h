#ifndef IK_HULA_H
#define IK_HULA_H

#include "Ik_motion.h"
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include <iostream>

class Ik_hula: public Ik_motion
{
public:
    Ik_hula():Ik_motion(){}
    void initialiseHula(float height, float stance_width, float foot_angle, float radius)
    {
        m_height = height;
        m_stance_width = stance_width;
        m_foot_angle = foot_angle;
        m_radius = radius;
    }

    // Functions to get pose for each of the limbs.
    Matrix headPose()
    {
        return Matrix();
    }
    Matrix leftFootPose()
    {
        return footPose(true);
    }
    Matrix rightFootPose()
    {
        return footPose(false);
    }
    Matrix leftHandPose()
    {
        return Matrix();
    }
    Matrix rightHandPose()
    {
        return Matrix();
    }

protected:
    float m_height;
    float m_stance_width;
    float m_foot_angle;
    float m_radius;

    float current_angle()
    {
        float prog_time = m_current_time_ms - m_cycle_start_time_ms;
        float progress = prog_time / m_period_ms;
        return 2 * mathGeneral::PI * progress;
    }

    Matrix footPose(bool isLeft)
    {
        const float mult = isLeft?0.5:-0.5;
        const float half_stance = m_stance_width * mult;
        const float half_angle = m_foot_angle * mult;
        float angle = current_angle();
        float x_offset = m_radius * cos(angle);
        float y_offset = m_radius * sin(angle);

        Matrix pose = TransformMatrices::Translation(-x_offset, half_stance - y_offset, -m_height) * TransformMatrices::RotZ(half_angle);
        return pose;
    }
};

#endif // IK_MOTION_H
